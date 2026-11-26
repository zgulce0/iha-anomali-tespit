"""GPS spoofing toplu testinin analizi: ROC egrisi ve tespit suresi tablosu.

Girdi: exporter/spoof_testi.py'nin kaydettigi klasor (ham.csv.gz + etiketler.csv + olaylar.csv).

Ne yapar:
  1. Ham kaydi, canli sistemdeki kodun AYNISIYLA (exporter/spoof_tespit.py, SpoofIzleyici) yeniden oynatir.
     Mesajlar geldikleri sirayla verilir; izleme kosulu (motorlar calisiyor, irtifa >= 2 m, 3D fix,
     >= 6 uydu) exporter'dakiyle aynidir.
  2. Simulatorun GPS'i hatasizdir. Bu yuzden ayni kayit bir de GERCEKCI GPS HATASI eklenerek oynatilir:
       yatay konum : ikinci derece Gauss-Markov, eksen basina std 1,7 m (NEO-M8N 2,0 m CEP) + 0,05 m beyaz
       hiz         : 0,05 m/s beyaz (veri sayfasi)
       yukseklik   : ikinci derece Gauss-Markov, std = DIKEY_KAT x 1,7 m (varsayim: 1,5) + 0,2 m beyaz
       barometre   : yavas kayma (Gauss-Markov, std 0,5 m, 600 sn) + 0,1 m beyaz
       Gezinmenin hizi (zaman sabiti) bilinmiyor: 30, 100 ve 300 sn denenir, her biri TOHUM kez.
  3. Testlerin degerleri (NIS'ler, kestirilen sapmalar) kaydedilir; karar esikleri bir carpan (k) ile
     birlikte buyutulup kucultulur. Chi-kare esiklerinde k ile carpmak yanlis alarm olasiligini
     degistirmekle aynidir (2 serbestlik derecesinde alfa -> alfa^k). k = 1 canli sistemin ayaridir.
     Not: filtrenin kendini sifirlama kurali (NIS > 9,21 surekli 0,8 sn) degismez; sadece karar degisir.
  4. Her saldiri bir deneme sayilir: saldiri suresince (geri alma ani haric) herhangi bir test
     tetiklenirse "yakalandi", tespit suresi = ilk tetik - komutun gonderildigi an.
     Saldirisiz zaman (izlemenin calistigi her an; saldiri suresi ve bitisinden sonraki 30 sn haric;
     kalkis, inis, daireye giris dahil) 30 sn'lik pencerelere bolunur; tetik olan pencere "yanlis alarm" sayilir. Ayrica saatte yanlis alarm sayisi (30 sn
     icinde tekrar eden tetikler tek alarm sayilir; canli sistemdeki alarm kilidi gibi).

Cikti: <klasor>/sonuc/
  tespit_tablosu.csv / .md   her saldiri icin yakalama orani ve tespit suresi (ortalama ± std)
  yanlis_alarm.csv           saldirisiz ucusta saatte yanlis alarm ve pencere orani
  roc.png                    ROC egrileri (saldiri grubu basina), k = 1 isaretli
  esik_secimi.png            esik carpanina gore tespit orani ve yanlis alarm orani
  dikkat.csv                 her testin sinirinin %50'sini gecme orani (saldirisiz) - "dikkat" seviyesi icin
  ozet.txt                   hepsinin metin ozeti

Kullanim (proje kokunden):
    venv/bin/python analiz/spoof_roc.py veri/spoof_testi_<tarih> [baska klasorler...] [--tohum 10] [--tau 30,100,300] [--dikey-kat 1.5]
    Birden fazla klasor verilirse (ornegin hover ve daire ayri kaydedildiyse) birlikte degerlendirilir;
    cikti ilk klasorun icine yazilir.
"""
import argparse
import gzip
import math
import os
import sys
import zlib
from multiprocessing import Pool

import numpy as np
import pandas as pd

KOK = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, os.path.join(KOK, 'exporter'))
import spoof_tespit as ST  # noqa: E402

TEMIZ_ATLA_SN = 30.0      # saldiri bittikten sonraki 30 sn saldirisiz sayilmaz: saldirgan GPS'i geri birakinca konum
                          # ters yonde sicrar (bu da gercek bir saldiri anidir), arac geri doner, filtre oturur.
                          # Canli sistemde de alarm son tetikten sonra 30 sn acik kalir.
PENCERE_SN = 30.0
KILIT_SN = 30.0
SON_PAY_SN = 0.0          # sadece saldiri suresindeki tetik sayilir (geri alma anindaki ters sicrama sayilmaz)
K_IZGARA = np.unique(np.round(np.concatenate([np.geomspace(0.25, 8.0, 41), [1.0]]), 4))

SIG_B, SIG_W, SIG_V = 1.7, 0.05, 0.05        # yatay (spoof_kalibrasyon.py ile ayni)
SIG_WZ = 0.2
BARO_DRIFT, BARO_TAU, BARO_W = 0.5, 600.0, 0.1


# ---------------------------------------------------------------------------
# OKUMA
# ---------------------------------------------------------------------------
def ham_oku(yol):
    """ham.csv.gz -> mesaj listesi [(w, kod, alanlar...)]. Yarida kalmis dosyayi da okur."""
    satirlar = []
    with gzip.open(yol, 'rt') as f:
        try:
            for satir in f:
                p = satir.rstrip('\n').split(',')
                if len(p) < 3:
                    continue
                satirlar.append((float(p[0]), p[1], p[2:]))
        except (EOFError, zlib.error, OSError):
            print('  not: ham kayit yarida kesilmis, okunabilen kismi kullaniliyor')
    return satirlar


def yukle(klasor):
    msj = ham_oku(os.path.join(klasor, 'ham.csv.gz'))
    et = pd.read_csv(os.path.join(klasor, 'etiketler.csv'))
    ol = pd.read_csv(os.path.join(klasor, 'olaylar.csv'))
    return msj, et, ol


# ---------------------------------------------------------------------------
# GERCEKCI HATA
# ---------------------------------------------------------------------------
def gm2(t, rng, sig, tau, boyut):
    """Ikinci derece Gauss-Markov (iki birinci derece suzgec), baslangic std sig. t: zamanlar (sn)."""
    n = len(t)
    out = np.zeros((n, boyut))
    if n == 0:
        return out
    x1 = rng.normal(0, sig, boyut); x2 = x1.copy()
    onceki = t[0]
    for i in range(n):
        a = math.exp(-max(t[i] - onceki, 0.0) / (tau / 2)); onceki = t[i]
        x1 = a * x1 + math.sqrt(1 - a * a) * rng.normal(0, sig, boyut)
        x2 = a * x2 + (1 - a) * x1
        out[i] = x2
    return out


def gm1(t, rng, sig, tau):
    n = len(t)
    out = np.zeros(n)
    x = rng.normal(0, sig); onceki = t[0] if n else 0
    for i in range(n):
        a = math.exp(-max(t[i] - onceki, 0.0) / tau); onceki = t[i]
        x = a * x + math.sqrt(1 - a * a) * rng.normal(0, sig)
        out[i] = x
    return out


def hata_uret(msj, tau, tohum, dikey_kat):
    """GPS ve barometre mesajlari icin eklenecek hatalar (mesaj sirasina gore dizi)."""
    rng = np.random.default_rng(tohum)
    tg = np.array([m[0] for m in msj if m[1] == 'G'])
    tb = np.array([m[0] for m in msj if m[1] == 'B'])
    yatay = gm2(tg, rng, SIG_B, tau, 2) + rng.normal(0, SIG_W, (len(tg), 2))
    hiz = rng.normal(0, SIG_V, (len(tg), 2))
    dikey = gm2(tg, rng, dikey_kat * SIG_B, tau, 1)[:, 0] + rng.normal(0, SIG_WZ, len(tg))
    baro = gm1(tb, rng, BARO_DRIFT, BARO_TAU) + rng.normal(0, BARO_W, len(tb)) if len(tb) else np.zeros(0)
    return yatay, hiz, dikey, baro


# ---------------------------------------------------------------------------
# YENIDEN OYNATMA (exporter.py ile ayni akis)
# ---------------------------------------------------------------------------
def oynat(msj, hata=None):
    """Mesajlari canli sistemdeki gibi SpoofIzleyici'ye verir. Her GPS adimi icin test degerlerini dondurur."""
    iz = ST.SpoofIzleyici(kilit_sn=KILIT_SN)
    armed, irtifa, cos_lat = 0, None, None
    gi = bi = 0
    kayit = []
    onceki_aktif = False
    onceki_t = None
    for w, kod, p in msj:
        if kod == 'I':
            iz.raw_imu(int(p[1]), int(p[2]), int(p[3]), saat=int(p[0]) / 1e6)
        elif kod == 'A':
            iz.attitude(float(p[1]), float(p[2]), float(p[3]))
        elif kod == 'B':
            pr = float(p[1])
            if hata is not None and pr > 0:
                if iz.p0 is None:
                    iz.p0 = pr
                h = ST.basinc_yukseklik(pr, iz.p0) + hata[3][bi]
                pr = iz.p0 * (1 - h / 44330.77) ** (1 / 0.190263)
            bi += 1
            iz.baro(pr, saat=int(p[0]) / 1000)
        elif kod == 'P':
            irtifa = int(p[4]) / 1000.0
        elif kod == 'H':
            armed = 1 if (int(p[0]) & 128) else 0
        elif kod == 'G':
            tus, lat, lon, alt, vel, cog, fix, uydu = (int(x) for x in p[:8])
            if hata is not None:
                if cos_lat is None and lat:
                    cos_lat = math.cos(math.radians(lat / 1e7))
                dn, de = hata[0][gi]
                lat += int(round(dn / ST.M_DERECE * 1e7))
                lon += int(round(de / (ST.M_DERECE * (cos_lat or 1.0)) * 1e7))
                alt += int(round(hata[2][gi] * 1000))
                if vel != 65535:
                    r = math.radians(cog / 100.0) if cog != 65535 else 0.0
                    vn = vel / 100 * math.cos(r) + hata[1][gi][0]
                    ve = vel / 100 * math.sin(r) + hata[1][gi][1]
                    vel = int(round(math.hypot(vn, ve) * 100))
                    cog = int(round(math.degrees(math.atan2(ve, vn)) % 360 * 100))
            gi += 1
            saglikli = fix >= 3 and uydu >= 6
            kapi = armed == 1 and irtifa is not None and irtifa >= ST.SpoofIzleyici.MIN_IRTIFA_M and saglikli
            t = tus / 1e6
            ayni = onceki_t is not None and t - onceki_t <= 1e-3   # ayni GPS olcumu tekrar geldi (GPS 5 Hz, mesaj 10 Hz)
            onceki_t = t
            s = iz.gps(t, lat, lon, vel, cog, kapi, saat=t, alt_mm=alt)
            if s is not None and not ayni:
                z = iz.son_z
                kayit.append((w, t, not onceki_aktif, s['nis'], s['nis_v'], s['sapma'],
                              iz.imu_sayisi < iz.MIN_IMU, z['nis_z'] if z else 0.0, z['fark_z'] if z else 0.0,
                              s['anlik'], s['hiz'], s['yavas'], s['irtifa']))
            if not ayni:
                onceki_aktif = s is not None
    return pd.DataFrame(kayit, columns=['w', 't', 'basla', 'nis', 'nis_v', 'sapma', 'seyrek', 'nis_z', 'fark_z',
                                        'anlik', 'hiz', 'yavas', 'irtifa'])


# ---------------------------------------------------------------------------
# KARAR (esik carpani k ile)
# ---------------------------------------------------------------------------
def _seri(b, basla):
    """Art arda True sayisi; basla=True olan adimda sayac sifirdan baslar."""
    idx = np.arange(len(b))
    son_yanlis = np.where(~b, idx, np.where(basla, idx - 1, -1))
    return idx - np.maximum.accumulate(son_yanlis)


def kararlar(d, k):
    """Her GPS adiminda hangi test tetiklendi (k: esik carpani). Sozluk: test adi -> bool dizi."""
    det, dz = ST.SpoofDedektor(), ST.IrtifaDedektor()
    basla = d.basla.values
    anlik = _seri(d.nis.values > ST.CHI2_99_2SD * k, basla) >= det.anlik_n
    hiz = _seri((d.nis_v.values > det.esik_v * k) & ~d.seyrek.values, basla) >= det.hiz_n
    yavas = (d.sapma.values / det.sig_b) ** 2 > det.esik_b * k
    z_anlik = _seri(d.nis_z.values > dz.esik_z * k, basla) >= dz.anlik_n
    z_yavas = (d.fark_z.values / dz.sig_bz) ** 2 > dz.esik_bz * k
    return {'anlik': anlik, 'hiz': hiz, 'yavas': yavas, 'irtifa': z_anlik | z_yavas}


def temiz_maske(w, et):
    """Saldirisiz zaman: izlemenin calistigi her an, saldiri suresi ve sonrasindaki TEMIZ_ATLA_SN haric
    (kalkis, inis, daireye giris dahil)."""
    temiz = np.ones(len(w), bool)
    for r in et[~et.tip.isin(['normal', 'toparlanma'])].itertuples():
        temiz[np.searchsorted(w, r.bas_w):np.searchsorted(w, r.bit_w + TEMIZ_ATLA_SN)] = False
    return temiz


def degerlendir(d, et, k):
    """Bir oynatma + bir k icin: saldiri basina tespit, temiz pencerelerde yanlis alarm."""
    kr = kararlar(d, k)
    tetik = kr['anlik'] | kr['hiz'] | kr['yavas'] | kr['irtifa']
    w = d.w.values
    sonuc = []
    for r in et[~et.tip.isin(['normal', 'toparlanma'])].itertuples():
        i0, i1 = np.searchsorted(w, r.bas_w), np.searchsorted(w, r.bit_w + SON_PAY_SN)
        hit = np.nonzero(tetik[i0:i1])[0]
        if len(hit):
            j = i0 + hit[0]
            hangi = next(ad for ad in ('hiz', 'anlik', 'yavas', 'irtifa') if kr[ad][j])
            sonuc.append((r.Index, True, w[j] - r.bas_w, hangi))
        else:
            sonuc.append((r.Index, False, np.nan, None))
    # temiz zaman: kesintisiz temiz araliklar PENCERE_SN'lik pencerelere bolunur
    temiz = temiz_maske(w, et)
    pencereler, temiz_sn = [], 0.0
    sinir = np.nonzero(np.diff(np.concatenate([[0], temiz.astype(int), [0]])))[0]
    for i0, i1 in zip(sinir[::2], sinir[1::2]):
        j = i0
        while j < i1:
            # izleme kesintisi (1 sn'den uzun bosluk) pencereyi boler
            son = min(np.searchsorted(w, w[j] + PENCERE_SN), i1)
            bosluk = np.nonzero(np.diff(w[j:son]) > 1.0)[0]
            if len(bosluk):
                son = j + bosluk[0] + 1
            if w[son - 1] - w[j] >= 5.0:
                pencereler.append(bool(tetik[j:son].any()))
                temiz_sn += w[son - 1] - w[j]
            j = son
    # saatte yanlis alarm: temiz zamandaki tetiklerden, oncekinden KILIT_SN sonra gelenler
    tw = w[tetik]
    yeni = np.ones(len(tw), bool)
    yeni[1:] = np.diff(tw) > KILIT_SN
    yanlis = int((yeni & temiz[tetik]).sum())
    hangi_yanlis = {}
    for j in np.nonzero(tetik)[0][yeni & temiz[tetik]]:
        ad = next(a for a in ('hiz', 'anlik', 'yavas', 'irtifa') if kr[a][j])
        hangi_yanlis[ad] = hangi_yanlis.get(ad, 0) + 1
    return sonuc, pencereler, temiz_sn, yanlis, hangi_yanlis


def kosu(arg):
    """Bir klasorun bir oynatmasi (hatasiz ya da bir tau/tohum) ve tum k degerleri."""
    klasor, et, tau, tohum, dikey_kat = arg
    msj = ham_oku(os.path.join(klasor, 'ham.csv.gz'))
    hata = None if tau is None else hata_uret(msj, tau, tohum, dikey_kat)
    d = oynat(msj, hata)
    if d.empty:
        return tau, tohum, None
    sonuclar = {k: degerlendir(d, et, k) for k in K_IZGARA}
    # k = 1'de vektorlu karar dedektorun kendi kararina uyuyor mu? (dogrulama)
    kr = kararlar(d, 1.0)
    uyum = {ad: float((kr[ad] == d[ad].values).mean()) for ad in ('anlik', 'hiz', 'yavas', 'irtifa')}
    # dikkat seviyesi icin: saldirisiz zamanda her test sinirin %50'sini ne siklikla geciyor
    temiz = temiz_maske(d.w.values, et)
    det, dz = ST.SpoofDedektor(), ST.IrtifaDedektor()
    oran = {}
    for ad, deger, sinir in [('konum NIS', d.nis, ST.CHI2_99_2SD), ('hiz NIS', d.nis_v, det.esik_v),
                             ('GPS sapmasi (m)', d.sapma, det.sapma_siniri),
                             ('yukseklik NIS', d.nis_z, dz.esik_z),
                             ('GPS-baro farki (m)', d.fark_z.abs(), dz.fark_siniri)]:
        v = deger.values[temiz]
        oran[ad] = (float((v > 0.5 * sinir).mean()) if len(v) else np.nan,
                    float(np.percentile(v, 99) / sinir) if len(v) else np.nan)
    return tau, tohum, dict(sonuclar=sonuclar, uyum=uyum, dikkat=oran, adim=len(d))


# ---------------------------------------------------------------------------
# RAPOR
# ---------------------------------------------------------------------------
TEST_ADI = {'anlik': 'konum sıçraması (konum NIS)', 'hiz': 'hız tutarsızlığı (hız NIS)',
            'yavas': 'GPS sapması', 'irtifa': 'yükseklik (GPS-barometre)'}


def grup(tip):
    return {'ani': 'Ani sıçrama', 'yavas': 'Yavaş kayma', 'tutarli': 'Tutarlı kayma (konum + hız)',
            'irtifa_ani': 'Yükseklik', 'irtifa_yavas': 'Yükseklik'}[tip]


def birim(tip):
    return 'm' if tip in ('ani', 'irtifa_ani') else 'm/s'


def tr(x, b=1):
    return '-' if x is None or (isinstance(x, float) and np.isnan(x)) else f'{x:.{b}f}'.replace('.', ',')


def ardupilot_uyarisi(et, ol):
    """Her saldiri icin ArduPilot'un kendi uyarisi ('GPS Glitch or Compass error') kac sn sonra geldi."""
    uy = ol[ol.olay.str.contains('ARDUPILOT', na=False)
            & ol.olay.str.contains('glitch', case=False, na=False)
            & ~ol.olay.str.contains('cleared', case=False, na=False)]
    out = {}
    for r in et[~et.tip.isin(['normal', 'toparlanma'])].itertuples():
        u = uy[(uy.w >= r.bas_w) & (uy.w <= r.bit_w + SON_PAY_SN)]
        out[r.Index] = (u.w.iloc[0] - r.bas_w, u.olay.iloc[0].replace('ARDUPILOT: ', '')) if len(u) else (np.nan, '')
    return out


def tablo(et, kosular, ap):
    """Saldiri turu x buyukluk x desen: yakalama orani, tespit suresi (hatasiz ve gercekci)."""
    sal = et[~et.tip.isin(['normal', 'toparlanma'])].copy()
    satirlar = []
    for (desen, tip, deger), g in sal.groupby(['desen', 'tip', 'deger'], sort=False):
        satir = {'desen': desen, 'saldiri': grup(tip), 'tip': tip, 'buyukluk': f'{tr(deger, 2 if deger < 1 else 0)} {birim(tip)}',
                 'deneme': len(g)}
        for ad, filtre in [('hatasiz', lambda t: t is None), ('gercekci', lambda t: t is not None)]:
            sure, ok, test = [], [], {}
            for tau, tohum, s in kosular:
                if s is None or not filtre(tau):
                    continue
                for idx, yak, sn, hangi in s['sonuclar'][1.0][0]:
                    if idx in g.index:
                        ok.append(yak)
                        if yak:
                            sure.append(sn); test[hangi] = test.get(hangi, 0) + 1
            satir[f'{ad}_yakalama_%'] = 100 * np.mean(ok) if ok else np.nan
            satir[f'{ad}_sure_ort'] = np.mean(sure) if sure else np.nan
            satir[f'{ad}_sure_std'] = np.std(sure) if sure else np.nan
            satir[f'{ad}_test'] = TEST_ADI[max(test, key=test.get)] if test else ''
        aps = [ap[i][0] for i in g.index if not np.isnan(ap[i][0])]
        satir['ardupilot_uyari_%'] = 100 * len(aps) / len(g)
        satir['ardupilot_uyari_sure'] = np.mean(aps) if aps else np.nan
        satirlar.append(satir)
    return pd.DataFrame(satirlar)


def md_tablo(t):
    s = ['| Desen | Saldırı | Büyüklük | Deneme | Yakalama (hatasız GPS) | Süre (hatasız GPS) | '
         'Yakalama (gerçekçi GPS) | Süre (gerçekçi GPS) | İlk tetikleyen test | ArduPilot uyarısı |',
         '|---|---|---|---|---|---|---|---|---|---|']
    for r in t.to_dict('records'):
        s.append(f"| {r['desen']} | {r['saldiri']} | {r['buyukluk']} | {r['deneme']} | %{tr(r['hatasiz_yakalama_%'], 0)} | "
                 f"{tr(r['hatasiz_sure_ort'])} ± {tr(r['hatasiz_sure_std'])} sn | %{tr(r['gercekci_yakalama_%'], 0)} | "
                 f"{tr(r['gercekci_sure_ort'])} ± {tr(r['gercekci_sure_std'])} sn | {r['gercekci_test'] or r['hatasiz_test']} | "
                 f"%{tr(r['ardupilot_uyari_%'], 0)}" + (f" ({tr(r['ardupilot_uyari_sure'])} sn)" if r['ardupilot_uyari_%'] > 0 else '') + ' |')
    return '\n'.join(s)


def roc_ciz(et, kosular, yol, taus):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    sal = et[~et.tip.isin(['normal', 'toparlanma'])]
    gruplar = [('Ani sıçrama', ['ani'], '#1f5fbf', 'o'), ('Yavaş kayma', ['yavas'], '#d98200', 's'),
               ('Tutarlı kayma', ['tutarli'], '#1b9e8f', '^'), ('Yükseklik', ['irtifa_ani', 'irtifa_yavas'], '#b8327a', 'D')]
    fig, eksenler = plt.subplots(1, len(taus), figsize=(4.6 * len(taus), 4.4), sharey=True, squeeze=False)
    egriler = {}
    for ax, tau in zip(eksenler[0], taus):
        ks = [s for t_, _, s in kosular if t_ == tau and s is not None]
        for ad, tipler, renk, isaret in gruplar:
            idx = set(sal[sal.tip.isin(tipler)].index)
            if not idx:
                continue
            fpr, tpr = [], []
            for k in K_IZGARA:
                ok = [yak for s in ks for i, yak, _, _ in s['sonuclar'][k][0] if i in idx]
                pen = [p for s in ks for p in s['sonuclar'][k][1]]
                fpr.append(np.mean(pen) if pen else np.nan); tpr.append(np.mean(ok) if ok else np.nan)
            fpr, tpr = np.array(fpr), np.array(tpr)
            egriler[(tau, ad)] = (fpr, tpr)
            sira = np.argsort(fpr)
            ax.plot(fpr[sira], tpr[sira], color=renk, lw=2, label=ad)
            j = int(np.nonzero(K_IZGARA == 1.0)[0][0])
            ax.plot(fpr[j], tpr[j], marker=isaret, ms=8, color=renk, mec='white', mew=1.5, zorder=5)
        ax.plot([0, 1], [0, 1], color='#9aa3ad', lw=1, ls=':')
        ax.set_xscale('symlog', linthresh=0.01)
        ax.set_xlim(-0.0015, 1); ax.set_ylim(0, 1.02)      # 0'daki noktalar eksen cizgisinin ustune binmesin
        ax.set_xticks([0, 0.01, 0.1, 1]); ax.set_xticklabels(['0', '0,01', '0,1', '1'])
        ax.set_title(f'GPS gezinme zaman sabiti {tau:g} sn', fontsize=11)
        ax.set_xlabel('Yanlış alarm oranı (saldırısız 30 sn pencere)')
        ax.grid(color='#e3e6ea', lw=0.8); ax.set_axisbelow(True)
        for kenar in ('top', 'right'):
            ax.spines[kenar].set_visible(False)
    eksenler[0][0].set_ylabel('Tespit oranı (saldırı başına)')
    eksenler[0][0].legend(loc='lower right', frameon=False, fontsize=9)
    fig.suptitle('Gerçekçi GPS hatasıyla ROC eğrileri (işaretli nokta: kullanılan eşik, k = 1)', fontsize=12)
    fig.tight_layout()
    fig.savefig(yol, dpi=160)
    plt.close(fig)

    # Ikinci grafik: esik carpanina gore tespit orani ve yanlis alarm orani (ayri eksenler, ayni k)
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 4.2), sharex=True)
    tau = taus[len(taus) // 2]
    for ad, _, renk, isaret in gruplar:
        if (tau, ad) in egriler:
            a1.plot(K_IZGARA, egriler[(tau, ad)][1], color=renk, lw=2, label=ad)
    a2.plot(K_IZGARA, egriler[next(k for k in egriler if k[0] == tau)][0], color='#4a5563', lw=2)
    for ax, baslik in ((a1, 'Tespit oranı (saldırı başına)'), (a2, 'Yanlış alarm oranı (saldırısız 30 sn pencere)')):
        ax.set_xscale('log'); ax.axvline(1.0, color='#9aa3ad', lw=1, ls='--')
        ax.set_xticks([0.25, 0.5, 1, 2, 4, 8]); ax.set_xticklabels(['0,25', '0,5', '1', '2', '4', '8'])
        ax.minorticks_off()
        ax.set_title(baslik, fontsize=11); ax.set_xlabel('Eşik çarpanı k (1 = kullanılan eşik; büyüdükçe eşik yükselir)')
        ax.grid(color='#e3e6ea', lw=0.8); ax.set_axisbelow(True)
        for kenar in ('top', 'right'):
            ax.spines[kenar].set_visible(False)
    a1.set_ylim(0, 1.02); a1.legend(loc='lower left', frameon=False, fontsize=9)
    a2.set_ylim(bottom=0)
    fig.suptitle(f'Eşik seçimi: k küçülünce tespit artar ama yanlış alarm da artar (GPS gezinme zaman sabiti {tau:g} sn)',
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(os.path.dirname(yol), 'esik_secimi.png'), dpi=160)
    plt.close(fig)
    return egriler


def main():
    ap_ = argparse.ArgumentParser()
    ap_.add_argument('klasor', nargs='+')
    ap_.add_argument('--tohum', type=int, default=10, help='her zaman sabiti icin kac farkli rastgele GPS hatasi')
    ap_.add_argument('--tau', default='30,100,300')
    ap_.add_argument('--dikey-kat', type=float, default=1.5, help='GPS dikey hatasi / yatay hatasi')
    ap_.add_argument('--islem', type=int, default=os.cpu_count() or 1, help='paralel islem sayisi')
    a = ap_.parse_args()
    taus = [float(x) for x in a.tau.split(',') if x]
    print('Okunuyor:')
    etler, ap = [], {}
    for i, kl in enumerate(a.klasor):
        e = pd.read_csv(os.path.join(kl, 'etiketler.csv'))
        e = e[e.sonuc == 'tamam'].copy()
        e['kayit'] = i
        etler.append(e)
    et = pd.concat(etler, ignore_index=True)      # her etiketin tum klasorlerde tek bir numarasi olur
    isler = []
    for i, kl in enumerate(a.klasor):
        e = et[et.kayit == i]
        ol = pd.read_csv(os.path.join(kl, 'olaylar.csv'))
        ap.update(ardupilot_uyarisi(e, ol))
        print(f'  {kl}: {len(e)} etiket ({(~e.tip.isin(["normal", "toparlanma"])).sum()} saldiri)')
        isler += [(kl, e, None, 0, a.dikey_kat)] + [(kl, e, tau, 1000 * int(tau) + 97 * i + s, a.dikey_kat)
                                                    for tau in taus for s in range(a.tohum)]
    print(f'  {len(isler)} oynatma ({a.islem} paralel)...')
    with Pool(a.islem) as havuz:
        kosular = []
        for i, r in enumerate(havuz.imap_unordered(kosu, isler), 1):
            kosular.append(r)
            print(f'    {i}/{len(isler)}', end='\r', flush=True)
    print()
    cikti = os.path.join(a.klasor[0], 'sonuc')
    os.makedirs(cikti, exist_ok=True)

    t = tablo(et, kosular, ap)
    t.to_csv(os.path.join(cikti, 'tespit_tablosu.csv'), index=False)
    md = md_tablo(t)
    with open(os.path.join(cikti, 'tespit_tablosu.md'), 'w') as f:
        f.write(md + '\n')

    # yanlis alarm (k = 1)
    ya = []
    for tau in [None] + taus:
        ks = [s for t_, _, s in kosular if t_ == tau and s is not None]
        if not ks:
            continue
        pen = [p for s in ks for p in s['sonuclar'][1.0][1]]
        sn = sum(s['sonuclar'][1.0][2] for s in ks)
        n = sum(s['sonuclar'][1.0][3] for s in ks)
        hangi = {}
        for s in ks:
            for k_, v in s['sonuclar'][1.0][4].items():
                hangi[k_] = hangi.get(k_, 0) + v
        ya.append({'gps': 'hatasiz' if tau is None else f'gercekci, tau {tau:g} sn', 'temiz_saat': sn / 3600,
                   'yanlis_alarm': n, 'saatte': n / max(sn, 1) * 3600, 'pencere_orani': np.mean(pen) if pen else np.nan,
                   'hangi_test': hangi})
    ya = pd.DataFrame(ya)
    ya.to_csv(os.path.join(cikti, 'yanlis_alarm.csv'), index=False)

    # dikkat seviyesi
    dk = []
    for tau in [None] + taus:
        ks = [s for t_, _, s in kosular if t_ == tau and s is not None]
        for ad in (ks[0]['dikkat'] if ks else {}):
            dk.append({'gps': 'hatasiz' if tau is None else f'gercekci, tau {tau:g} sn', 'test': ad,
                       'sinirin_yarisini_gecme_%': 100 * np.nanmean([s['dikkat'][ad][0] for s in ks]),
                       'yuzde99_deger_sinira_orani': np.nanmean([s['dikkat'][ad][1] for s in ks])})
    dk = pd.DataFrame(dk)
    dk.to_csv(os.path.join(cikti, 'dikkat.csv'), index=False)

    egriler = roc_ciz(et, [k for k in kosular if k[0] is not None], os.path.join(cikti, 'roc.png'), taus) if taus else {}

    uyum = pd.DataFrame([dict(s['uyum'], tau=t_, tohum=o) for t_, o, s in kosular if s is not None])
    with open(os.path.join(cikti, 'ozet.txt'), 'w') as f:
        def yaz(x=''):
            print(x); f.write(x + '\n')
        yaz('Klasor: ' + ', '.join(a.klasor))
        yaz(f'Oynatma: 1 hatasiz + {len(taus)} zaman sabiti x {a.tohum} tohum; dikey hata = {a.dikey_kat} x yatay')
        yaz(f'Dogrulama (k=1 vektorlu karar = dedektorun kendi karari): en kotu uyum '
            + ', '.join(f'{c} %{100 * uyum[c].min():.3f}' for c in ('anlik', 'hiz', 'yavas', 'irtifa')))
        yaz('\nYANLIS ALARM (saldirisiz ucus, k = 1)')
        for r in ya.itertuples():
            yaz(f'  {r.gps:28s} {r.temiz_saat:6.2f} saat  saatte {r.saatte:5.2f}  pencere orani %{100 * r.pencere_orani:5.2f}  {r.hangi_test or ""}')
        yaz('\nTESPIT (k = 1)')
        yaz(md)
        yaz('\nDIKKAT SEVIYESI ICIN: saldirisiz ucusta testin sinirinin yarisini gecme orani')
        for r in dk.to_dict('records'):
            yaz(f"  {r['gps']:28s} {r['test']:20s} %{r['sinirin_yarisini_gecme_%']:6.2f}   %99 degeri / sinir = {r['yuzde99_deger_sinira_orani']:.2f}")
        if egriler:
            yaz('\nROC (k: esik carpani; FPR: saldirisiz 30 sn pencerelerde yanlis alarm orani, TPR: tespit orani)')
            for (tau, ad), (fpr, tpr) in egriler.items():
                yaz(f'  tau {tau:g} | {ad:14s} ' + '  '.join(f'k={k:g}: {100 * x:.1f}/{100 * y:.0f}'
                                                        for k, x, y in zip(K_IZGARA, fpr, tpr) if k in (0.5, 0.75, 1.0, 1.5, 2.0)))
    print(f'\nCikti: {cikti}')


if __name__ == '__main__':
    main()
