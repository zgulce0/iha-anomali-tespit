"""GPS spoofing dedektorunun kalibrasyonu: GERCEKCI GPS HATASIYLA deneme (Monte Carlo).

Neden: simulatorun (SITL) GPS'i hatasizdir. Gercek bir alici ise birkac metre yavasca gezinir.
Dedektoru sadece hatasiz GPS'te denemek iyimser sonuc verir. Bu script kayitli ucuslarin GPS
olcumlerine gercekci hata ekler ve dedektoru defalarca calistirir:

  GPS konum hatasi : yavas gezinme, eksen basina std SIG_B = 1,7 m (NEO-M8N veri sayfasi 2,0 m CEP,
                     CEP = 1,1774 * std) ve ikinci derece Gauss-Markov (zaman sabiti TAU sn)
                     + beyaz titresim 0,05 m
  GPS hiz hatasi   : beyaz, eksen basina 0,05 m/s (veri sayfasi: hiz dogrulugu 0,05 m/s)
  Gezinmenin hizi (TAU) veri sayfasinda yok; 30, 100 ve 300 sn denenir.

Cikti, yeni dedektor (exporter/spoof_tespit.py) ve eski dedektor (analiz/spoof_eski_dedektor.py) icin:
  - temiz ucusta saatte yanlis alarm ve hangi testten geldigi
  - normal ucusta NIS ortalamalari (chi-kare 2 sd icin teori: 2)
  - her saldiri senaryosu icin yakalama orani ve tespit suresi (ortalama ± std)

Kullanim (proje kokunden):
    venv/bin/python analiz/spoof_kalibrasyon.py veri/spoof_ucuncu_*.csv veri/spoof_besinci_*.csv [--tekrar 5] [--tau 30,100,300]
Not: Bu bir modeldir, gercek alicinin hatasi bundan farkli olabilir. Sonuclar "bu varsayimlar altinda" gecerlidir.
"""
import argparse
import collections
import os
import sys

import numpy as np
import pandas as pd

KOK = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, os.path.join(KOK, 'exporter'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spoof_tespit import SpoofDedektor            # noqa: E402
from spoof_eski_dedektor import EskiSpoofDedektor  # noqa: E402
import spoof_analiz as A                           # noqa: E402

SIG_B, SIG_W, SIG_V = 1.7, 0.05, 0.05


def gps_hatasi(t, rng, tau, sig_b=SIG_B, sig_w=SIG_W, sig_v=SIG_V):
    """Konum hatasi (n x 2, m) ve hiz hatasi (n x 2, m/s). Konum: iki birinci derece suzgecten gecmis
    beyaz gurultu (ikinci derece Gauss-Markov), baslangic durumu da std sig_b."""
    n = len(t)
    dt = np.diff(t, prepend=t[0]); dt[0] = np.median(dt[1:]) if n > 1 else 0.2
    b = np.zeros((n, 2)); x1 = rng.normal(0, sig_b, 2); x2 = x1.copy()
    for i in range(n):
        a = np.exp(-dt[i] / (tau / 2))
        x1 = a * x1 + np.sqrt(1 - a * a) * rng.normal(0, sig_b, 2)
        x2 = a * x2 + (1 - a) * x1
        b[i] = x2
    return b + rng.normal(0, sig_w, (n, 2)), rng.normal(0, sig_v, (n, 2))


def kos(d, yap, dp, dv):
    det = yap(); cikti = []
    for i, r in enumerate(d.itertuples()):
        cikti.append(det.adim(r.zaman_sn, (r.gps_n + dp[i, 0], r.gps_e + dp[i, 1]),
                              (r.gvn + dv[i, 0], r.gve + dv[i, 1]), (r.an, r.ae)))
    o = pd.DataFrame(cikti, index=d.index)
    return o.assign(zaman_sn=d.zaman_sn, etiket=d.etiket, temiz=d.temiz)


def degerlendir(ad, yap, veriler, tau, tekrar, tohum):
    rng = np.random.default_rng(tohum)
    yanlis = collections.Counter(); sure = 0.0; nis = []; nis_v = []
    tespit = collections.defaultdict(list)
    for _ in range(tekrar):
        for d in veriler:
            if tau is None:
                dp = dv = np.zeros((len(d), 2))
            else:
                dp, dv = gps_hatasi(d.zaman_sn.values, rng, tau)
            o = kos(d, yap, dp, dv); t = o[o.temiz]
            bas = t.alarm & ~t.alarm.shift(fill_value=False)
            yanlis.update(t[bas].tetik.fillna('?'))
            sure += (t.zaman_sn.diff().clip(upper=1.0).fillna(0)).sum()
            nis += list(t.nis); nis_v += list(t.nis_v)
            for et, g in o[o.etiket.str.startswith('spoof')].groupby('etiket', sort=False):
                h = g[g.alarm]
                tespit[et].append(h.zaman_sn.iloc[0] - g.zaman_sn.iloc[0] if len(h) else np.nan)
    kosul = 'simulatorun GPS\'i (hatasiz)' if tau is None else f'gercekci GPS, gezinme zaman sabiti {tau} sn'
    print(f'\n{ad} | {kosul} | {tekrar} tekrar, {sure / 3600:.2f} saat temiz ucus')
    print(f'  yanlis alarm: saatte {sum(yanlis.values()) / max(sure, 1) * 3600:.1f}  {dict(yanlis) or ""}')
    print(f'  normal ucusta NIS ortalamasi: konum {np.mean(nis):.2f}, hiz {np.mean(nis_v):.2f}  (teori: 2)')
    for et, v in tespit.items():
        v = np.array(v); ok = ~np.isnan(v)
        sure_m = f'{np.nanmean(v):6.1f} ± {np.nanstd(v):4.1f} sn' if ok.any() else '      -'
        print(f'    {et:22s} yakalama %{ok.mean() * 100:3.0f}   {sure_m}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv', nargs='+')
    ap.add_argument('--tekrar', type=int, default=5)
    ap.add_argument('--tau', default='30,100,300')
    ap.add_argument('--eskisiz', action='store_true', help='eski dedektoru calistirma')
    a = ap.parse_args()
    veriler = []
    for yol in a.csv:
        if yol.endswith('_olaylar.csv'):
            continue
        d = pd.read_csv(yol); d = d[d.irtifa_m > 8].reset_index(drop=True)
        veriler.append(A.bolge(A.yerel(d)))
    adaylar = [('YENI', lambda: SpoofDedektor())]
    if not a.eskisiz:
        adaylar.append(('ESKI', lambda: EskiSpoofDedektor()))
    for ad, yap in adaylar:
        degerlendir(ad, yap, veriler, None, 1, 0)
        for tau in [float(x) for x in a.tau.split(',')]:
            degerlendir(ad, yap, veriler, tau, a.tekrar, int(tau))


if __name__ == '__main__':
    main()
