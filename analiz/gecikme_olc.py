"""Uctan uca gecikme olcumu: ariza -> exporter -> Alertmanager -> Telegram.

Kullanim:
    python3 analiz/gecikme_olc.py --ucus ~/gecikme_hepsi.txt --am ~/am_hepsi.log
        --exporter ~/iha-anomali-tespit/exporter.log [--csv sonuc.csv]
    (iki satir tek komuttur; bosluklarla ayrilmis tek satir olarak yaz)

Girdiler (hepsi duz metin):
    --ucus      otomatik_ucus.py ciktisi (tee ile kaydedilmis; satirlarda saat var)
    --am        Alertmanager gunlugu (docker logs --timestamps; token maskelenmis)
    --exporter  exporter.log (opsiyonel; 'ANOMALI ALARMI BASLADI' satirlari)

Her ariza icin olculenler:
    exp~   ariza -> exporter ALARM verdi. Exporter saatleri tam saniye oldugu icin +-0.5 sn
    AM     ariza -> Alertmanager'a 'active' alarm ulasti (ana olcum, ms hassasiyetli)
    prom~  exporter -> Alertmanager (Prometheus: kazima + kural degerlendirme + for), +-0.5 sn
    tg     ariza -> Telegram'a ALARM mesaji gonderildi. Mesaj yoksa 'yok': Alertmanager
           arizayi onceki alarmla ayni olay sayip birlestirdi (gruplama)
    coz    ariza bitisi -> Telegram'a COZULDU mesaji gonderildi

Zamanlarin hepsi duvar saatidir. WSL duvar saati Windows ile birebir uyusuyor
(30 sn'de Windows 30.28, WSL duvar 30.26; WSL ic sayaci 31.16 hizli aktigi icin
kullanilmadi).
"""
import argparse
import csv
import re
import statistics
import sys
from datetime import datetime, timezone

UCUS_SATIR = re.compile(r'^\[\s*([\d.]+) sn\] (\d\d:\d\d:\d\d(?:\.\d+)?) (.*)$')
DOSYA_TARIHI = re.compile(r'ucus_.*?_(\d{8})_\d{6}')
AM_ALARM = re.compile(r'^(\S+Z) .*msg="Received alert".*alert=(\w+)\[[0-9a-f]+\]\[(active|resolved)\]')
AM_TELEGRAM = re.compile(r'^(\S+Z) .*Telegram message successfully published.*message_id=(\d+)')
EXP_BASLADI = re.compile(r'^(\d\d:\d\d:\d\d) ANOMAL\S* ALARMI BA\S*LADI')
SAAT_ESIGI = 8.0   # ariza suresi duvar saati ile ic sayacta bu kadar sn'den fazla ayrisirsa (uyku/duraksama) gecersiz


def docker_zamani(s):
    """2026-10-17T10:38:05.034952187Z -> epoch saniye (UTC)."""
    m = re.match(r'(\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d)(\.\d+)?Z', s)
    taban = datetime.strptime(m.group(1), '%Y-%m-%dT%H:%M:%S').replace(tzinfo=timezone.utc).timestamp()
    return taban + (float('0' + m.group(2)) if m.group(2) else 0.0)


def yerel_zaman(tarih, saat):
    """('20261017', '13:37:50.123') -> epoch saniye. Makinenin yerel saat dilimi kullanilir."""
    ana, _, kesir = saat.partition('.')
    d = datetime.strptime(tarih + ana, '%Y%m%d%H:%M:%S')
    return d.timestamp() + (float('0.' + kesir) if kesir else 0.0)


def ucus_oku(yol):
    tarih = None
    arizalar = []
    for satir in open(yol, encoding='utf-8', errors='replace'):
        satir = satir.rstrip('\r\n')
        if tarih is None and satir.startswith(('Kayit:', 'Olaylar:')):
            m = DOSYA_TARIHI.search(satir)
            if m:
                tarih = m.group(1)
        m = UCUS_SATIR.match(satir)
        if not m:
            continue
        if tarih is None:
            sys.exit(f"{yol}: 'Kayit:' satiri bulunamadi, tarih bilinmiyor. "
                     "Dosyayi otomatik_ucus.py'nin ilk satirlariyla birlikte kaydet.")
        t = yerel_zaman(tarih, m.group(2))
        mono = float(m.group(1))
        metin = m.group(3)
        if metin.startswith('ARIZA BASLIYOR: '):
            arizalar.append({'ad': metin.split(': ', 1)[1], 'bas': t, 'bas_t': mono, 'bit': None, 'bit_t': None})
        elif metin.startswith('ARIZA BITTI: ') and arizalar:
            arizalar[-1]['bit'] = t
            arizalar[-1]['bit_t'] = mono
    return tarih, arizalar


def am_oku(yol, alarm_adi):
    olaylar, mesajlar = [], []
    for satir in open(yol, encoding='utf-8', errors='replace'):
        m = AM_ALARM.match(satir)
        if m and m.group(2) == alarm_adi:
            olaylar.append((docker_zamani(m.group(1)), m.group(3)))
            continue
        m = AM_TELEGRAM.match(satir)
        if m:
            mesajlar.append((docker_zamani(m.group(1)), int(m.group(2))))
    olaylar.sort()
    mesajlar.sort()
    return olaylar, mesajlar


def exporter_oku(yol, tarih):
    zamanlar = []
    for satir in open(yol, encoding='utf-8', errors='replace'):
        m = EXP_BASLADI.match(satir)
        if m:
            zamanlar.append(yerel_zaman(tarih, m.group(1)))
    return zamanlar


def olc(arizalar, am_olaylar, mesajlar, exporter_zamanlari):
    # alarmin 'active' oldugu gecisler (resolved/yok -> active); tekrar gonderimler sayilmaz
    gecisler, onceki = [], 'resolved'
    for t, d in am_olaylar:
        if d == 'active' and onceki != 'active':
            gecisler.append(t)
        onceki = d

    def durum(t):
        son = 'resolved'
        for zaman, d in am_olaylar:
            if zaman >= t:
                break
            son = d
        return son

    # Telegram mesajlari: gonderim aninda alarm aktifse ALARM, degilse COZULDU mesajidir
    alarm_msj = {}   # gecis zamani -> ilk ALARM mesaji zamani
    cozuldu_msj = []
    for p, _id in mesajlar:
        if durum(p) == 'active':
            onceki_gecis = [g for g in gecisler if g <= p]
            if onceki_gecis and onceki_gecis[-1] not in alarm_msj:
                alarm_msj[onceki_gecis[-1]] = p
        else:
            cozuldu_msj.append(p)

    sonuc, atanan = [], set()
    for i, a in enumerate(arizalar):
        bas = a['bas']
        bit = a['bit'] if a['bit'] is not None else bas + 30
        sonraki = arizalar[i + 1]['bas'] if i + 1 < len(arizalar) else None
        r = {'ad': a['ad'], 'bas': bas, 'exp': None, 'am': None, 'prom': None,
             'tg': None, 'tg_durum': '-', 'coz': None, 'not': '', 'gecersiz': False}
        if a['bit'] is not None and a['bit_t'] is not None:
            sapma = (a['bit'] - bas) - (a['bit_t'] - a['bas_t'])
            if abs(sapma) > SAAT_ESIGI:
                r['gecersiz'] = True
                r['not'] = (f"SAAT SICRAMASI: ariza {a['bit'] - bas:.0f} sn surdu (duvar saati), "
                            f"{a['bit_t'] - a['bas_t']:.0f} sn (ic sayac); ozete katilmadi. ")
        if durum(bas) == 'active':
            r['not'] += 'onceki alarm suruyordu, olculemedi'
            sonuc.append(r)
            continue
        g = next((x for x in gecisler if bas <= x < bit + 20), None)
        e = next((x for x in exporter_zamanlari if bas - 1 <= x < bit + 20), None)
        if e is not None:
            r['exp'] = e + 0.5 - bas
        if g is None:
            r['not'] += 'ALARM YOK (Alertmanager almadi)'
        else:
            atanan.add(g)
            r['am'] = g - bas
            if e is not None:
                r['prom'] = g - (e + 0.5)
            if g in alarm_msj:
                r['tg'] = alarm_msj[g] - bas
                r['tg_durum'] = 'var'
            else:
                r['tg_durum'] = 'yok'
        c = next((p for p in cozuldu_msj if p >= bit), None)
        if c is not None and (sonraki is None or c < sonraki):
            r['coz'] = c - bit
        sonuc.append(r)
    plansiz = [g for g in gecisler if g not in atanan]
    return sonuc, plansiz


def f(x, w=6, d=1):
    return f'{x:{w}.{d}f}' if x is not None else ' ' * (w - 1) + '-'


def ozet(ad, degerler):
    if not degerler:
        return f'{ad}: olcum yok'
    return (f'{ad}: n={len(degerler)}  en az {min(degerler):.1f}  medyan {statistics.median(degerler):.1f}'
            f'  ort {statistics.mean(degerler):.1f}  en cok {max(degerler):.1f} sn')


def tur(ad):
    if ad.startswith('batt_volt'):
        return 'batarya voltaj'
    if ad.startswith('batt_res'):
        return 'batarya direnc'
    return 'motor'


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ucus', required=True)
    ap.add_argument('--am', required=True)
    ap.add_argument('--exporter')
    ap.add_argument('--alarm', default='DonanimAnomalisi', help='Alertmanager alarm adi')
    ap.add_argument('--csv', help='sonuclari bu CSV dosyasina da yaz')
    a = ap.parse_args()

    tarih, arizalar = ucus_oku(a.ucus)
    if not arizalar:
        sys.exit('Ucus dosyasinda ARIZA BASLIYOR satiri yok.')
    am_olaylar, mesajlar = am_oku(a.am, a.alarm)
    exp = exporter_oku(a.exporter, tarih) if a.exporter else []
    sonuc, plansiz = olc(arizalar, am_olaylar, mesajlar, exp)

    print(f'Ucus tarihi {tarih} | {len(arizalar)} ariza | {len(am_olaylar)} AM alarm olayi | '
          f'{len(mesajlar)} Telegram mesaji')
    print()
    print(f"{'#':>2} {'ariza':<16} {'basladi':<12} {'exp~':>6} {'AM':>6} {'prom~':>6} {'tg':>6} {'coz':>6}  not")
    for i, r in enumerate(sonuc, 1):
        saat = datetime.fromtimestamp(r['bas']).strftime('%H:%M:%S.%f')[:-3]
        tg = f(r['tg']) if r['tg'] is not None else ('   yok' if r['tg_durum'] == 'yok' else '     -')
        print(f"{i:>2} {r['ad']:<16} {saat:<12} {f(r['exp'])} {f(r['am'])} {f(r['prom'])} {tg} {f(r['coz'])}  {r['not']}")
    print()
    tum_sonuc = sonuc
    gecersiz = [r['ad'] for r in sonuc if r['gecersiz']]
    sonuc = [r for r in sonuc if not r['gecersiz']]
    if gecersiz:
        print('Saat sicramasi nedeniyle ozete KATILMAYAN ariza: ' + ', '.join(gecersiz))
    am = [r['am'] for r in sonuc if r['am'] is not None]
    kacan = [r['ad'] for r in sonuc if r['am'] is None]
    print(f'Alarm uretilen ariza: {len(am)}/{len(sonuc)} (gecerli arizalar icinde)')
    if kacan:
        print('Alarm gelmeyen/olculemeyen: ' + ', '.join(kacan))
    print(ozet('ariza -> Alertmanager (AM)', am))
    for t in ('batarya voltaj', 'batarya direnc', 'motor'):
        print('  ' + ozet(t, [r['am'] for r in sonuc if r['am'] is not None and tur(r['ad']) == t]))
    print(ozet('exporter alarmi (exp~, +-0.5 sn)', [r['exp'] for r in sonuc if r['exp'] is not None]))
    print(ozet('Prometheus asamasi (prom~, +-0.5 sn)', [r['prom'] for r in sonuc if r['prom'] is not None]))
    tg = [r['tg'] for r in sonuc if r['tg'] is not None]
    print(ozet('ariza -> Telegram ALARM mesaji (tg)', tg))
    print(f"Telegram'a mesaj giden ariza: {len(tg)}/{len(am)} | birlestirilen (mesaj yok): "
          f"{sum(1 for r in sonuc if r['tg_durum'] == 'yok')}")
    print(ozet('ariza bitisi -> COZULDU mesaji (coz)', [r['coz'] for r in sonuc if r['coz'] is not None]))
    if plansiz:
        print('PLANSIZ alarm (hicbir arizaya bagli degil): ' + ', '.join(
            datetime.fromtimestamp(g).strftime('%H:%M:%S') for g in plansiz))
    else:
        print('Plansiz alarm: yok')

    if a.csv:
        with open(a.csv, 'w', newline='', encoding='utf-8') as fh:
            w = csv.writer(fh)
            w.writerow(['ariza', 'basladi_epoch', 'exporter_sn', 'alertmanager_sn', 'prometheus_sn',
                        'telegram_alarm_sn', 'telegram_durum', 'cozuldu_sn', 'gecersiz', 'not'])
            for r in tum_sonuc:
                w.writerow([r['ad'], f"{r['bas']:.3f}", r['exp'], r['am'], r['prom'], r['tg'],
                            r['tg_durum'], r['coz'], r['gecersiz'], r['not']])
        print(f'CSV: {a.csv}')


if __name__ == '__main__':
    main()