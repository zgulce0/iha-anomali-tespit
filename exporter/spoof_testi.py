# ===========================================================================
# BU DOSYA NE YAPAR?
# GPS spoofing dedektörünün TOPLU TESTİ, tek komutla. SITL'de drone'u uçurur (hover ve daire),
# her turda saldırı yönünü değiştirir (kuzey, doğu, kuzeydoğu) ve şu saldırıları sırayla uygular:
#   ani sıçrama 1/2/5/10/25 m, yavaş kayma 0,1/0,2/0,5/1 m/s, tutarlı kayma (konum + hız) 0,2/0,5/1 m/s,
#   yükseklik sıçraması 2/5/10/20 m, yavaş yükseklik kayması 0,2/0,5 m/s.
# Her turun başında saldırısız normal uçuş vardır (yanlış alarm ölçmek için).
# Süre: her desende tur başına ~23 dk; 'hepsi' ve 5 tur ~3,9 saat. Hover ve daire ayrı zamanlarda da
# çalıştırılabilir; analiz iki klasörü birlikte okur.
#
# Tespit burada YAPILMAZ. Araçtan gelen ham mesajlar (IMU, tutum, GPS, barometre, gerçek konum)
# olduğu gibi kaydedilir; analiz/spoof_roc.py bu kaydı canlı sistemdeki kodun aynısıyla yeniden
# oynatır. Böylece aynı uçuşta farklı eşikler ve gerçekçi GPS hatası denenebilir (ROC eğrisi).
#
# Çıktı: veri/spoof_testi_<tarih>/ klasörü
#   ham.csv.gz      araçtan gelen mesajlar (sıkıştırılmış)
#   etiketler.csv   her saldırının ve normal uçuşun başlangıç/bitiş anı
#   olaylar.csv     komutlar, ArduPilot'un kendi uyarıları (ör. "GPS Glitch")
#   bilgi.txt       ayarlar
# ===========================================================================
"""GPS spoofing toplu testi (SITL).

Kullanim (proje kokunden; SITL scripts/baslat.sh ile acik; otomatik_ucus.py, demo_ariza.py ve
spoof_deney.py KAPALI. Exporter acik kalabilir ama her saldiri Telegram'a alarm gonderir):

    venv/bin/python exporter/spoof_testi.py [hover|daire|hepsi] [--tur N] [--deneme]

    hover | daire | hepsi   ucus deseni (varsayilan: hepsi = once hover, sonra daire)
    --tur N                 her desende kac tur (varsayilan 5). Her tur tum saldirilari bir kez yapar.
    --deneme                kisa deneme: her desende 1 tur, 4 saldiri (her sey calisiyor mu?)

Saldirilar SITL parametreleriyle verilir:
    SIM_GPS1_GLTCH_X/Y  konum kaymasi (DERECE: enlem/boylama eklenir)
    SIM_GPS1_GLTCH_Z    yukseklik kaymasi (metre, yukari +)
    SIM_GPS1_GLTV_X/Y   tutarli kayma: GPS hizina eklenir ve konum bu hizla kayar (m/s)
"""
import gzip
import math
import os
import signal
import sys
import time
from datetime import datetime

from pymavlink import mavutil

# ---------------------------------------------------------------------------
# AYARLAR
# ---------------------------------------------------------------------------
BAGLANTI = 'udpin:127.0.0.1:14553'
KLASOR = os.path.expanduser('~/iha-anomali-tespit/veri')

IRTIFA = 10            # hedef irtifa (m)
KARARLI_SN = 10        # kalkis (ya da daireye gecis) sonrasi bekleme
NORMAL_SN = 120        # her turun basinda saldirisiz ucus (yanlis alarm olcumu icin)
TOPARLANMA_SN = 35     # her saldiri geri alindiktan sonra bekleme (canli alarm kilidi 30 sn; saldirilar ayri alarm olsun)
YAVAS_ADIM_SN = 0.5    # yavas kaymada parametrenin guncellenme araligi
DUSME_ESIGI = 5.0      # havadayken irtifa bunun altina inerse dur
M_DERECE = 1.0 / 111320.0

YONLER = {'kuzey': (1.0, 0.0), 'dogu': (0.0, 1.0), 'kuzeydogu': (math.sqrt(0.5), math.sqrt(0.5))}
YON_SIRASI = ['kuzey', 'dogu', 'kuzeydogu']

# (tip, deger, sure_sn). tip: ani (m), yavas (m/s), tutarli (m/s), irtifa_ani (m), irtifa_yavas (m/s)
# Kucuk saldirilar (1 m, 0,1 m/s, 0,2 m/s, 2 m) bilerek var: tespit sinirina yakin olanlar olmadan ROC egrisi
# anlamsiz olur (her esikte her sey yakalanir).
PLAN = [
    ('ani', 1.0, 10), ('ani', 2.0, 10), ('ani', 5.0, 10), ('ani', 10.0, 10), ('ani', 25.0, 10),
    ('yavas', 0.1, 120), ('yavas', 0.2, 75), ('yavas', 0.5, 40), ('yavas', 1.0, 30),
    ('tutarli', 0.2, 75), ('tutarli', 0.5, 40), ('tutarli', 1.0, 30),
    ('irtifa_ani', 2.0, 10), ('irtifa_ani', 5.0, 10), ('irtifa_ani', 10.0, 10), ('irtifa_ani', 20.0, 10),
    ('irtifa_yavas', 0.2, 75), ('irtifa_yavas', 0.5, 40),
]
DENEME_PLAN = [('ani', 10.0, 10), ('yavas', 1.0, 15), ('tutarli', 1.0, 10), ('irtifa_ani', 10.0, 10)]
DENEME_NORMAL_SN, DENEME_TOPARLANMA_SN = 30, 35

# Araçtan istenen mesajlar (numara -> Hz)
MESAJ_HIZLARI = {27: 50, 30: 50, 24: 10, 29: 10, 33: 10, 164: 10}
#               RAW_IMU ATTITUDE GPS_RAW_INT SCALED_PRESSURE GLOBAL_POSITION_INT SIMSTATE
EN_AZ_HIZ = {'RAW_IMU': 35, 'GPS_RAW_INT': 7, 'SCALED_PRESSURE': 5}   # bunlarin altindaysa test anlamsiz

TEMIZLIK = [('SIM_GPS1_GLTCH_X', 0.0), ('SIM_GPS1_GLTCH_Y', 0.0), ('SIM_GPS1_GLTCH_Z', 0.0),
            ('SIM_GPS1_HNSE', 0.0), ('SIM_GPS1_VERR_X', 0.0), ('SIM_GPS1_VERR_Y', 0.0)]
GLTV = [('SIM_GPS1_GLTV_X', 0.0), ('SIM_GPS1_GLTV_Y', 0.0), ('SIM_GPS1_GLTV_Z', 0.0)]

MODLAR = {'GUIDED': 4, 'LOITER': 5, 'CIRCLE': 7, 'LAND': 9}

# Ham kayit: her satir "w,kod,alanlar..."  (w: testin basindan beri saniye, bilgisayar saati)
HAM_ALANLAR = {
    'RAW_IMU': ('I', ('time_usec', 'xacc', 'yacc', 'zacc')),
    'ATTITUDE': ('A', ('time_boot_ms', 'roll', 'pitch', 'yaw')),
    'GPS_RAW_INT': ('G', ('time_usec', 'lat', 'lon', 'alt', 'vel', 'cog', 'fix_type', 'satellites_visible', 'eph')),
    'SCALED_PRESSURE': ('B', ('time_boot_ms', 'press_abs', 'temperature')),
    'GLOBAL_POSITION_INT': ('P', ('time_boot_ms', 'lat', 'lon', 'alt', 'relative_alt')),
    'SIMSTATE': ('S', ('lat', 'lng')),
}


class Hata(Exception):
    pass


class Dusme(Exception):
    pass


# ---------------------------------------------------------------------------
# HAFIZA
# ---------------------------------------------------------------------------
son = {'armed': None, 'irtifa': None, 'enlem': None, 'arac_saat': None}
param_degerleri = {}
durum = {'havada': False, 'beklenen_mod': None, 'rc': False, 'sonraki_rc': 0.0, 'son_paket': 0.0,
         'son_yazdirma': 0.0, 'etiket': 'hazirlik', 'sayac': {}, 'satir': 0, 'son_flush': 0.0,
         'son_istek': 0.0, 'hiz_bas': 0.0, 'hiz_sayac': {}, 'seyrek_pencere': 0}
baglanti = None
ham = None
etiket_dosya = None
olay_dosya = None
T0 = 0.0


def w():
    return time.monotonic() - T0


def olay(metin):
    t = w()
    print(f'[{t:7.1f} sn] {datetime.now():%H:%M:%S} {metin}', flush=True)
    if olay_dosya is not None:
        olay_dosya.write(f'{t:.3f},{son["arac_saat"] or ""},{metin.replace(",", ";")}\n')
        olay_dosya.flush()


def etiket_yaz(desen, tur, yon, tip, deger, bas, bit, sonuc='tamam'):
    """bas/bit: (w, arac_saat) ikilileri."""
    etiket_dosya.write(f'{desen},{tur},{yon},{tip},{deger},{bas[0]:.3f},{bas[1] or ""},'
                       f'{bit[0]:.3f},{bit[1] or ""},{sonuc}\n')
    etiket_dosya.flush()


def an():
    return (w(), son['arac_saat'])


# ---------------------------------------------------------------------------
# MESAJ OKUMA VE KAYIT
# ---------------------------------------------------------------------------
def guncelle(m):
    tip = m.get_type()
    durum['sayac'][tip] = durum['sayac'].get(tip, 0) + 1
    durum['hiz_sayac'][tip] = durum['hiz_sayac'].get(tip, 0) + 1
    if tip in HAM_ALANLAR:
        kod, alanlar = HAM_ALANLAR[tip]
        ham.write(f'{w():.3f},{kod},' + ','.join(str(getattr(m, a)) for a in alanlar) + '\n')
        durum['satir'] += 1
    if tip == 'HEARTBEAT':
        if m.type != mavutil.mavlink.MAV_TYPE_GCS:
            son['armed'] = 1 if (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) else 0
            baglanti.target_system = m.get_srcSystem()
            baglanti.target_component = m.get_srcComponent()
            ham.write(f'{w():.3f},H,{m.base_mode},{m.custom_mode}\n')
    elif tip == 'RAW_IMU':
        son['arac_saat'] = round(m.time_usec / 1e6, 4)
    elif tip == 'GLOBAL_POSITION_INT':
        son['irtifa'] = m.relative_alt / 1000.0
    elif tip == 'GPS_RAW_INT':
        if m.fix_type >= 3:
            son['enlem'] = m.lat / 1e7
    elif tip == 'STATUSTEXT':          # ArduPilot'un kendi uyarilari (ornegin "GPS Glitch")
        metin = m.text if isinstance(m.text, str) else m.text.decode('utf-8', 'ignore')
        olay('ARDUPILOT: ' + metin.strip('\x00').strip())
    elif tip == 'PARAM_VALUE':
        pid = m.param_id if isinstance(m.param_id, str) else m.param_id.decode('utf-8', 'ignore')
        param_degerleri[pid.rstrip('\x00')] = m.param_value


def rc_gonder():
    """CIRCLE/LOITER'da irtifayi gaz cubugu belirler; SITL'de kumanda yok. Gazi ortada tut."""
    baglanti.mav.rc_channels_override_send(baglanti.target_system, baglanti.target_component,
                                           65535, 65535, 1500, 65535, 65535, 65535, 65535, 65535)


def adim():
    m = baglanti.recv_match(blocking=True, timeout=0.05)
    simdi = time.monotonic()
    if m is not None:
        durum['son_paket'] = simdi
        guncelle(m)
    if durum['rc'] and simdi >= durum['sonraki_rc']:
        rc_gonder()
        durum['sonraki_rc'] = simdi + 0.5
    if simdi - durum['son_flush'] > 10:
        ham.flush()
        durum['son_flush'] = simdi
    if simdi - durum['son_paket'] > 10:
        raise Hata('10 sn boyunca aractan mesaj gelmedi (SITL kapandi mi?)')
    if simdi - durum['son_istek'] > 30:      # SITL ya da MAVProxy hizlari degistirmis olabilir: ara ara yeniden iste
        durum['son_istek'] = simdi
        mesaj_hizlarini_iste(bekleme=False)
    if simdi - durum['hiz_bas'] >= 10:       # son 10 sn'de mesajlar yeterince sik geldi mi?
        sure = simdi - durum['hiz_bas']
        seyrek = [k for k, v in EN_AZ_HIZ.items() if durum['hiz_sayac'].get(k, 0) / sure < v]
        durum['hiz_bas'], durum['hiz_sayac'] = simdi, {}
        durum['seyrek_pencere'] = durum['seyrek_pencere'] + 1 if seyrek else 0
        if seyrek:
            olay('UYARI: mesajlar seyrek geliyor: ' + ', '.join(seyrek))
        if durum['seyrek_pencere'] >= 3:
            raise Hata("mesajlar 30 sn'dir seyrek geliyor (" + ', '.join(seyrek) + "). MAVProxy penceresine "
                       "'set streamrate -1' yazip testi yeniden baslat.")
    if durum['havada']:
        if son['irtifa'] is not None and son['irtifa'] < DUSME_ESIGI:
            raise Dusme(f"irtifa {son['irtifa']:.1f} m (esik {DUSME_ESIGI} m)")
        if durum['beklenen_mod'] and baglanti.flightmode not in (durum['beklenen_mod'], None):
            raise Hata(f"mod kendiliginden {baglanti.flightmode} oldu (ArduPilot failsafe olabilir)")
    if simdi - durum['son_yazdirma'] >= 10:
        durum['son_yazdirma'] = simdi
        irt = '-' if son['irtifa'] is None else f"{son['irtifa']:.1f}"
        print(f"{w():7.1f} sn | {durum['etiket']:<28} | irtifa {irt} m | mod {baglanti.flightmode}", flush=True)


def bekle_kosul(kosul, sn):
    bitis = time.monotonic() + sn
    while time.monotonic() < bitis:
        adim()
        if kosul():
            return True
    return kosul()


def bekle(sn):
    bekle_kosul(lambda: False, sn)


# ---------------------------------------------------------------------------
# KOMUTLAR
# ---------------------------------------------------------------------------
def komut(cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
    baglanti.mav.command_long_send(baglanti.target_system, baglanti.target_component, cmd, 0,
                                   p1, p2, p3, p4, p5, p6, p7)


def mesaj_hizlarini_iste(bekleme=True):
    for msg_id, hz in MESAJ_HIZLARI.items():
        komut(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, int(1e6 / hz))
        if bekleme:
            bekle(0.05)


def hizlari_kontrol_et():
    """Mesajlar istenen hizda geliyor mu? MAVProxy 'set streamrate -1' yazilmadiysa hizlari ezer."""
    mesaj_hizlarini_iste()
    bekle(5)
    durum['sayac'] = {}
    bekle(10)
    hz = {k: durum['sayac'].get(k, 0) / 10 for k in EN_AZ_HIZ}
    olay('mesaj hizlari (sn basina): ' + ', '.join(f'{k} {v:.0f}' for k, v in hz.items()))
    eksik = [k for k, v in hz.items() if v < EN_AZ_HIZ[k]]
    if eksik:
        raise Hata(f"{', '.join(eksik)} yeterince sik gelmiyor. MAVProxy penceresine 'set streamrate -1' "
                   "yazip testi yeniden baslat.")


def mod_ayarla(ad):
    for _ in range(3):
        komut(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, MODLAR[ad])
        if bekle_kosul(lambda: baglanti.flightmode == ad, 4):
            olay(f'mod {ad}')
            return
    raise Hata(f'{ad} moduna gecilemedi (mod su an {baglanti.flightmode})')


def param_gonder(ad, deger):
    param_degerleri.pop(ad, None)
    baglanti.param_set_send(ad, float(deger))


def param_onayli(ayarlar, yazdir=True):
    """[(ad, deger), ...] hepsini gonderir ve SITL'in hepsini onaylamasini bekler."""
    for _ in range(3):
        for ad, v in ayarlar:
            param_gonder(ad, v)

        def tamam():
            return all(ad in param_degerleri and abs(param_degerleri[ad] - v) <= 1e-9 + 1e-6 * abs(v)
                       for ad, v in ayarlar)
        if bekle_kosul(tamam, 3):
            if yazdir:
                olay('param ' + ' '.join(f'{ad}={v:.9g}' for ad, v in ayarlar))
            return
    raise Hata(' '.join(ad for ad, _ in ayarlar) + ' SITL tarafindan onaylanmadi')


def arm_et():
    for _ in range(24):
        komut(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        if bekle_kosul(lambda: son['armed'] == 1, 5):
            olay('ARM tamam')
            return
        olay('ARM olmadi (GPS / on kontroller bekleniyor olabilir), tekrar deneniyor')
    raise Hata('arm edilemedi')


def kalkis():
    komut(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, IRTIFA)
    olay(f'kalkis komutu, hedef {IRTIFA} m')
    if not bekle_kosul(lambda: son['irtifa'] is not None and son['irtifa'] >= IRTIFA - 1, 60):
        raise Hata(f"{IRTIFA - 1} m'ye cikilamadi (irtifa {son['irtifa']})")
    olay(f"hedef irtifaya ulasildi ({son['irtifa']:.1f} m)")


def kayma_param(tip, kuzey_m, dogu_m, yukari_m=0.0):
    """Metre cinsinden kaymayi SITL parametrelerine cevirir."""
    cos_lat = math.cos(math.radians(son['enlem'] or 0.0))
    if tip in ('ani', 'yavas'):
        return [('SIM_GPS1_GLTCH_X', kuzey_m * M_DERECE), ('SIM_GPS1_GLTCH_Y', dogu_m * M_DERECE / cos_lat)]
    if tip == 'tutarli':
        return [('SIM_GPS1_GLTV_X', kuzey_m), ('SIM_GPS1_GLTV_Y', dogu_m)]
    return [('SIM_GPS1_GLTCH_Z', yukari_m)]


# ---------------------------------------------------------------------------
# SALDIRI
# ---------------------------------------------------------------------------
def saldiri(desen, tur, yon, tip, deger, sure, toparlanma):
    un, ue = YONLER[yon]
    yatay = not tip.startswith('irtifa')
    ad = f"{tip} {deger:g}{' m' if tip in ('ani', 'irtifa_ani') else ' m/s'}" + (f' {yon}' if yatay else ' yukari')
    olay(f'SALDIRI BASLIYOR: {desen} tur {tur} | {ad} | {sure} sn')
    durum['etiket'] = f'{desen} t{tur} {ad}'

    def ayar(m):         # m metre (ya da m/s) -> parametreler
        if yatay:
            return kayma_param(tip, m * un, m * ue)
        return kayma_param(tip, 0, 0, m)

    bas = an()           # olcum, komutun GONDERILDIGI andan baslar (tespit suresi biraz fazla olculur, az degil)
    try:
        if tip in ('ani', 'irtifa_ani', 'tutarli'):
            param_onayli(ayar(deger))
            bekle(sure - (w() - bas[0]))
        else:                                    # yavas kayma: parametre her YAVAS_ADIM_SN'de buyur
            t0 = time.monotonic()
            while time.monotonic() - t0 < sure:
                for p, v in ayar(deger * (time.monotonic() - t0)):
                    param_gonder(p, v)
                bekle(YAVAS_ADIM_SN)
            param_onayli(ayar(deger * sure), yazdir=False)
        bit = an()
        param_onayli(ayar(0.0))                  # geri al (bu da ters yonde bir sicramadir)
    except BaseException:
        etiket_yaz(desen, tur, yon, tip, deger, bas, an(), 'yarim')
        raise
    etiket_yaz(desen, tur, yon, tip, deger, bas, bit)
    olay(f'SALDIRI BITTI: {ad}')
    durum['etiket'] = f'{desen} t{tur} toparlanma'
    tb = an()
    bekle(toparlanma)
    etiket_yaz(desen, tur, yon, 'toparlanma', 0, tb, an())


def desen_uc(desen, turlar, plan, normal_sn, toparlanma):
    durum['etiket'] = f'{desen} kalkis'
    mod_ayarla('GUIDED')
    arm_et()
    durum['beklenen_mod'] = 'GUIDED'
    kalkis()
    durum['havada'] = True
    if desen == 'daire':
        durum['rc'] = True
        durum['beklenen_mod'] = None
        mod_ayarla('LOITER')
        bekle(5)
        mod_ayarla('CIRCLE')
        durum['beklenen_mod'] = 'CIRCLE'
    olay(f'kararlasma bekleniyor ({KARARLI_SN} sn)')
    bekle(KARARLI_SN)
    for tur in range(1, turlar + 1):
        yon = YON_SIRASI[(tur - 1) % len(YON_SIRASI)]
        olay(f'=== {desen} TUR {tur}/{turlar} (yatay saldiri yonu: {yon}) ===')
        durum['etiket'] = f'{desen} t{tur} normal'
        nb = an()
        bekle(normal_sn)
        etiket_yaz(desen, tur, yon, 'normal', 0, nb, an())
        for tip, deger, sure in plan:
            saldiri(desen, tur, yon, tip, deger, sure, toparlanma)
    inis()


def inis():
    durum['etiket'] = 'inis'
    if durum['rc'] and baglanti.flightmode == 'CIRCLE':
        durum['beklenen_mod'] = None
        mod_ayarla('LOITER')
        bekle(5)
    durum['havada'] = False
    durum['beklenen_mod'] = None
    mod_ayarla('LAND')
    if not bekle_kosul(lambda: son['armed'] == 0, 120):
        raise Hata('inis sonrasi disarm olmadi')
    durum['rc'] = False
    olay('inis tamam, disarm')


def temizle():
    """Saldiri parametrelerini sifirla; havadaysa indir."""
    durum['havada'] = False
    durum['beklenen_mod'] = None
    try:
        for ayarlar in (TEMIZLIK, GLTV):
            try:
                param_onayli(ayarlar)
            except Hata as e:
                olay(f'UYARI: temizlik basarisiz: {e}')
        if son['armed'] == 1:
            durum['etiket'] = 'sonlandirma'
            komut(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, MODLAR['LAND'])
            olay('LAND komutu gonderildi')
            bekle_kosul(lambda: son['armed'] == 0, 120)
        durum['rc'] = False
    except KeyboardInterrupt:
        olay('ikinci Ctrl+C: temizlik yarida kesildi')


def main(argv=None):
    global baglanti, ham, etiket_dosya, olay_dosya, T0
    args = (sys.argv if argv is None else argv)[1:]
    # Terminal kapanirsa ya da surec durdurulursa da Ctrl+C gibi davran: saldiriyi geri al, araci indir.
    for sinyal in (signal.SIGTERM, getattr(signal, 'SIGHUP', None)):
        if sinyal is not None:
            signal.signal(sinyal, signal.default_int_handler)
    if '-h' in args or '--help' in args:
        print(__doc__)
        return 0
    deneme = '--deneme' in args
    turlar = 5
    if '--tur' in args:
        i = args.index('--tur')
        try:
            turlar = int(args[i + 1])
        except (IndexError, ValueError):
            print('--tur sonrasina bir sayi yaz (ornek: --tur 3)')
            return 2
    secim = next((a for a in args if a in ('hover', 'daire', 'hepsi')), 'hepsi')
    desenler = ['hover', 'daire'] if secim == 'hepsi' else [secim]
    plan, normal_sn, toparlanma = PLAN, NORMAL_SN, TOPARLANMA_SN
    if deneme:
        plan, normal_sn, toparlanma, turlar = DENEME_PLAN, DENEME_NORMAL_SN, DENEME_TOPARLANMA_SN, 1

    tur_sn = normal_sn + sum(s[2] + toparlanma for s in plan)
    toplam_dk = len(desenler) * (turlar * tur_sn + 90) / 60

    baglanti = mavutil.mavlink_connection(BAGLANTI)
    print(f'{BAGLANTI} dinleniyor, heartbeat bekleniyor...')
    if baglanti.wait_heartbeat(timeout=15) is None:
        print("Heartbeat gelmedi. SITL acik mi? MAVProxy'de: output add 127.0.0.1:14553")
        return 1

    klasor = os.path.join(KLASOR, f"spoof_testi_{'deneme_' if deneme else ''}{datetime.now():%Y%m%d_%H%M%S}")
    os.makedirs(klasor, exist_ok=True)
    ham = gzip.open(os.path.join(klasor, 'ham.csv.gz'), 'wt', compresslevel=6)
    etiket_dosya = open(os.path.join(klasor, 'etiketler.csv'), 'w')
    etiket_dosya.write('desen,tur,yon,tip,deger,bas_w,bas_arac,bit_w,bit_arac,sonuc\n')
    olay_dosya = open(os.path.join(klasor, 'olaylar.csv'), 'w')
    olay_dosya.write('w,arac_saat,olay\n')
    with open(os.path.join(klasor, 'bilgi.txt'), 'w') as f:
        f.write(f'baslangic: {datetime.now():%Y-%m-%d %H:%M:%S}\ndesenler: {desenler}\ntur: {turlar}\n'
                f'irtifa: {IRTIFA} m\nnormal_sn: {normal_sn}\ntoparlanma_sn: {toparlanma}\n'
                f'yon sirasi: {YON_SIRASI}\nplan (tip, deger, sure_sn): {plan}\n'
                f'mesaj hizlari: {MESAJ_HIZLARI}\n')
    T0 = time.monotonic()
    durum['son_paket'] = durum['son_yazdirma'] = durum['son_flush'] = time.monotonic()
    durum['son_istek'] = durum['hiz_bas'] = time.monotonic()
    print(f'Kayit klasoru: {klasor}')
    print(f"Desen: {', '.join(desenler)} | her desende {turlar} tur x {len(plan)} saldiri | "
          f'tahmini sure ~{toplam_dk:.0f} dk')

    cikis = 0
    try:
        hizlari_kontrol_et()
        try:
            param_onayli(GLTV)
        except Hata:
            raise Hata("SIM_GPS1_GLTV_X parametresi yok: ArduPilot surumu eski. Tutarli kayma icin ArduPilot'u "
                       "guncelleyip yeniden derle.")
        param_onayli(TEMIZLIK)
        param_onayli([('DISARM_DELAY', 60)])
        for desen in desenler:
            try:
                desen_uc(desen, turlar, plan, normal_sn, toparlanma)
            except (Hata, Dusme) as e:
                olay(f'HATA ({desen}): {e}')
                cikis = 1
                temizle()
                if desenler.index(desen) < len(desenler) - 1:
                    olay('sonraki desene geciliyor')
                    bekle(5)
    except KeyboardInterrupt:
        olay('kullanici durdurdu (Ctrl+C)')
        cikis = 1
    except Hata as e:
        olay(f'HATA: {e}')
        cikis = 1
    finally:
        temizle()
        ham.close()
        etiket_dosya.close()
        olay_dosya.close()
    print(f"Kaydedilen ham satir: {durum['satir']}")
    print(f'Klasor: {klasor}')
    print('Analiz icin: venv/bin/python analiz/spoof_roc.py ' + os.path.relpath(klasor, os.path.join(KLASOR, '..')))
    return cikis


if __name__ == '__main__':
    sys.exit(main())
