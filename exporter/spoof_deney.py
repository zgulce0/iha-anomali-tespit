# ===========================================================================
# BU DOSYA NE YAPAR?
# GPS spoofing deneyi: SITL'de drone'u kaldırır, 10 m'de hover yaptırır, önce normal
# uçurur, sonra GPS konumunu sahte olarak kaydırır (ani sıçrama ve yavaş kayma).
# Bütün süre boyunca saniyede 5 satır kaydeder. Her satırda hem ham GPS konumu hem
# ArduPilot'un kendi tahmini (EKF) hem de ivmeölçer var; tespit yöntemleri
# (GPS-EKF farkı, dead-reckoning, kendi EKF'miz) bu kayıtla geliştirilecek.
# Çıktı iki dosyadır (otomatik_ucus.py ile aynı düzen):
#   spoof_<ad>_<tarih>.csv          -> saniyede 5 satır veri
#   spoof_<ad>_<tarih>_olaylar.csv  -> "SPOOF BASLIYOR" gibi olayların saatleri
# ===========================================================================
"""GPS spoofing deneyi + kayit (SITL).

Kullanim (proje kokunden; SITL acik, otomatik_ucus.py ve demo_ariza.py KAPALI):
    venv/bin/python exporter/spoof_deney.py <ad> [ani|yavas|hepsi] [gurultu]

    ad       dosya adinda kullanilir (ornek: ilk_deneme)
    secim    ani   -> sadece ani sicramalar
             yavas -> sadece yavas kaymalar
             hepsi -> ikisi de (varsayilan)
    gurultu  yazilirsa SITL'in GPS gurultusu acilir (SIM_GPS1_HNSE ve SIM_GPS1_VERR_X/Y).
             Not: SITL'in yatay "gurultusu" rastgele degil, 12,6 sn'lik yavas bir dairesel
             salinimdir; GPS hizina ise rastgele gurultu ekler.

Spoofing, SITL'in SIM_GPS1_GLTCH_X parametresiyle verilir. Bu parametre METRE DEGIL
DERECE cinsindendir: GPS'in bildirdigi enleme dogrudan eklenir (0.0001 derece ~ 11 m kuzey).
Etiket sutunu, kaymanin gercekten uygulandigi araligi gosterir.
"""
import math
import os
import sys
import time
from datetime import datetime

from pymavlink import mavutil

# ---------------------------------------------------------------------------
# KISIM 1: AYARLAR
# ---------------------------------------------------------------------------
BAGLANTI = 'udpin:127.0.0.1:14553'   # otomatik_ucus.py ve demo_ariza.py ile ayni kapi (ayni anda calismamali)
KLASOR = os.path.expanduser('~/iha-anomali-tespit/veri')

IRTIFA = 10            # hedef irtifa (m)
KARARLI_SN = 10        # kalkistan sonra bekleme
BASLANGIC_SN = 40      # spoofing'den once normal ucus
TOPARLANMA_SN = 40     # her spoofing geri alindiktan sonra bekleme (etiket 'toparlanma')
PERIYOT = 0.2          # kayit periyodu: saniyede 5 satir (GPS de saniyede 5 olcum veriyor)
DUSME_ESIGI = 5.0      # havadayken irtifa bunun altina inerse ucus bozuldu sayilir (m)
M_DERECE = 1.0 / 111320.0   # 1 metre kac derece enlem eder (yaklasik)

# Spoofing senaryolari. Her biri: (etiket, tur, deger, sure_sn)
#   ani   : konum bir anda 'deger' metre kuzeye kayar ve sure boyunca orada kalir
#   yavas : konum saniyede 'deger' metre hizla kuzeye kayar (sure boyunca surekli artar)
SENARYOLAR = [
    ('spoof_ani_2m',      'ani',   2.0,  30),
    ('spoof_ani_5m',      'ani',   5.0,  30),
    ('spoof_ani_10m',     'ani',  10.0,  30),
    ('spoof_ani_25m',     'ani',  25.0,  30),
    ('spoof_yavas_0.05ms', 'yavas', 0.05, 120), # 120 sn'de 6 m
    ('spoof_yavas_0.1ms', 'yavas', 0.1,  90),   # 90 sn'de 9 m
    ('spoof_yavas_0.2ms', 'yavas', 0.2,  60),   # 60 sn'de 12 m
    ('spoof_yavas_0.5ms', 'yavas', 0.5,  40),   # 40 sn'de 20 m
    ('spoof_yavas_1ms',   'yavas', 1.0,  30),   # 30 sn'de 30 m
]
YAVAS_ADIM_SN = 0.5    # yavas kaymada parametrenin guncellenme araligi

TEMIZLIK = [('SIM_GPS1_GLTCH_X', 0.0), ('SIM_GPS1_GLTCH_Y', 0.0),
            ('SIM_GPS1_HNSE', 0.0), ('SIM_GPS1_VERR_X', 0.0), ('SIM_GPS1_VERR_Y', 0.0)]
# 'gurultu' secilirse uygulanacak ayarlar
GURULTU = [('SIM_GPS1_HNSE', 0.5),     # konum: 0,5 m yaricapli yavas dairesel salinim (12,6 sn'de bir tur)
           ('SIM_GPS1_VERR_X', 0.1),   # GPS hizina +-0,1 m/s rastgele hata (kuzey)
           ('SIM_GPS1_VERR_Y', 0.1)]   # (dogu)

MODLAR = {'GUIDED': 4, 'LAND': 9}

# Hangi MAVLink mesajlarini kac Hz isteyecegiz (mesaj numarasi -> Hz)
MESAJ_HIZLARI = {
    24: 10,    # GPS_RAW_INT          ham GPS konumu, hizi, uydu, fix
    33: 10,    # GLOBAL_POSITION_INT  ArduPilot'un EKF tahmini (konum ve hiz)
    32: 10,    # LOCAL_POSITION_NED   EKF tahmini, kalkis noktasina gore metre (kuzey, dogu, asagi)
    27: 10,    # RAW_IMU              ivmeolcer (milli-g)
    30: 10,    # ATTITUDE             yatis, egim, yon
    193: 5,    # EKF_STATUS_REPORT    EKF'nin kendi guven degerleri
    164: 10,   # SIMSTATE             SADECE SIMULATORDE: aracin GERCEK konumu (spoofing'den etkilenmez)
}

BASLIK = ('zaman_sn,etiket,glitch_m,armed,mod,irtifa_m,'
          'gps_enlem,gps_boylam,gps_hiz_ms,gps_rota_der,gps_eph,uydu,fix,'
          'ekf_enlem,ekf_boylam,ekf_vx,ekf_vy,ekf_vz,'
          'lokal_x,lokal_y,lokal_z,lokal_vx,lokal_vy,lokal_vz,'
          'ivme_x_mg,ivme_y_mg,ivme_z_mg,roll_der,pitch_der,yaw_der,'
          'ekf_bayrak,ekf_hiz_var,ekf_konum_var,gps_ekf_fark_m,'
          'gercek_enlem,gercek_boylam,gercek_sapma_m')


class Hata(Exception):
    pass


class Dusme(Exception):
    pass


# ---------------------------------------------------------------------------
# KISIM 2: PROGRAMIN HAFIZASI
# ---------------------------------------------------------------------------
son = {}
param_degerleri = {}
durum = {}
baglanti = None
dosya = None
olay_dosya = None
T0 = 0.0


def sifirla():
    global son, param_degerleri, durum
    son = {k: None for k in (
        'armed', 'irtifa', 'gps_enlem', 'gps_boylam', 'gps_hiz', 'gps_rota', 'gps_eph', 'uydu', 'fix',
        'ekf_enlem', 'ekf_boylam', 'ekf_vx', 'ekf_vy', 'ekf_vz',
        'lx', 'ly', 'lz', 'lvx', 'lvy', 'lvz', 'ax', 'ay', 'az', 'roll', 'pitch', 'yaw',
        'ekf_bayrak', 'ekf_hiz_var', 'ekf_konum_var', 'gercek_enlem', 'gercek_boylam')}
    param_degerleri = {}
    durum = {'etiket': 'hazirlik', 'glitch_m': 0.0, 'havada': False,
             'sonraki_yazim': 0.0, 'son_yazdirma': 0.0, 'son_paket': 0.0, 'satir': 0}


def olay(metin):
    t = time.monotonic() - T0
    print(f'[{t:7.1f} sn] {datetime.now().strftime("%H:%M:%S.%f")[:-3]} {metin}')
    if olay_dosya is not None:
        olay_dosya.write(f'{t:.1f},{metin.replace(",", ";")}\n')
        olay_dosya.flush()


# ---------------------------------------------------------------------------
# KISIM 3: DRONE'DAN OKUMA
# ---------------------------------------------------------------------------
def guncelle(m):
    tip = m.get_type()
    if tip == 'HEARTBEAT':
        if m.type != mavutil.mavlink.MAV_TYPE_GCS:
            son['armed'] = 1 if (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) else 0
            baglanti.target_system = m.get_srcSystem()
            baglanti.target_component = m.get_srcComponent()
    elif tip == 'GPS_RAW_INT':                        # HAM GPS: spoofing'in dogrudan etkiledigi veri
        son['gps_enlem'] = m.lat / 1e7
        son['gps_boylam'] = m.lon / 1e7
        son['gps_hiz'] = None if m.vel == 65535 else m.vel / 100.0          # cm/s -> m/s
        son['gps_rota'] = None if m.cog == 65535 else m.cog / 100.0         # derece*100 -> derece
        son['gps_eph'] = None if m.eph == 65535 else m.eph / 100.0          # HDOP
        son['uydu'] = None if m.satellites_visible == 255 else m.satellites_visible
        son['fix'] = m.fix_type
    elif tip == 'GLOBAL_POSITION_INT':                # EKF TAHMINI: ArduPilot'un "ben buradayim" dedigi yer
        son['irtifa'] = round(m.relative_alt / 1000.0, 2)
        son['ekf_enlem'] = m.lat / 1e7
        son['ekf_boylam'] = m.lon / 1e7
        son['ekf_vx'] = m.vx / 100.0                  # kuzey hizi (m/s)
        son['ekf_vy'] = m.vy / 100.0                  # dogu hizi
        son['ekf_vz'] = m.vz / 100.0                  # asagi hizi
    elif tip == 'LOCAL_POSITION_NED':
        son['lx'], son['ly'], son['lz'] = round(m.x, 3), round(m.y, 3), round(m.z, 3)
        son['lvx'], son['lvy'], son['lvz'] = round(m.vx, 3), round(m.vy, 3), round(m.vz, 3)
    elif tip == 'RAW_IMU':                            # IVMEOLCER: GPS'ten bagimsiz, spoofing etkilemez
        son['ax'], son['ay'], son['az'] = m.xacc, m.yacc, m.zacc
    elif tip == 'ATTITUDE':
        son['roll'] = round(math.degrees(m.roll), 2)
        son['pitch'] = round(math.degrees(m.pitch), 2)
        son['yaw'] = round(math.degrees(m.yaw), 2)
    elif tip == 'EKF_STATUS_REPORT':
        son['ekf_bayrak'] = m.flags
        son['ekf_hiz_var'] = round(m.velocity_variance, 4)
        son['ekf_konum_var'] = round(m.pos_horiz_variance, 4)
    elif tip == 'SIMSTATE':                           # GERCEK KONUM: sadece simulator bilir, tespitte KULLANILMAZ
        son['gercek_enlem'] = m.lat / 1e7             #  (rota sapmasini olcmek icin)
        son['gercek_boylam'] = m.lng / 1e7
    elif tip == 'PARAM_VALUE':
        pid = m.param_id
        if isinstance(pid, bytes):
            pid = pid.decode('utf-8', 'ignore')
        param_degerleri[pid.rstrip('\x00')] = m.param_value


def uzaklik_m(enlem1, boylam1, enlem2, boylam2):
    """Iki konum arasindaki yatay uzaklik (metre). Biri eksikse None."""
    if None in (enlem1, boylam1, enlem2, boylam2):
        return None
    dkuzey = (enlem1 - enlem2) * 111320.0
    ddogu = (boylam1 - boylam2) * 111320.0 * math.cos(math.radians(enlem2))
    return round(math.hypot(dkuzey, ddogu), 2)


def gps_ekf_fark_m():
    """Ham GPS konumu ile EKF tahmini arasindaki yatay uzaklik (metre)."""
    return uzaklik_m(son['gps_enlem'], son['gps_boylam'], son['ekf_enlem'], son['ekf_boylam'])


def gercek_sapma_m():
    """Aracin GERCEKTE oldugu yer ile kendini sandigi yer (EKF) arasindaki uzaklik (metre)."""
    return uzaklik_m(son['gercek_enlem'], son['gercek_boylam'], son['ekf_enlem'], son['ekf_boylam'])


def satir_yaz(simdi):
    if simdi - durum['son_paket'] > 3:
        return
    if son['armed'] is None or son['irtifa'] is None or son['gps_enlem'] is None:
        return
    t = simdi - T0
    fark = gps_ekf_fark_m()
    alanlar = [
        f'{t:.2f}', durum['etiket'], round(durum['glitch_m'], 2), son['armed'], baglanti.flightmode, son['irtifa'],
        son['gps_enlem'], son['gps_boylam'], son['gps_hiz'], son['gps_rota'], son['gps_eph'], son['uydu'], son['fix'],
        son['ekf_enlem'], son['ekf_boylam'], son['ekf_vx'], son['ekf_vy'], son['ekf_vz'],
        son['lx'], son['ly'], son['lz'], son['lvx'], son['lvy'], son['lvz'],
        son['ax'], son['ay'], son['az'], son['roll'], son['pitch'], son['yaw'],
        son['ekf_bayrak'], son['ekf_hiz_var'], son['ekf_konum_var'], fark,
        son['gercek_enlem'], son['gercek_boylam'], gercek_sapma_m(),
    ]
    dosya.write(','.join('' if a is None else str(a) for a in alanlar) + '\n')
    dosya.flush()
    durum['satir'] += 1
    if simdi - durum['son_yazdirma'] >= 5:
        durum['son_yazdirma'] = simdi
        print(f"{t:6.1f} sn | {durum['etiket']:<18} | kayma {durum['glitch_m']:5.1f} m | "
              f"GPS-EKF farki {fark} m | gercek sapma {gercek_sapma_m()} m | irtifa {son['irtifa']} | mod {baglanti.flightmode}")


# ---------------------------------------------------------------------------
# KISIM 4: CEKIRDEK DONGU, BEKLEME, KOMUTLAR (otomatik_ucus.py ile ayni kalip)
# ---------------------------------------------------------------------------
def adim():
    m = baglanti.recv_match(blocking=True, timeout=0.05)
    simdi = time.monotonic()
    if m is not None:
        durum['son_paket'] = simdi
        guncelle(m)
    if simdi >= durum['sonraki_yazim']:
        durum['sonraki_yazim'] += PERIYOT
        if simdi - durum['sonraki_yazim'] > 2 * PERIYOT:
            durum['sonraki_yazim'] = simdi + PERIYOT
        satir_yaz(simdi)
    if durum['havada'] and son['irtifa'] is not None and son['irtifa'] < DUSME_ESIGI:
        raise Dusme(f"irtifa {son['irtifa']} m (esik {DUSME_ESIGI} m)")
    if durum['havada'] and baglanti.flightmode not in ('GUIDED', None):
        raise Hata(f'mod kendiliginden {baglanti.flightmode} oldu (ArduPilot failsafe olabilir)')


def bekle_kosul(kosul, sn):
    bitis = time.monotonic() + sn
    while time.monotonic() < bitis:
        adim()
        if kosul():
            return True
    return kosul()


def bekle(sn):
    bekle_kosul(lambda: False, sn)


def komut(cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
    baglanti.mav.command_long_send(
        baglanti.target_system, baglanti.target_component, cmd, 0, p1, p2, p3, p4, p5, p6, p7)


def mesaj_hizlarini_iste():
    for msg_id, hz in MESAJ_HIZLARI.items():
        komut(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, int(1e6 / hz))
        bekle(0.1)
    olay('mesaj hizlari istendi: ' + ' '.join(f'{k}:{v}Hz' for k, v in MESAJ_HIZLARI.items()))


def mod_ayarla(ad):
    for _ in range(3):
        komut(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, MODLAR[ad])
        if bekle_kosul(lambda: baglanti.flightmode == ad, 4):
            olay(f'mod {ad}')
            return
    raise Hata(f'{ad} moduna gecilemedi (mod su an {baglanti.flightmode})')


def param_ayarla(ad, deger):
    for _ in range(3):
        param_degerleri.pop(ad, None)
        baglanti.param_set_send(ad, float(deger))
        if bekle_kosul(lambda: ad in param_degerleri and abs(param_degerleri[ad] - deger) < 1e-9 + 1e-6 * abs(deger), 3):
            olay(f'param {ad} = {deger}')
            return
    raise Hata(f'{ad} = {deger} SITL tarafindan onaylanmadi')


def glitch_ayarla(metre, onay=True):
    """GPS konumunu 'metre' kadar kuzeye kaydir. onay=False ise sadece gonderir (yavas kaymada)."""
    derece = metre * M_DERECE
    if onay:
        param_ayarla('SIM_GPS1_GLTCH_X', derece)
    else:
        baglanti.param_set_send('SIM_GPS1_GLTCH_X', float(derece))
    durum['glitch_m'] = metre


def arm_et():
    for _ in range(24):
        komut(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        if bekle_kosul(lambda: son['armed'] == 1, 5):
            olay('ARM tamam')
            return
        olay('ARM olmadi (GPS / pre-arm bekleniyor olabilir), tekrar deneniyor')
    raise Hata('arm edilemedi')


def kalkis():
    komut(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, IRTIFA)
    olay(f'kalkis komutu, hedef {IRTIFA} m')
    if not bekle_kosul(lambda: son['irtifa'] is not None and son['irtifa'] >= IRTIFA - 1, 60):
        raise Hata(f"{IRTIFA - 1} m'ye cikilamadi (irtifa {son['irtifa']})")
    olay(f"hedef irtifaya ulasildi ({son['irtifa']} m)")


# ---------------------------------------------------------------------------
# KISIM 5: ANA SENARYO
# ---------------------------------------------------------------------------
def spoof_uygula(etiket, tur, deger, sure):
    olay(f'SPOOF BASLIYOR: {etiket}')
    if tur == 'ani':
        glitch_ayarla(deger)                 # SITL onaylayinca etiket degisir
        durum['etiket'] = etiket
        bekle(sure)
    else:                                    # yavas: her YAVAS_ADIM_SN'de biraz daha kaydir
        durum['etiket'] = etiket
        bas = time.monotonic()
        while time.monotonic() - bas < sure:
            glitch_ayarla(deger * (time.monotonic() - bas), onay=False)
            bekle(YAVAS_ADIM_SN)
        glitch_ayarla(deger * sure)          # son degeri onaylat (kaymanin gercekten uygulandigini dogrula)
    olay(f"SPOOF SURESI BITTI: {etiket} (son kayma {durum['glitch_m']:.1f} m)")
    glitch_ayarla(0.0)                       # geri al: bu da ters yonde bir sicramadir
    durum['etiket'] = 'toparlanma'           # geri donus aninin satirlari ayri etiketlenir, normal sayilmaz
    bekle(TOPARLANMA_SN)
    durum['etiket'] = 'normal'
    olay(f'TOPARLANMA BITTI: {etiket}')


def ucus(plan, gurultu):
    for ad, v in TEMIZLIK:                   # onceki denemeden kalan kayma ve gurultu olmasin
        param_ayarla(ad, v)
    if gurultu:
        for ad, v in GURULTU:
            param_ayarla(ad, v)
    param_ayarla('DISARM_DELAY', 60)
    mesaj_hizlarini_iste()
    mod_ayarla('GUIDED')
    arm_et()
    kalkis()
    durum['havada'] = True
    olay(f'kararlasma bekleniyor ({KARARLI_SN} sn)')
    bekle(KARARLI_SN)
    durum['etiket'] = 'normal'
    olay(f'NORMAL ucus basladi ({BASLANGIC_SN} sn)')
    bekle(BASLANGIC_SN)
    for senaryo in plan:
        spoof_uygula(*senaryo)
    durum['havada'] = False
    mod_ayarla('LAND')
    if not bekle_kosul(lambda: son['armed'] == 0, 120):
        raise Hata('inis sonrasi disarm olmadi')
    olay('inis tamam, disarm')


def guvenli_son():
    durum['havada'] = False
    try:
        for ad, v in TEMIZLIK:
            try:
                param_ayarla(ad, v)
            except Hata as e:
                olay(f'UYARI: temizlik basarisiz: {e}')
        durum['glitch_m'] = 0.0
        if son['armed'] == 1:
            durum['etiket'] = 'sonlandirma'
            komut(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, MODLAR['LAND'])
            olay('LAND komutu gonderildi')
            bekle_kosul(lambda: son['armed'] == 0, 120)
    except KeyboardInterrupt:
        olay('ikinci Ctrl+C: temizlik yarida kesildi')


def main(argv=None):
    global baglanti, dosya, olay_dosya, T0
    argv = sys.argv if argv is None else argv
    args = argv[1:]
    if not args:
        print(__doc__)
        return 2
    ad = args[0]
    ekstra = args[1:]
    gurultu = 'gurultu' in ekstra
    secimler = [a for a in ekstra if a != 'gurultu']
    secim = secimler[0] if secimler else 'hepsi'
    if secim not in ('ani', 'yavas', 'hepsi'):
        print("secim 'ani', 'yavas' ya da 'hepsi' olmali")
        return 2
    if gurultu:
        ad = ad + '_gurultu'
    plan = [s for s in SENARYOLAR if secim == 'hepsi' or s[1] == secim]

    os.makedirs(KLASOR, exist_ok=True)
    damga = datetime.now().strftime('%Y%m%d_%H%M%S')
    dosya_adi = os.path.join(KLASOR, f'spoof_{ad}_{damga}.csv')
    olay_adi = os.path.join(KLASOR, f'spoof_{ad}_{damga}_olaylar.csv')

    sifirla()
    baglanti = mavutil.mavlink_connection(BAGLANTI)
    print(f'{BAGLANTI} dinleniyor, heartbeat bekleniyor...')
    if baglanti.wait_heartbeat(timeout=15) is None:
        print("Heartbeat gelmedi. MAVProxy'de su komutu yaz: output add 127.0.0.1:14553")
        return 1

    dosya = open(dosya_adi, 'w')
    dosya.write(BASLIK + '\n')
    olay_dosya = open(olay_adi, 'w')
    olay_dosya.write('zaman_sn,olay\n')
    T0 = time.monotonic()
    durum['son_yazdirma'] = durum['sonraki_yazim'] = durum['son_paket'] = T0
    toplam = BASLANGIC_SN + sum(s[3] + TOPARLANMA_SN for s in plan)
    print(f'Kayit: {dosya_adi}')
    print(f"{len(plan)} senaryo | sadece ucus kismi yaklasik {toplam} sn ({toplam / 60:.1f} dk)")
    olay(('GURULTU ACIK | ' if gurultu else '') + 'plan: ' + ' '.join(s[0] for s in plan))

    cikis = 0
    try:
        ucus(plan, gurultu)
    except Dusme as e:
        durum['etiket'] = 'dusme'
        olay(f'DUSME ALGILANDI: {e}')
        cikis = 1
    except KeyboardInterrupt:
        olay('kullanici durdurdu (Ctrl+C)')
        cikis = 1
    except Hata as e:
        olay(f'HATA: {e}')
        cikis = 1
    finally:
        guvenli_son()
        dosya.close()
        olay_dosya.close()
    print(f"Kaydedilen satir: {durum['satir']}")
    print(f'Dosya: {dosya_adi}')
    return cikis


if __name__ == '__main__':
    sys.exit(main())
