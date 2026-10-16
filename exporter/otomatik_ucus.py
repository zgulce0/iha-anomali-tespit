"""Otomatik ucus + ariza enjeksiyonu + kayit (SITL).

Kullanim (SITL acik, kayit_ucus.py KAPALI):
    ../venv/bin/python otomatik_ucus.py <ad> [filtre] [daire]

    ad      dosya adinda kullanilir (ornek: motor_deneme)
    filtre  etiketinde bu yazi gecen arizalar uygulanir (ornek: motor1_0.85, batt, motor3)
            bos birakmak ya da 'hepsi' yazmak tum arizalari uygular
    daire   yazilirsa hover yerine LOITER ve CIRCLE modunda ucar

Gerekli: MAVProxy'nin bu porta da cikis vermesi (--out=127.0.0.1:14553).
Etiket sutunu, arizanin gercekten acik oldugu araligi gosterir: param komutu
SITL tarafindan onaylandiktan sonra degisir. Sinyallerden turetilmez.
"""
import math
import os
import random
import sys
import time
from datetime import datetime

from pymavlink import mavutil

BAGLANTI = 'udpin:127.0.0.1:14553'
KLASOR = os.path.expanduser('~/iha-anomali-tespit/veri')

IRTIFA = 10            # metre
BASLANGIC_SN = 40      # arizasiz normal ucus suresi
ARIZA_SN = 30          # her arizanin acik kaldigi sure
TOPARLANMA_SN = 30     # ariza kapandiktan sonra normal bekleme
PERIYOT = 1.0          # kayit periyodu (sn)
DUSME_ESIGI = 5.0      # havadayken bunun altina inersek ucus bozuldu say (m)
KARARLI_SN = 10        # hedef irtifaya cikinca, 'normal' etiketi baslamadan once kararlilasma suresi
BATARYA_TOL_V = 0.05   # normal batarya modeli V = 12.6 - 0.01*I'dan en fazla bu kadar sapabilir (V)

BASLIK = ('zaman_sn,etiket,armed,voltaj_v,akim_a,kalan_yuzde,irtifa_m,hiz_ms,gaz_yuzde,'
          'tirmanma_ms,motor1_pwm,motor2_pwm,motor3_pwm,motor4_pwm,uydu,enlem,boylam,'
          'mod,roll_der,pitch_der')

# ArduCopter ozel mod numaralari
MODLAR = {'GUIDED': 4, 'LOITER': 5, 'CIRCLE': 7, 'LAND': 9}

# (etiket, uygulanacak [(param, deger)] SIRAYLA, geri alinacak [(param, deger)] SIRAYLA)
# Motor arizasinda MUL once ayarlanir, FAIL en son acilir (MUL 0.0 iken FAIL acilirsa
# motor tamamen durur). Geri alirken once FAIL kapatilir.
# Ic direnc geri alinirken -1 YAZILMAZ: 16 Ekim denemesinde -1 direnci varsayilana
# dondurmedi, 0.05'te kaldi. Normal direnc 0.01 (V = 12.6 - 0.01*I olculdu).
ARIZALAR = []
for _v in (12.5, 12.2, 11.8, 11.5):
    ARIZALAR.append((f'batt_volt_{_v}', [('SIM_BATT_VOLTAGE', _v)], [('SIM_BATT_VOLTAGE', 12.6)]))
for _r in (0.02, 0.03, 0.05):
    ARIZALAR.append((f'batt_res_{_r}', [('SIM_BATT_RES_OHM', _r)], [('SIM_BATT_RES_OHM', 0.01)]))
for _m in (1, 2, 3, 4):
    for _mul in (0.95, 0.9, 0.85, 0.7):
        ARIZALAR.append((f'motor{_m}_{_mul}',
                         [('SIM_ENGINE_MUL', _mul), ('SIM_ENGINE_FAIL', 1 << (_m - 1))],
                         [('SIM_ENGINE_FAIL', 0)]))

# Ucus bitince / hata olunca SITL'i temiz birakmak icin (FAIL, MUL'dan once kapanir)
TEMIZLIK = [('SIM_ENGINE_FAIL', 0), ('SIM_ENGINE_MUL', 0.0),
            ('SIM_BATT_VOLTAGE', 12.6), ('SIM_BATT_RES_OHM', 0.01)]


class Hata(Exception):
    pass


class Dusme(Exception):
    pass


son = {}
param_degerleri = {}
durum = {}
baglanti = None
dosya = None
olay_dosya = None
T0 = 0.0


def sifirla():
    global son, param_degerleri, durum
    son = {
        'armed': None, 'voltaj': None, 'akim': None, 'kalan': None,
        'irtifa': None, 'hiz': None, 'gaz': None, 'tirmanma': None,
        'm1': None, 'm2': None, 'm3': None, 'm4': None,
        'uydu': None, 'enlem': None, 'boylam': None, 'roll': None, 'pitch': None,
    }
    param_degerleri = {}
    durum = {
        'etiket': 'hazirlik', 'havada': False, 'rc_aktif': False, 'sonraki_rc': 0.0,
        'sonraki_yazim': 0.0, 'son_yazdirma': 0.0, 'son_paket': 0.0, 'satir': 0,
    }


def olay(metin):
    t = time.monotonic() - T0
    print(f'[{t:7.1f} sn] {metin}')
    if olay_dosya is not None:
        olay_dosya.write(f'{t:.1f},{metin.replace(",", ";")}\n')  # CSV bozulmasin
        olay_dosya.flush()


def guncelle(m):
    tip = m.get_type()
    if tip == 'HEARTBEAT':
        if m.type != mavutil.mavlink.MAV_TYPE_GCS:
            son['armed'] = 1 if (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) else 0
            baglanti.target_system = m.get_srcSystem()
            baglanti.target_component = m.get_srcComponent()
    elif tip == 'BATTERY_STATUS':
        v = m.voltages[0]
        son['voltaj'] = None if v == 65535 else round(v / 1000.0, 3)
        son['akim'] = None if m.current_battery == -1 else round(m.current_battery / 100.0, 2)
        son['kalan'] = None if m.battery_remaining == -1 else m.battery_remaining
    elif tip == 'GLOBAL_POSITION_INT':
        son['irtifa'] = round(m.relative_alt / 1000.0, 2)
        son['enlem'] = m.lat / 1e7
        son['boylam'] = m.lon / 1e7
    elif tip == 'VFR_HUD':
        son['hiz'] = round(m.groundspeed, 2)
        son['gaz'] = m.throttle
        son['tirmanma'] = round(m.climb, 2)
    elif tip == 'SERVO_OUTPUT_RAW':
        son['m1'] = m.servo1_raw
        son['m2'] = m.servo2_raw
        son['m3'] = m.servo3_raw
        son['m4'] = m.servo4_raw
    elif tip == 'GPS_RAW_INT':
        son['uydu'] = None if m.satellites_visible == 255 else m.satellites_visible
    elif tip == 'ATTITUDE':
        son['roll'] = round(math.degrees(m.roll), 1)
        son['pitch'] = round(math.degrees(m.pitch), 1)
    elif tip == 'PARAM_VALUE':
        pid = m.param_id
        if isinstance(pid, bytes):
            pid = pid.decode('utf-8', 'ignore')
        param_degerleri[pid.rstrip('\x00')] = m.param_value


def satir_yaz(simdi):
    if simdi - durum['son_paket'] > 3:
        return
    if son['armed'] is None or son['voltaj'] is None or son['irtifa'] is None:
        return
    t = simdi - T0
    alanlar = [
        f'{t:.1f}', durum['etiket'], son['armed'], son['voltaj'], son['akim'], son['kalan'],
        son['irtifa'], son['hiz'], son['gaz'], son['tirmanma'],
        son['m1'], son['m2'], son['m3'], son['m4'],
        son['uydu'], son['enlem'], son['boylam'], baglanti.flightmode,
        son['roll'], son['pitch'],
    ]
    dosya.write(','.join('' if a is None else str(a) for a in alanlar) + '\n')
    dosya.flush()
    durum['satir'] += 1
    if simdi - durum['son_yazdirma'] >= 10:
        durum['son_yazdirma'] = simdi
        print(f"{t:5.0f} sn | {durum['etiket']:<16} | armed {son['armed']} | V {son['voltaj']} | "
              f"A {son['akim']} | irtifa {son['irtifa']} | gaz {son['gaz']} | "
              f"motor {son['m1']} {son['m2']} {son['m3']} {son['m4']} | "
              f"roll {son['roll']} | mod {baglanti.flightmode}")


def rc_gonder():
    # MAVProxy'deki "rc 3 1500" ile ayni sey: gaz kanalini ortada tut.
    # Override en fazla birkac sn gecerli kalir, bu yuzden surekli tekrar gonderilir.
    baglanti.mav.rc_channels_override_send(
        baglanti.target_system, baglanti.target_component,
        65535, 65535, 1500, 65535, 65535, 65535, 65535, 65535)


def adim():
    """Tek dongu: bir mesaj oku, gerekirse RC gonder, 1 Hz satir yaz, dusmeyi kontrol et."""
    m = baglanti.recv_match(blocking=True, timeout=0.2)
    simdi = time.monotonic()
    if m is not None:
        durum['son_paket'] = simdi
        guncelle(m)
    if durum['rc_aktif'] and simdi >= durum['sonraki_rc']:
        rc_gonder()
        durum['sonraki_rc'] = simdi + 0.5
    if simdi >= durum['sonraki_yazim']:
        durum['sonraki_yazim'] += PERIYOT
        if simdi - durum['sonraki_yazim'] > 2 * PERIYOT:
            durum['sonraki_yazim'] = simdi + PERIYOT
        satir_yaz(simdi)
    if durum['havada'] and son['irtifa'] is not None and son['irtifa'] < DUSME_ESIGI:
        raise Dusme(f"irtifa {son['irtifa']} m (esik {DUSME_ESIGI} m)")


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
        if bekle_kosul(lambda: ad in param_degerleri and abs(param_degerleri[ad] - deger) < 1e-3, 3):
            olay(f'param {ad} = {deger}')
            return
    raise Hata(f'{ad} = {deger} SITL tarafindan onaylanmadi')


def arm_et():
    for _ in range(24):   # yaklasik 2 dakika
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


def batarya_normal_mi():
    """Voltaj, akimdan beklenen degere uyuyor mu? (SITL modeli: V = 12.6 - 0.01*I, 6 akimda olculdu)
    Etiketler komutlardan uretildigi icin, komut ise yaramazsa etiket yalan soyler.
    Bu kontrol, bataryanin gercekten normale dondugunu sinyalden dogrular."""
    if son['voltaj'] is None or son['akim'] is None:
        return True
    return abs(son['voltaj'] - (12.6 - 0.01 * son['akim'])) < BATARYA_TOL_V


def ucus(plan, daire):
    param_ayarla('DISARM_DELAY', 60)
    mod_ayarla('GUIDED')
    arm_et()
    kalkis()
    durum['havada'] = True
    if daire:
        durum['rc_aktif'] = True
        mod_ayarla('LOITER')
        bekle(5)
        mod_ayarla('CIRCLE')
    olay(f'kararlasma bekleniyor ({KARARLI_SN} sn)')
    bekle(KARARLI_SN)
    if not batarya_normal_mi():
        raise Hata(f"baslangicta batarya modeli normal degil (V {son['voltaj']}, A {son['akim']}). "
                   "Onceki kosudan kalan ariza olabilir: baslat.sh ile SITL'i yeniden baslat")
    durum['etiket'] = 'normal'
    olay(f'NORMAL ucus basladi ({BASLANGIC_SN} sn)')
    bekle(BASLANGIC_SN)
    for ad, uygula, geri_al in plan:
        olay(f'ARIZA BASLIYOR: {ad}')
        for p, v in uygula:
            param_ayarla(p, v)
        durum['etiket'] = ad
        bekle(ARIZA_SN)
        for p, v in geri_al:
            param_ayarla(p, v)
        durum['etiket'] = 'normal'
        olay(f'ARIZA BITTI: {ad}')
        bekle(TOPARLANMA_SN)
        if not batarya_normal_mi():
            durum['etiket'] = 'bilinmiyor'
            raise Hata(f"{ad} sonrasi batarya modeli normale donmedi (V {son['voltaj']}, A {son['akim']}): "
                       "bu kosudaki 'normal' etiketleri guvenilmez")
    if daire:
        mod_ayarla('LOITER')
        bekle(10)
    durum['rc_aktif'] = False
    durum['havada'] = False
    mod_ayarla('LAND')
    if not bekle_kosul(lambda: son['armed'] == 0, 120):
        raise Hata('inis sonrasi disarm olmadi')
    olay('inis tamam, disarm')


def guvenli_son():
    """Normal bitis de hata da buradan gecer: SITL'i temiz birak, havadaysak indir."""
    durum['havada'] = False
    durum['rc_aktif'] = False
    try:
        if son['armed'] == 1:
            durum['etiket'] = 'sonlandirma'
            komut(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, MODLAR['LAND'])
            olay('LAND komutu gonderildi')
        for ad, v in TEMIZLIK:
            try:
                param_ayarla(ad, v)
            except Hata as e:
                olay(f'UYARI: temizlik basarisiz: {e}')
        if son['armed'] == 1:
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
    daire = 'daire' in ekstra
    filtreler = [a for a in ekstra if a not in ('daire', 'hepsi')]
    filtre = filtreler[0] if filtreler else None

    secili = [a for a in ARIZALAR if filtre is None or filtre in a[0]]
    if not secili:
        print(f"'{filtre}' ile eslesen ariza yok. Etiketler:")
        print(', '.join(a[0] for a in ARIZALAR))
        return 2
    tohum = int(time.time())
    plan = list(secili)
    random.Random(tohum).shuffle(plan)

    os.makedirs(KLASOR, exist_ok=True)
    damga = datetime.now().strftime('%Y%m%d_%H%M%S')
    dosya_adi = os.path.join(KLASOR, f'ucus_{ad}_{damga}.csv')
    olay_adi = os.path.join(KLASOR, f'ucus_{ad}_{damga}_olaylar.csv')

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
    bas = time.monotonic()
    T0 = bas
    durum['son_yazdirma'] = durum['sonraki_yazim'] = durum['son_paket'] = bas
    toplam = BASLANGIC_SN + len(plan) * (ARIZA_SN + TOPARLANMA_SN)
    print(f'Kayit: {dosya_adi}')
    print(f'Olaylar: {olay_adi}')
    print(f"Mod: {'daire' if daire else 'hover'} | {len(plan)} ariza | sadece ucus kismi yaklasik {toplam} sn")
    olay(f'tohum {tohum}, plan: ' + ' '.join(a[0] for a in plan))

    cikis = 0
    try:
        ucus(plan, daire)
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