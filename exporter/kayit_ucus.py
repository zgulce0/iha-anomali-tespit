from pymavlink import mavutil
import time
import sys
import os
from datetime import datetime

ETIKET = sys.argv[1] if len(sys.argv) > 1 else 'normal'
SURE = int(sys.argv[2]) if len(sys.argv) > 2 else 600
PERIYOT = 1.0

KLASOR = os.path.expanduser('~/iha-anomali-tespit/veri')
os.makedirs(KLASOR, exist_ok=True)
DOSYA_ADI = os.path.join(KLASOR, f'ucus_{ETIKET}_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv')

BASLIK = 'zaman_sn,etiket,armed,voltaj_v,akim_a,kalan_yuzde,irtifa_m,hiz_ms,gaz_yuzde,tirmanma_ms,motor1_pwm,motor2_pwm,motor3_pwm,motor4_pwm,uydu,enlem,boylam'

son = {
    'armed': None, 'voltaj': None, 'akim': None, 'kalan': None,
    'irtifa': None, 'hiz': None, 'gaz': None, 'tirmanma': None,
    'm1': None, 'm2': None, 'm3': None, 'm4': None,
    'uydu': None, 'enlem': None, 'boylam': None,
}


def guncelle(m):
    tip = m.get_type()
    if tip == 'HEARTBEAT':
        if m.type != mavutil.mavlink.MAV_TYPE_GCS:
            son['armed'] = 1 if (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) else 0
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


baglanti = mavutil.mavlink_connection('udpin:127.0.0.1:14552')
dosya = open(DOSYA_ADI, 'w')
dosya.write(BASLIK + '\n')
print('Kayit basliyor:', DOSYA_ADI)
print(f'Etiket: {ETIKET} | Sure: {SURE} sn | Periyot: {PERIYOT} sn')

baslangic = time.monotonic()
sonraki_yazim = baslangic
son_yazdirma = baslangic
son_paket = baslangic
satir_sayisi = 0
voltajlar = []
irtifalar = []

try:
    while time.monotonic() - baslangic < SURE:
        m = baglanti.recv_match(blocking=True, timeout=0.2)
        simdi = time.monotonic()
        if m is not None:
            son_paket = simdi
            guncelle(m)
        if simdi >= sonraki_yazim:
            sonraki_yazim += PERIYOT
            if simdi - son_paket > 3:
                print('3 sn den uzun suredir paket gelmiyor, SITL acik mi?')
                continue
            t = simdi - baslangic
            alanlar = [
                f'{t:.1f}', ETIKET, son['armed'], son['voltaj'], son['akim'], son['kalan'],
                son['irtifa'], son['hiz'], son['gaz'], son['tirmanma'],
                son['m1'], son['m2'], son['m3'], son['m4'],
                son['uydu'], son['enlem'], son['boylam'],
            ]
            dosya.write(','.join('' if a is None else str(a) for a in alanlar) + '\n')
            dosya.flush()
            satir_sayisi += 1
            if son['voltaj'] is not None:
                voltajlar.append(son['voltaj'])
            if son['irtifa'] is not None:
                irtifalar.append(son['irtifa'])
            if simdi - son_yazdirma >= 10:
                son_yazdirma = simdi
                print(f"{t:.0f} sn | armed {son['armed']} | V {son['voltaj']} | A {son['akim']} | irtifa {son['irtifa']} | hiz {son['hiz']} | gaz {son['gaz']} | motor {son['m1']} {son['m2']} {son['m3']} {son['m4']} | uydu {son['uydu']}")
except KeyboardInterrupt:
    print('Kullanici durdurdu')
finally:
    dosya.close()

print(f'Kaydedilen satir: {satir_sayisi}')
print(f'Dosya: {DOSYA_ADI}')
if voltajlar:
    print(f'Voltaj en dusuk / en yuksek: {min(voltajlar)} / {max(voltajlar)} V')
if irtifalar:
    print(f'En yuksek irtifa: {max(irtifalar)} m')