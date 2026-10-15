from pymavlink import mavutil
import time

SURE = 600
baglanti = mavutil.mavlink_connection('udpin:127.0.0.1:14552')
uydu = 0
fix = 0
hiz = 0.0
irtifalar = []
baslangic = time.time()
dosya = open('irtifa_kaydi.csv', 'w')
dosya.write('zaman_sn,relative_alt_m,groundspeed,uydu,fix\n')
while time.time() - baslangic < SURE:
    m = baglanti.recv_match(type=['GLOBAL_POSITION_INT', 'VFR_HUD', 'GPS_RAW_INT'], blocking=True, timeout=5)
    if m is None:
        continue
    tip = m.get_type()
    if tip == 'GPS_RAW_INT':
        uydu = m.satellites_visible
        fix = m.fix_type
    elif tip == 'VFR_HUD':
        hiz = m.groundspeed
    elif tip == 'GLOBAL_POSITION_INT':
        irt = m.relative_alt / 1000.0
        irtifalar.append(irt)
        t = time.time() - baslangic
        dosya.write(f'{t:.1f},{irt:.2f},{hiz:.1f},{uydu},{fix}\n')
dosya.close()
if irtifalar:
    print('paket sayisi:', len(irtifalar))
    print('en dusuk irtifa:', min(irtifalar))
    print('en yuksek irtifa:', max(irtifalar))
    print('-0.5 m altindaki paket sayisi:', len([x for x in irtifalar if x < -0.5]))
else:
    print('hic paket gelmedi')