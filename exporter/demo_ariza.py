"""
Canli demo: tek tusla ariza ver / geri al.

Kullanim (proje kokunden, sistem baslat.sh ile acikken, otomatik_ucus.py CALISMIYORKEN):
  venv/bin/python exporter/demo_ariza.py

otomatik_ucus.py ile ayni portu (14553) ve ayni yontemi kullanir: parametreyi gonderir,
simulatorun onayladigini (PARAM_VALUE) bekler. Arizalar otomatik_ucus.py'deki ARIZALAR
listesindekilerle ayni parametreler.
"""
import time
from datetime import datetime
from pymavlink import mavutil

BAGLANTI = 'udpin:127.0.0.1:14553'
IRTIFA = 10  # otomatik_ucus.py ile ayni

# tus: (aciklama, [(parametre, deger)] SIRAYLA)
# Motor: once carpan (MUL), sonra hangi motor (FAIL bit maskesi: 1=m1, 2=m2, 4=m3, 8=m4).
MENU = {
    '1': ('Motor 1 gucu %70', [('SIM_ENGINE_MUL', 0.7), ('SIM_ENGINE_FAIL', 1)]),
    '2': ('Motor 3 gucu %85', [('SIM_ENGINE_MUL', 0.85), ('SIM_ENGINE_FAIL', 4)]),
    '3': ('Batarya ic direnci 0,05 ohm', [('SIM_BATT_RES_OHM', 0.05)]),
    '4': ('Batarya voltaji 11,8 V', [('SIM_BATT_VOLTAGE', 11.8)]),
    '5': ('GPS jamming', [('SIM_GPS1_JAM', 1)]),
    # Geri alma: once FAIL kapanir; direnc -1 DEGIL 0.01 (otomatik_ucus.py notu)
    '0': ('HEPSINI GERI AL', [('SIM_ENGINE_FAIL', 0), ('SIM_BATT_RES_OHM', 0.01),
                              ('SIM_BATT_VOLTAGE', 12.6), ('SIM_GPS1_JAM', 0)]),
}
MODLAR = {'GUIDED': 4, 'LAND': 9}


def saat():
    return datetime.now().strftime('%H:%M:%S')


def param_ayarla(b, ad, deger):
    for _ in range(3):
        b.param_set_send(ad, float(deger))
        bitis = time.time() + 3
        while time.time() < bitis:
            m = b.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
            if m is None:
                continue
            pid = m.param_id if isinstance(m.param_id, str) else m.param_id.decode(errors='ignore')
            if pid.strip('\x00') == ad and abs(m.param_value - float(deger)) < 1e-3:
                print(f'   {ad} = {deger}  (simulator onayladi)')
                return True
    print(f'   UYARI: {ad} = {deger} onaylanmadi!')
    return False


def komut(b, cmd, *p):
    p = list(p) + [0] * (7 - len(p))
    b.mav.command_long_send(b.target_system, b.target_component, cmd, 0, *p)


def mod(b, ad):
    komut(b, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, MODLAR[ad])


def kalkis(b):
    param_ayarla(b, 'DISARM_DELAY', 60)
    mod(b, 'GUIDED'); time.sleep(2)
    komut(b, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1); time.sleep(2)
    komut(b, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, IRTIFA)
    print(f'[{saat()}] Kalkis komutu gonderildi ({IRTIFA} m). Model, 9 m\'ye cikip 5 sn gectikten sonra karar vermeye baslar.')


def main():
    print('Simulatore baglaniliyor (14553)...')
    b = mavutil.mavlink_connection(BAGLANTI)
    b.wait_heartbeat()
    print('Baglandi.\n')
    while True:
        print('k = kalkis (GUIDED, 10 m)   i = inis')
        for t, (ac, _) in MENU.items():
            print(f'{t} = {ac}')
        print('q = cikis')
        sec = input('> ').strip().lower()
        if sec == 'q':
            break
        if sec == 'k':
            kalkis(b)
        elif sec == 'i':
            mod(b, 'LAND'); print(f'[{saat()}] Inis (LAND).')
        elif sec in MENU:
            ac, adimlar = MENU[sec]
            print(f'[{saat()}] {ac}')
            for p, v in adimlar:
                param_ayarla(b, p, v)
        else:
            print('Bilinmeyen tus.')
        print()


if __name__ == '__main__':
    main()
