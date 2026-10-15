from pymavlink import mavutil
from prometheus_client import start_http_server, Gauge
import time

# --- Prometheus metriklerini tanımla ---

batarya_voltaji = Gauge('drone_battery_voltage', 'Batarya voltajı (V)', ['drone_id'])
batarya_yuzdesi = Gauge('drone_battery_remaining', 'Kalan batarya yüzdesi', ['drone_id'])
irtifa = Gauge('drone_altitude', 'Yerden irtifa (m)', ['drone_id'])
yer_hizi = Gauge('drone_groundspeed', 'Yer hızı (m/s)', ['drone_id'])
gps_uydu = Gauge('drone_gps_satellites', 'GPS uydu sayısı (numSV)', ['drone_id'])
gps_hdop = Gauge('drone_gps_hdop', 'GPS yatay doğruluk (HDOP)', ['drone_id'])
gps_fix = Gauge('drone_gps_fix_type', 'GPS fix tipi (0-1 = fix yok, 3+ = 3D fix)', ['drone_id'])
gps_jamming = Gauge('drone_gps_jamming_alarm', 'GPS jamming alarmı (1=jamming şüphesi)', ['drone_id'])
baglanti_durumu = Gauge('drone_connected', 'Bağlantı durumu (1=bağlı, 0=değil)', ['drone_id'])

DRONE_ID = 'sitl_1'

# Jamming alarmı kilidi: simülatör jamming'li paketleri aralıklı gönderiyor
# (ölçüm: 120 saniyede 481 paketin 138'i şüpheli, yaklaşık %29). Tek pakete
# bakmak yerine, son şüpheli paketten sonra alarmı JAM_TUTMA_SN saniye
# boyunca açık tutuyoruz.
JAM_TUTMA_SN = 10

# 'hazir': GPS'ten ilk sağlıklı (3D fix ve yeterli uydu) paket gelene kadar
# False kalır. Açılışta GPS henüz kilitlenmemişken "fix yok, uydu yok" durumu
# jamming'den ayırt edilemez, o yüzden bu sürede kural alarm üretmez.
# 'son': son şüpheli paketin zamanı (time.monotonic ile), henüz yoksa None.
# time.monotonic geri sarmaz; WSL'in "time moved backwards" saat sıçramalarından
# etkilenmez (time.time etkilenirdi).
jam_durum = {'son': None, 'hazir': False}


def main():
    # Prometheus'un okuyacağı /metrics endpoint'ini başlat (port 8000)
    start_http_server(8000)
    print("Exporter başladı: http://localhost:8000/metrics")

    print("SITL'e bağlanılıyor...")
    baglanti = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
    baglanti.wait_heartbeat()
    print(f"Bağlantı kuruldu! Sistem ID: {baglanti.target_system}")
    baglanti_durumu.labels(drone_id=DRONE_ID).set(1)

    print("Veri okunuyor ve Prometheus metriklerine yazılıyor... (Ctrl+C ile durdur)")

    while True:
        mesaj = baglanti.recv_match(blocking=True, timeout=5)

        if mesaj is None:
            # 5 saniye içinde hiç veri gelmediyse, bağlantı kopmuş olabilir
            baglanti_durumu.labels(drone_id=DRONE_ID).set(0)
            continue

        baglanti_durumu.labels(drone_id=DRONE_ID).set(1)
        tip = mesaj.get_type()

        if tip == 'BATTERY_STATUS':
            if mesaj.voltages[0] != 65535:  # sentinel değeri hatırlıyoruz
                batarya_voltaji.labels(drone_id=DRONE_ID).set(mesaj.voltages[0] / 1000.0)
            batarya_yuzdesi.labels(drone_id=DRONE_ID).set(mesaj.battery_remaining)

        elif tip == 'GLOBAL_POSITION_INT':
            irtifa.labels(drone_id=DRONE_ID).set(mesaj.relative_alt / 1000.0)

        elif tip == 'VFR_HUD':
            yer_hizi.labels(drone_id=DRONE_ID).set(mesaj.groundspeed)

        elif tip == 'GPS_RAW_INT':
            uydu = mesaj.satellites_visible
            gps_uydu.labels(drone_id=DRONE_ID).set(0 if uydu == 255 else uydu)
            hdop = mesaj.eph / 100.0
            gps_hdop.labels(drone_id=DRONE_ID).set(99.99 if mesaj.eph == 65535 else hdop)
            gps_fix.labels(drone_id=DRONE_ID).set(mesaj.fix_type)

            simdi = time.monotonic()
            saglikli = mesaj.fix_type >= 3 and uydu >= 6

            # İlk sağlıklı paket: GPS kilitlendi, jamming kuralı bundan sonra devrede
            if saglikli and not jam_durum['hazir']:
                jam_durum['hazir'] = True
                print("GPS ilk 3D fix'i aldı, jamming tespiti devrede.")

            # Anlık şüphe: bu paket jamming belirtisi taşıyor mu? (sadece hazırsa)
            if jam_durum['hazir'] and not saglikli:
                jam_durum['son'] = simdi

            # Alarm: son şüpheli paketten bu yana JAM_TUTMA_SN geçmediyse açık
            jamming_aktif = (
                jam_durum['son'] is not None
                and (simdi - jam_durum['son']) < JAM_TUTMA_SN
            )
            gps_jamming.labels(drone_id=DRONE_ID).set(1 if jamming_aktif else 0)


if __name__ == '__main__':
    main()