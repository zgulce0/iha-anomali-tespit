from pymavlink import mavutil
from prometheus_client import start_http_server, Gauge
import time

# --- Prometheus metriklerini tanımla ---
# Gauge: yukarı da aşağı da gidebilen değerler için (voltaj, irtifa gibi)
batarya_voltaji = Gauge('drone_battery_voltage', 'Batarya voltajı (V)', ['drone_id'])
batarya_yuzdesi = Gauge('drone_battery_remaining', 'Kalan batarya yüzdesi', ['drone_id'])
irtifa = Gauge('drone_altitude', 'Yerden irtifa (m)', ['drone_id'])
baglanti_durumu = Gauge('drone_connected', 'Bağlantı durumu (1=bağlı, 0=değil)', ['drone_id'])

DRONE_ID = 'sitl_1'

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

if __name__ == '__main__':
    main()
