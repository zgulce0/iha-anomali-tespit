from pymavlink import mavutil
from prometheus_client import start_http_server, Gauge
import os
import time
from datetime import datetime

from anomali_model import MahalanobisModeli, AnomaliDedektoru

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

# Anomali modeli (Mahalanobis): normal uçuştan ne kadar uzak?
anomali_skor = Gauge('drone_anomaly_score', 'Modelin skoru: mesafe / alarm eşiği (1.0 = sınır, üstü anomali)', ['drone_id'])
anomali_alarm = Gauge('drone_anomaly_alarm', 'Donanım anomali alarmı (1=alarm)', ['drone_id'])
anomali_aktif = Gauge('drone_anomaly_active', 'Model şu an karar veriyor mu (1=evet, 0=kapı kapalı)', ['drone_id'])

DRONE_ID = 'sitl_1'

MODEL_YOLU = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'model', 'mahalanobis_v1.json')
# Eğitim verisi saniyede bir kaydedilmişti; model canlıda da saniyede bir bakar.
KARAR_PERIYODU_SN = 1.0

# "Defter": her değerin SON bilinen hali. Mesajlar farklı zamanlarda geliyor
# (motorlar bir mesajda, batarya başka mesajda), her mesaj sadece kendi satırını
# günceller. Model karar verirken hepsine birlikte bakacak.
# None = henüz hiç gelmedi.
son = {'armed': None, 'irtifa': None, 'voltaj': None, 'akim': None,
       'm1': None, 'm2': None, 'm3': None, 'm4': None}

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


def anomali_yaz(skor, alarm):
    """skor None ise model karar vermiyor (kapı kapalı): alarm yok, skor boş."""
    anomali_aktif.labels(drone_id=DRONE_ID).set(0 if skor is None else 1)
    anomali_skor.labels(drone_id=DRONE_ID).set(float('nan') if skor is None else skor)
    anomali_alarm.labels(drone_id=DRONE_ID).set(1 if alarm else 0)


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
    son_yazdirma = time.monotonic()

    try:
        dedektor = AnomaliDedektoru(MahalanobisModeli.yukle(MODEL_YOLU))
        print("Anomali modeli yüklendi:", os.path.normpath(MODEL_YOLU))
    except (OSError, ValueError, KeyError) as hata:
        dedektor = None
        print(f"UYARI: anomali modeli yüklenemedi ({hata}); sadece jamming kuralı çalışacak.")
    sonraki_karar = time.monotonic()
    onceki_alarm = False
    model_ozet = 'model: henüz karar yok'

    while True:
        mesaj = baglanti.recv_match(blocking=True, timeout=5)

        if mesaj is None:
            # 5 saniye içinde hiç veri gelmediyse, bağlantı kopmuş olabilir
            baglanti_durumu.labels(drone_id=DRONE_ID).set(0)
            anomali_yaz(None, False)  # veri yokken model de karar veremez
            continue

        baglanti_durumu.labels(drone_id=DRONE_ID).set(1)
        tip = mesaj.get_type()

        if tip == 'BATTERY_STATUS':
            if mesaj.voltages[0] != 65535:  # sentinel değeri hatırlıyoruz
                batarya_voltaji.labels(drone_id=DRONE_ID).set(mesaj.voltages[0] / 1000.0)
                son['voltaj'] = round(mesaj.voltages[0] / 1000.0, 3)
            batarya_yuzdesi.labels(drone_id=DRONE_ID).set(mesaj.battery_remaining)
            # akım: santiamper (cA) gelir, -1 = bilinmiyor
            son['akim'] = None if mesaj.current_battery == -1 else round(mesaj.current_battery / 100.0, 2)

        elif tip == 'GLOBAL_POSITION_INT':
            irtifa.labels(drone_id=DRONE_ID).set(mesaj.relative_alt / 1000.0)
            son['irtifa'] = round(mesaj.relative_alt / 1000.0, 2)

        elif tip == 'VFR_HUD':
            yer_hizi.labels(drone_id=DRONE_ID).set(mesaj.groundspeed)

        elif tip == 'HEARTBEAT':
            # Yer istasyonunun (GCS) kalp atışını sayma, sadece drone'unkini
            if mesaj.type != mavutil.mavlink.MAV_TYPE_GCS:
                son['armed'] = 1 if (mesaj.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) else 0

        elif tip == 'SERVO_OUTPUT_RAW':
            # 4 motorun PWM değeri (1000 = durgun, 2000 = tam güç)
            son['m1'] = mesaj.servo1_raw
            son['m2'] = mesaj.servo2_raw
            son['m3'] = mesaj.servo3_raw
            son['m4'] = mesaj.servo4_raw

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

        an = time.monotonic()

        # Model kararı: saniyede bir, defterin son haline bakarak
        if dedektor is not None and an >= sonraki_karar:
            sonraki_karar = an + KARAR_PERIYODU_SN
            sebep, mesafe, esik, alarm = dedektor.karar(an, dict(son, mod=baglanti.flightmode))
            if sebep is None:
                anomali_yaz(mesafe / esik, alarm)
                model_ozet = f"model {mesafe:.2f}/{esik:.2f}" + (" ALARM" if alarm else "")
            else:
                anomali_yaz(None, False)
                alarm = False
                model_ozet = f"model: karar yok ({sebep})"
            if alarm and not onceki_alarm:
                print(f"{datetime.now():%H:%M:%S} ANOMALİ ALARMI BAŞLADI | {model_ozet} | motor {son['m1']} {son['m2']} {son['m3']} {son['m4']} | V {son['voltaj']} | A {son['akim']}", flush=True)
            elif onceki_alarm and not alarm:
                print(f"{datetime.now():%H:%M:%S} Anomali alarmı bitti | {model_ozet}", flush=True)
            onceki_alarm = alarm

        # Kontrol: her 10 saniyede bir defterin içini yaz (flush: log dosyasına hemen düşsün)
        if an - son_yazdirma >= 10:
            son_yazdirma = an
            print(f"mod {baglanti.flightmode} | armed {son['armed']} | irtifa {son['irtifa']} | "
                  f"motor {son['m1']} {son['m2']} {son['m3']} {son['m4']} | "
                  f"V {son['voltaj']} | A {son['akim']} | {model_ozet}", flush=True)

if __name__ == '__main__':
    main()