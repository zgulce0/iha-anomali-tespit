from pymavlink import mavutil
from prometheus_client import start_http_server, Gauge
import os
import time
from collections import deque
from datetime import datetime

from anomali_model import MahalanobisModeli, AnomaliDedektoru
from aciklama import Aciklayici
from ekran import EkranDurumu, sunucu_baslat
from spoof_tespit import SpoofIzleyici, spoofing_aciklama
from harita import HaritaDurumu

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

# GPS spoofing (kendi Kalman filtremiz, exporter/spoof_tespit.py)
spoof_alarm = Gauge('drone_gps_spoofing_alarm', 'GPS spoofing alarmı (1=spoofing şüphesi)', ['drone_id'])
spoof_aktif = Gauge('drone_gps_spoofing_active', 'Spoofing izlemesi çalışıyor mu (1=evet, 0=kapı kapalı)', ['drone_id'])
spoof_nis = Gauge('drone_gps_spoof_nis', 'GPS konum yeniliği NIS, son 5 sn en yüksek (chi-kare, sınır 9.21)', ['drone_id'])
spoof_nis_v = Gauge('drone_gps_spoof_nis_vel', 'GPS hız yeniliği NIS, son 5 sn en yüksek (chi-kare, sınır 13.82, art arda 2 ölçüm)', ['drone_id'])
spoof_kayma = Gauge('drone_gps_spoof_drift_m', 'Kestirilen GPS sapması, son 5 sn en yüksek (m, sınır ~7.3)', ['drone_id'])
spoof_nis_z = Gauge('drone_gps_spoof_nis_alt', 'GPS yükseklik yeniliği NIS, son 5 sn en yüksek (chi-kare 1 sd, sınır 6.63)', ['drone_id'])
spoof_fark_z = Gauge('drone_gps_spoof_alt_diff_m', 'Kestirilen GPS-barometre yükseklik farkı (mutlak), son 5 sn en yüksek (m, sınır ~11.7)', ['drone_id'])

# Anomali modeli (Mahalanobis): normal uçuştan ne kadar uzak?
anomali_skor = Gauge('drone_anomaly_score', 'Modelin skoru: mesafe / alarm eşiği (1.0 = sınır, üstü anomali)', ['drone_id'])
anomali_alarm = Gauge('drone_anomaly_alarm', 'Donanım anomali alarmı (1=alarm)', ['drone_id'])
anomali_aktif = Gauge('drone_anomaly_active', 'Model şu an karar veriyor mu (1=evet, 0=kapı kapalı)', ['drone_id'])
# Alarm sırasında arızanın türü. 5 seri her zaman var; alarmdayken sadece biri 1, diğerleri 0.
anomali_tur = Gauge('drone_anomaly_type', 'Arıza türü (1=alarm şu an bu türde, 0=değil)', ['drone_id', 'tur'])
TURLER = ['batarya', 'motor1', 'motor2', 'motor3', 'motor4']

DRONE_ID = 'sitl_1'

MODEL_YOLU = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'model', 'mahalanobis_v1.json')
# Eğitim verisi saniyede bir kaydedilmişti; model canlıda da saniyede bir bakar.
KARAR_PERIYODU_SN = 1.0

# "Defter": her değerin SON bilinen hali. Mesajlar farklı zamanlarda geliyor
# (motorlar bir mesajda, batarya başka mesajda), her mesaj sadece kendi satırını
# günceller. Model karar verirken hepsine birlikte bakacak.
# None = henüz hiç gelmedi.
son = {'armed': None, 'irtifa': None, 'voltaj': None, 'akim': None,
       'm1': None, 'm2': None, 'm3': None, 'm4': None,
       'uydu': None, 'fix': None,   # uydu/fix: sadece operatör ekranı için
       'batarya': None, 'hiz': None}   # kalan batarya % ve yer hızı: sadece operatör ekranı için

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

# Spoofing alarmı açıldıktan sonra SPOOF_TUTMA_SN saniye açık kalır. Ani sıçramada saldırı sonrası
# sabit kalan kayma artık görünmez (filtre yeni konuma oturur); alarmın hemen kapanmaması için uzun tutulur.
SPOOF_TUTMA_SN = 30
spoof = SpoofIzleyici(kilit_sn=SPOOF_TUTMA_SN)
spoof_durum = {'alarm': False, 'sonuc': None}


# Prometheus değerleri birkaç saniyede bir okur; sıçrama anındaki yüksek NIS ise yarım saniye sürer.
# Anlık değer yazılırsa Grafana bu tepeleri kaçırır. Bu yüzden son TEPE_SN saniyenin en yükseği yazılır.
TEPE_SN = 5.0
spoof_gecmis = deque()      # (saat, nis, nis_v, sapma, nis_z, |fark_z|)


def spoof_yaz(sonuc, alarm):
    """sonuc None ise izleme kapalı (yerde, GPS sağlıksız ya da jamming var)."""
    spoof_aktif.labels(drone_id=DRONE_ID).set(0 if sonuc is None else 1)
    spoof_alarm.labels(drone_id=DRONE_ID).set(1 if alarm else 0)
    nan = float('nan')
    if sonuc is None:
        spoof_gecmis.clear()
        for g in (spoof_nis, spoof_nis_v, spoof_kayma, spoof_nis_z, spoof_fark_z):
            g.labels(drone_id=DRONE_ID).set(nan)
        return
    an = time.monotonic()
    spoof_gecmis.append((an, sonuc['nis'], sonuc['nis_v'], sonuc['sapma'], sonuc['nis_z'], abs(sonuc['fark_z'])))
    while spoof_gecmis and spoof_gecmis[0][0] < an - TEPE_SN:
        spoof_gecmis.popleft()
    spoof_nis.labels(drone_id=DRONE_ID).set(max(x[1] for x in spoof_gecmis))
    spoof_nis_v.labels(drone_id=DRONE_ID).set(max(x[2] for x in spoof_gecmis))
    spoof_kayma.labels(drone_id=DRONE_ID).set(max(x[3] for x in spoof_gecmis))
    spoof_nis_z.labels(drone_id=DRONE_ID).set(max(x[4] for x in spoof_gecmis))
    spoof_fark_z.labels(drone_id=DRONE_ID).set(max(x[5] for x in spoof_gecmis))


def anomali_yaz(skor, alarm, tur=None):
    """skor None ise model karar vermiyor (kapı kapalı): alarm yok, skor boş.
    tur: alarm varken arıza türü ('batarya', 'motor1'..'motor4'); alarm yoksa None."""
    anomali_aktif.labels(drone_id=DRONE_ID).set(0 if skor is None else 1)
    anomali_skor.labels(drone_id=DRONE_ID).set(float('nan') if skor is None else skor)
    anomali_alarm.labels(drone_id=DRONE_ID).set(1 if alarm else 0)
    for t in TURLER:
        anomali_tur.labels(drone_id=DRONE_ID, tur=t).set(1 if (alarm and t == tur) else 0)


def spoof_ozet():
    s_ = spoof_durum['sonuc']
    if not spoof.aktif or s_ is None:
        return 'spoof: izleme kapalı'
    return (f"spoof NIS {s_['nis']:.1f} hızNIS {s_['nis_v']:.1f} sapma {s_['sapma']:.1f} m "
            f"yükseklik NIS {s_['nis_z']:.1f} fark {s_['fark_z']:.1f} m"
            + (" ALARM" if spoof_durum['alarm'] else ""))


# Spoofing dedektörü ivmeölçeri (RAW_IMU) ve tutum açılarını (ATTITUDE) sık ister: iki GPS ölçümü
# arasındaki ivmenin ortalaması alınır. Seyrek gelirse ani bir sarsıntı yanlış ölçülür.
# Not: MAVProxy'de "set streamrate -1" yazılmazsa MAVProxy bu hızları kendi değeriyle ezer.
ISTENEN_HIZLAR = {27: 50, 30: 50, 24: 10,     # RAW_IMU, ATTITUDE: 50 Hz; GPS_RAW_INT: 10 Hz
                  29: 10,                     # SCALED_PRESSURE (barometre): yükseklik spoofing testi için
                  # Anomali modelinin girdileri: motor PWM, batarya, yer hızı. MAVProxy'de "set streamrate -1"
                  # iken (baslat.sh bunu ayarlıyor) bunlar istenmezse hiç gelmez ve model karar veremez.
                  36: 4, 147: 4, 74: 4,       # SERVO_OUTPUT_RAW, BATTERY_STATUS, VFR_HUD
                  33: 10, 164: 5}             # harita için: GLOBAL_POSITION_INT 10 Hz, SIMSTATE (gerçek konum, sadece SITL) 5 Hz


def mesaj_hizlarini_iste(baglanti):
    for msg_id, hz in ISTENEN_HIZLAR.items():
        baglanti.mav.command_long_send(baglanti.target_system, baglanti.target_component,
                                       mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                       msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)


def main():
    # Prometheus'un okuyacağı /metrics endpoint'ini başlat (port 8000)
    start_http_server(8000)
    print("Exporter başladı: http://localhost:8000/metrics")

    print("SITL'e bağlanılıyor...")
    baglanti = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
    baglanti.wait_heartbeat()
    print(f"Bağlantı kuruldu! Sistem ID: {baglanti.target_system}")
    mesaj_hizlarini_iste(baglanti)
    son_istek = time.monotonic()
    seyrek_sayac = 0
    baglanti_durumu.labels(drone_id=DRONE_ID).set(1)

    print("Veri okunuyor ve Prometheus metriklerine yazılıyor... (Ctrl+C ile durdur)")
    son_yazdirma = time.monotonic()

    try:
        dedektor = AnomaliDedektoru(MahalanobisModeli.yukle(MODEL_YOLU))
        print("Anomali modeli yüklendi:", os.path.normpath(MODEL_YOLU))
    except (OSError, ValueError, KeyError) as hata:
        dedektor = None
        print(f"UYARI: anomali modeli yüklenemedi ({hata}); sadece jamming kuralı çalışacak.")
    # Operatör ekranı (salt okunur web sayfası): http://localhost:8088/
    ekran = None
    harita = HaritaDurumu()
    if dedektor is not None:
        ekran = EkranDurumu(Aciklayici(dedektor.model), DRONE_ID, os.environ.get('GRAFANA_PANEL_URL', ''))
        _, ekran_port = sunucu_baslat(ekran, harita=harita)
        print(f"Operatör ekranı: http://localhost:{ekran_port}/   3B harita: http://localhost:{ekran_port}/harita")
    sonraki_karar = time.monotonic()
    onceki_alarm = False
    model_ozet = 'model: henüz karar yok'

    while True:
        mesaj = baglanti.recv_match(blocking=True, timeout=5)

        if mesaj is None:
            # 5 saniye içinde hiç veri gelmediyse, bağlantı kopmuş olabilir
            baglanti_durumu.labels(drone_id=DRONE_ID).set(0)
            anomali_yaz(None, False)  # veri yokken model de karar veremez
            if ekran is not None:
                ekran.baglanti_yok(datetime.now())
            continue

        baglanti_durumu.labels(drone_id=DRONE_ID).set(1)
        tip = mesaj.get_type()
        if tip in ('SIMSTATE', 'GPS_RAW_INT', 'GLOBAL_POSITION_INT', 'ATTITUDE'):
            harita.mesaj(mesaj)

        if tip == 'BATTERY_STATUS':
            if mesaj.voltages[0] != 65535:  # sentinel değeri hatırlıyoruz
                batarya_voltaji.labels(drone_id=DRONE_ID).set(mesaj.voltages[0] / 1000.0)
                son['voltaj'] = round(mesaj.voltages[0] / 1000.0, 3)
            batarya_yuzdesi.labels(drone_id=DRONE_ID).set(mesaj.battery_remaining)
            son['batarya'] = None if mesaj.battery_remaining == -1 else mesaj.battery_remaining
            # akım: santiamper (cA) gelir, -1 = bilinmiyor
            son['akim'] = None if mesaj.current_battery == -1 else round(mesaj.current_battery / 100.0, 2)

        elif tip == 'GLOBAL_POSITION_INT':
            irtifa.labels(drone_id=DRONE_ID).set(mesaj.relative_alt / 1000.0)
            son['irtifa'] = round(mesaj.relative_alt / 1000.0, 2)

        elif tip == 'VFR_HUD':
            yer_hizi.labels(drone_id=DRONE_ID).set(mesaj.groundspeed)
            son['hiz'] = round(mesaj.groundspeed, 2)

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
            son['uydu'] = 0 if uydu == 255 else uydu
            son['fix'] = mesaj.fix_type
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

            # Spoofing: her yeni GPS ölçümünde kendi filtremiz bir adım ilerler.
            # Zaman olarak GPS'in kendi zaman damgası kullanılır (ağ gecikmesinden etkilenmesin).
            t_gps = mesaj.time_usec / 1e6 if mesaj.time_usec else simdi
            kapi = (son['armed'] == 1 and son['irtifa'] is not None
                    and son['irtifa'] >= SpoofIzleyici.MIN_IRTIFA_M and saglikli and not jamming_aktif)
            # GPS_RAW_INT ve RAW_IMU zaman damgalari ayni saatten (aracin acilistan beri gecen sure); ivmeyi
            # GPS ile eslemek icin bu kullanilir.
            sonuc = spoof.gps(t_gps, mesaj.lat, mesaj.lon, mesaj.vel, mesaj.cog, kapi,
                              saat=t_gps if mesaj.time_usec else None, alt_mm=mesaj.alt)
            if sonuc is not None:
                seyrek_sayac = seyrek_sayac + 1 if spoof.imu_sayisi < spoof.MIN_IMU else 0
                if seyrek_sayac == 25:      # ~5 sn boyunca seyrek
                    print(f"{datetime.now():%H:%M:%S} UYARI: ivmeölçer verisi seyrek geliyor "
                          f"(GPS ölçümü başına {spoof.imu_sayisi}). MAVProxy'de 'set streamrate -1' yazıldı mı? "
                          "Spoofing hız testi bu sürede daha az hassas.", flush=True)
            if sonuc is not None:
                spoof_durum['sonuc'] = sonuc
            harita.filtre_konum(spoof.tahmin_konum())
            spoof_alarm_simdi = spoof.alarm(t_gps)
            spoof_yaz(sonuc, spoof_alarm_simdi)
            if spoof_alarm_simdi and not spoof_durum['alarm']:
                s_ = spoof_durum['sonuc'] or {}
                print(f"{datetime.now():%H:%M:%S} GPS SPOOFING ALARMI BAŞLADI | test {s_.get('tetik')} | "
                      f"NIS {s_.get('nis', 0):.1f} | hız NIS {s_.get('nis_v', 0):.1f} | GPS sapması {s_.get('sapma', 0):.1f} m | "
                      f"yükseklik NIS {s_.get('nis_z', 0):.1f} | GPS-baro farkı {s_.get('fark_z', 0):.1f} m", flush=True)
            elif spoof_durum['alarm'] and not spoof_alarm_simdi:
                print(f"{datetime.now():%H:%M:%S} GPS spoofing alarmı bitti", flush=True)
            spoof_durum['alarm'] = spoof_alarm_simdi

        elif tip == 'ATTITUDE':
            spoof.attitude(mesaj.roll, mesaj.pitch, mesaj.yaw)

        elif tip == 'SCALED_PRESSURE':
            spoof.baro(mesaj.press_abs, saat=mesaj.time_boot_ms / 1000 if mesaj.time_boot_ms else None)

        elif tip == 'RAW_IMU':
            spoof.raw_imu(mesaj.xacc, mesaj.yacc, mesaj.zacc,
                          saat=mesaj.time_usec / 1e6 if mesaj.time_usec else None)

        an = time.monotonic()
        if an - son_istek >= 30:            # simülatör yeniden başlarsa hızlar sıfırlanır: ara ara tekrar iste
            mesaj_hizlarini_iste(baglanti)
            son_istek = an

        # Model kararı: saniyede bir, defterin son haline bakarak
        if dedektor is not None and an >= sonraki_karar:
            sonraki_karar = an + KARAR_PERIYODU_SN
            durum = dict(son, mod=baglanti.flightmode)
            sebep, mesafe, esik, alarm = dedektor.karar(an, durum)
            if sebep is None:
                tur = dedektor.tur(durum) if alarm else None
                anomali_yaz(mesafe / esik, alarm, tur)
                model_ozet = f"model {mesafe:.2f}/{esik:.2f}" + (f" ALARM ({tur})" if alarm else "")
            else:
                anomali_yaz(None, False)
                alarm = False
                model_ozet = f"model: karar yok ({sebep})"
            if alarm and not onceki_alarm:
                print(f"{datetime.now():%H:%M:%S} ANOMALİ ALARMI BAŞLADI | {model_ozet} | motor {son['m1']} {son['m2']} {son['m3']} {son['m4']} | V {son['voltaj']} | A {son['akim']}", flush=True)
            elif onceki_alarm and not alarm:
                print(f"{datetime.now():%H:%M:%S} Anomali alarmı bitti | {model_ozet}", flush=True)
            onceki_alarm = alarm

            if ekran is not None:
                jam_simdi = jam_durum['son'] is not None and (an - jam_durum['son']) < JAM_TUTMA_SN
                spoof_bilgi = spoofing_aciklama(spoof_durum['sonuc']) if spoof_durum['alarm'] else None
                ekran.guncelle(datetime.now(), son, baglanti.flightmode, sebep,
                               (mesafe / esik) if sebep is None else None, alarm, jam_simdi,
                               son['uydu'], son['fix'], tur if sebep is None else None, spoof_bilgi,
                               konum=harita.konum_ozeti())
                ed = ekran.durum
                harita.kare(datetime.now(), son['armed'] == 1, spoof_durum['alarm'], jam_simdi,
                            ed.get('durum') == 'alarm', tur if (alarm and sebep is None) else None,
                            ed.get('baslik') if ed.get('durum') == 'alarm' else None,
                            son['voltaj'], son.get('batarya'), baglanti.flightmode)

        # Kontrol: her 10 saniyede bir defterin içini yaz (flush: log dosyasına hemen düşsün)
        if an - son_yazdirma >= 10:
            son_yazdirma = an
            print(f"mod {baglanti.flightmode} | armed {son['armed']} | irtifa {son['irtifa']} | "
                  f"motor {son['m1']} {son['m2']} {son['m3']} {son['m4']} | "
                  f"V {son['voltaj']} | A {son['akim']} | {model_ozet} | {spoof_ozet()}", flush=True)

if __name__ == '__main__':
    main()