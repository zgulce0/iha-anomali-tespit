from pymavlink import mavutil   # drone ile MAVLink dilinde konuşmak için hazır kütüphane
import time                     # zaman ölçmek için
import sys                      # terminalde yazılan kelimelere (sys.argv) ulaşmak için
import os                       # klasör ve dosya işleri için
from datetime import datetime   # şu anki tarih ve saat için

# ---------------------------------------------------------------------------
# KISIM 1: AYARLAR (terminalden gelir)
# Kullanım: python kayit_ucus.py <etiket> <sure_sn>
# Örnek:    python kayit_ucus.py hover 600
# ---------------------------------------------------------------------------
ETIKET = sys.argv[1] if len(sys.argv) > 1 else 'normal'  # 1. kelime = kaydın adı; yazılmadıysa 'normal'
SURE = int(sys.argv[2]) if len(sys.argv) > 2 else 600    # 2. kelime = süre (sn); yazılmadıysa 600. int() yazıyı sayıya çevirir
PERIYOT = 1.0                                            # saniyede 1 satır yazılacak (model de saniyede bir veriyle öğrendi)

KLASOR = os.path.expanduser('~/iha-anomali-tespit/veri')  # '~' yerine ev klasörünün tam yolunu koyar
os.makedirs(KLASOR, exist_ok=True)                        # klasör yoksa oluşturur, varsa hata vermez
# Dosyanın tam yolu: .../veri/ucus_<etiket>_<yılaygün_saatdakikasaniye>.csv
# Zaman dosya adında olduğu için her kayıt ayrı dosya olur, eskinin üstüne yazılmaz
DOSYA_ADI = os.path.join(KLASOR, f'ucus_{ETIKET}_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv')

# CSV dosyasının ilk satırı: 18 sütunun adı (virgülle ayrılmış)
BASLIK = 'zaman_sn,etiket,armed,voltaj_v,akim_a,kalan_yuzde,irtifa_m,hiz_ms,gaz_yuzde,tirmanma_ms,motor1_pwm,motor2_pwm,motor3_pwm,motor4_pwm,uydu,enlem,boylam,mod'

# ---------------------------------------------------------------------------
# KISIM 2: KUTULAR
# Her değerin SON bilinen hâli burada durur. Drone bilgiyi parça parça gönderir,
# her mesaj sadece kendi kutularını günceller. None = bu değer henüz hiç gelmedi.
# ---------------------------------------------------------------------------
son = {
    'armed': None, 'voltaj': None, 'akim': None, 'kalan': None,  # armed: motorlar çalışır durumda mı (1/0) | kalan: batarya yüzdesi
    'irtifa': None, 'hiz': None, 'gaz': None, 'tirmanma': None,  # tirmanma: dikey hız (m/s)
    'm1': None, 'm2': None, 'm3': None, 'm4': None,              # 4 motorun PWM değeri
    'uydu': None, 'enlem': None, 'boylam': None,                 # GPS: uydu sayısı ve konum
}


# ---------------------------------------------------------------------------
# KISIM 3: guncelle() - gelen mesaja bakıp ilgili kutuları doldurur
# ---------------------------------------------------------------------------
def guncelle(m):                   # m = drone'dan gelen mesaj
    tip = m.get_type()             # mesajın türü, örn. 'BATTERY_STATUS'
    if tip == 'HEARTBEAT':         # kalp atışı mesajı
        if m.type != mavutil.mavlink.MAV_TYPE_GCS:  # yer istasyonunun kendi kalp atışını sayma, sadece drone'unkini
            # base_mode bir bayraklar sayısıdır; & ile "ARMED biti açık mı" diye bakılır. Açıksa 1, değilse 0
            son['armed'] = 1 if (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) else 0
    elif tip == 'BATTERY_STATUS':  # batarya mesajı
        v = m.voltages[0]          # voltaj milivolt olarak gelir
        son['voltaj'] = None if v == 65535 else round(v / 1000.0, 3)  # 65535 = "bilinmiyor" -> None; değilse volta çevir (/1000), 3 basamağa yuvarla
        son['akim'] = None if m.current_battery == -1 else round(m.current_battery / 100.0, 2)  # akım santiamper gelir (/100 -> amper); -1 = bilinmiyor
        son['kalan'] = None if m.battery_remaining == -1 else m.battery_remaining  # kalan batarya yüzdesi; -1 = bilinmiyor
    elif tip == 'GLOBAL_POSITION_INT':  # konum mesajı
        son['irtifa'] = round(m.relative_alt / 1000.0, 2)  # yerden irtifa milimetre gelir (/1000 -> metre)
        son['enlem'] = m.lat / 1e7     # enlem ve boylam 10 milyonla çarpılmış tam sayı gelir; 1e7'ye bölüp dereceye çeviriyoruz
        son['boylam'] = m.lon / 1e7
    elif tip == 'VFR_HUD':         # uçuş göstergeleri mesajı
        son['hiz'] = round(m.groundspeed, 2)  # yer hızı (m/s)
        son['gaz'] = m.throttle    # gaz yüzdesi
        son['tirmanma'] = round(m.climb, 2)   # dikey hız (m/s)       
    elif tip == 'SERVO_OUTPUT_RAW':  # motorlara giden komut mesajı
        son['m1'] = m.servo1_raw   # PWM değeri: 1000 = durgun, 2000 = tam güç
        son['m2'] = m.servo2_raw
        son['m3'] = m.servo3_raw
        son['m4'] = m.servo4_raw
    elif tip == 'GPS_RAW_INT':     # GPS mesajı
        son['uydu'] = None if m.satellites_visible == 255 else m.satellites_visible  # 255 = bilinmiyor


# ---------------------------------------------------------------------------
# KISIM 4: BAĞLANTI VE DOSYA
# ---------------------------------------------------------------------------
baglanti = mavutil.mavlink_connection('udpin:127.0.0.1:14552')  # bu bilgisayarın 14552 numaralı UDP kapısını dinle (SITL mesajları buraya da yollar; exporter 14551'i dinler, iki program aynı kapıyı dinleyemez)
dosya = open(DOSYA_ADI, 'w')       # CSV dosyasını yazmak için aç ('w' = write)
dosya.write(BASLIK + '\n')         # ilk satıra sütun adlarını yaz ('\n' = satır sonu)
print('Kayit basliyor:', DOSYA_ADI)
print(f'Etiket: {ETIKET} | Sure: {SURE} sn | Periyot: {PERIYOT} sn')

# ---------------------------------------------------------------------------
# KISIM 5: SAYAÇLAR
# time.monotonic() saniye cinsinden, geri sarmayan bir sayaçtır
# ---------------------------------------------------------------------------
baslangic = time.monotonic()       # kaydın başladığı an
sonraki_yazim = baslangic          # bir sonraki CSV satırının yazılacağı an (ilk satır hemen yazılır)
son_yazdirma = baslangic           # ekrana en son özet yazdığımız an
son_paket = baslangic              # drone'dan en son mesajın geldiği an
satir_sayisi = 0                   # dosyaya yazılan satır sayısı
voltajlar = []                     # sonda en düşük/en yüksek voltajı bulmak için biriktirilir
irtifalar = []                     # sonda en yüksek irtifayı bulmak için biriktirilir

# ---------------------------------------------------------------------------
# KISIM 6: ANA DÖNGÜ - mesaj al, kutuları güncelle, saniyede bir satır yaz
# ---------------------------------------------------------------------------
try:                               # Ctrl+C ile durdurulursa program hata verip çökmesin diye
    while time.monotonic() - baslangic < SURE:  # SURE saniye dolana kadar tekrarla
        m = baglanti.recv_match(blocking=True, timeout=0.2)  # bir mesaj bekle (en fazla 0,2 sn); gelmezse None olur
        simdi = time.monotonic()   # şu anki an
        if m is not None:          # mesaj geldiyse
            son_paket = simdi      # "en son mesaj şimdi geldi" diye not et
            guncelle(m)            # mesajın getirdiği kutuları güncelle
        if simdi >= sonraki_yazim:  # satır yazma zamanı geldi mi? (saniyede bir)
            sonraki_yazim += PERIYOT  # bir sonraki satırın zamanını 1 sn ileri al
            if simdi - son_paket > 3:  # 3 sn'den uzun süredir mesaj yoksa veri bayattır
                print('3 sn den uzun suredir paket gelmiyor, SITL acik mi?')
                continue           # bu saniyeye satır yazma, döngünün başına dön
            t = simdi - baslangic  # kayıt başladığından beri geçen süre (sn)
            alanlar = [            # bir CSV satırının değerleri; sıra BASLIK'taki sütunlarla aynı olmalı
                f'{t:.1f}', ETIKET, son['armed'], son['voltaj'], son['akim'], son['kalan'],  # zaman_sn (1 ondalık basamak), etiket, armed, voltaj, akım, kalan yüzde
                son['irtifa'], son['hiz'], son['gaz'], son['tirmanma'],                      # irtifa, hız, gaz, tırmanma
                son['m1'], son['m2'], son['m3'], son['m4'],                                  # 4 motor PWM
                son['uydu'], son['enlem'], son['boylam'], baglanti.flightmode,               # uydu, konum, uçuş modu (örn. GUIDED)
            ]
            # Her değeri yazıya çevir (None ise boş bırak), virgülle birleştir, satır sonu ekle, dosyaya yaz
            dosya.write(','.join('' if a is None else str(a) for a in alanlar) + '\n')
            dosya.flush()          # satırı hemen diske yazdır (program çökse bile veri kaybolmasın)
            satir_sayisi += 1      # satır sayacını 1 artır
            if son['voltaj'] is not None:
                voltajlar.append(son['voltaj'])  # sonda özet için biriktir
            if son['irtifa'] is not None:
                irtifalar.append(son['irtifa'])  # sonda özet için biriktir
            if simdi - son_yazdirma >= 10:  # son ekran özetinden 1                  0 sn geçtiyse
                son_yazdirma = simdi
                print(f"{t:.0f} sn | armed {son['armed']} | V {son['voltaj']} | A {son['akim']} | irtifa {son['irtifa']} | hiz {son['hiz']} | gaz {son['gaz']} | motor {son['m1']} {son['m2']} {son['m3']} {son['m4']} | uydu {son['uydu']} | mod {baglanti.flightmode}")  # ekrana tek satır özet
except KeyboardInterrupt:          # Ctrl+C'ye basıldı
    print('Kullanici durdurdu')
finally:                           # ne olursa olsun (hata da çıksa) burası çalışır
    dosya.close()                  # dosyayı kapat

# ---------------------------------------------------------------------------
# KISIM 7: KAPANIŞ ÖZETİ - kaydın sağlıklı olup olmadığına hızlıca bakmak için
# ---------------------------------------------------------------------------
print(f'Kaydedilen satir: {satir_sayisi}')  # kaç satır yazıldı
print(f'Dosya: {DOSYA_ADI}')                # dosyanın yolu
if voltajlar:                               # liste boş değilse (en az bir voltaj okunduysa)
    print(f'Voltaj en dusuk / en yuksek: {min(voltajlar)} / {max(voltajlar)} V')  # min() en küçüğü, max() en büyüğü verir
if irtifalar:
    print(f'En yuksek irtifa: {max(irtifalar)} m')