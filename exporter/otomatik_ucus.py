# ===========================================================================
# BU DOSYA NE YAPAR?
# SITL'de drone'u kendi kendine uçurur: arm eder, kalkar, 10 m'de önce normal uçar,
# sonra 23 arızayı sırayla uygular (her biri 30 sn açık kalır, sonra geri alınır).
# Bütün süre boyunca saniyede bir satırı CSV dosyasına kaydeder. Her satırdaki
# "etiket" sütunu o an uçuşun normal mi, yoksa hangi arızanın açık olduğunu söyler.
# Çıktı iki dosyadır:
#   ucus_<ad>_<tarih>.csv          -> saniyede bir satır veri
#   ucus_<ad>_<tarih>_olaylar.csv  -> "ARIZA BASLIYOR" gibi olayların saatleri
# Aşağıdaki kullanım yazısı (tırnak içinde) kodun parçasıdır, programa argüman
# verilmezse ekrana basılır. Dokunma.
# ===========================================================================
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
import math                      # radyanı dereceye çevirmek için (math.degrees)
import os                        # klasör ve dosya yolu işleri
import random                    # arıza sırasını karıştırmak için
import sys                       # terminalden gelen kelimeler (sys.argv) ve çıkış kodu
import time                      # zaman ölçmek için
from datetime import datetime    # tarih ve saat için

from pymavlink import mavutil    # drone ile MAVLink dilinde konuşmak için hazır kütüphane

# ---------------------------------------------------------------------------
# KISIM 1: AYARLAR
# ---------------------------------------------------------------------------
BAGLANTI = 'udpin:127.0.0.1:14553'  # bu bilgisayarın 14553 numaralı UDP kapısını dinle (exporter 14551, kayıt scripti 14552, bu script 14553 kullanır)
KLASOR = os.path.expanduser('~/iha-anomali-tespit/veri')  # CSV dosyalarının yazılacağı klasör ('~' ev klasörü demek)

IRTIFA = 10            # hedef irtifa (metre)
BASLANGIC_SN = 40      # arızasız normal uçuş süresi (sn)
ARIZA_SN = 30          # her arızanın açık kaldığı süre (sn)
TOPARLANMA_SN = 30     # arıza kapandıktan sonra normal bekleme süresi (sn)
PERIYOT = 1.0          # kayıt periyodu: saniyede 1 satır
DUSME_ESIGI = 5.0      # havadayken irtifa bunun altına inerse uçuş bozuldu sayılır (m)
KARARLI_SN = 10        # hedef irtifaya çıkınca 'normal' etiketi başlamadan önce kararlılaşma süresi (sn)
BATARYA_TOL_V = 0.05   # voltaj, "12.6 - 0.01 x akım" formülünden en fazla bu kadar (V) sapabilir; fazlası "batarya normal değil" demek

# CSV'nin ilk satırı: 20 sütun adı (kayit_ucus.py'deki 18 sütun + roll_der ve pitch_der)
BASLIK = ('zaman_sn,etiket,armed,voltaj_v,akim_a,kalan_yuzde,irtifa_m,hiz_ms,gaz_yuzde,'
          'tirmanma_ms,motor1_pwm,motor2_pwm,motor3_pwm,motor4_pwm,uydu,enlem,boylam,'
          'mod,roll_der,pitch_der')

# ArduCopter'da uçuş modları numarayla seçilir: modun adı -> numarası
MODLAR = {'GUIDED': 4, 'LOITER': 5, 'CIRCLE': 7, 'LAND': 9}

# ---------------------------------------------------------------------------
# KISIM 2: ARIZA LİSTESİ
# Her arıza üç parçalı bir grup (tuple):
#   (etiket, uygulanacak [(ayar, değer)] SIRAYLA, geri alınacak [(ayar, değer)] SIRAYLA)
# Ayar = SITL'in simülasyon ayarı (SIM_ ile başlayanlar). Değer değişince SITL'in davranışı değişir.
# Motor arızasında MUL önce ayarlanır, FAIL en son açılır (MUL 0.0 iken FAIL açılırsa
# motor tamamen durur). Geri alırken önce FAIL kapatılır.
# İç direnç geri alınırken -1 YAZILMAZ: 16 Ekim denemesinde -1 direnci varsayılana
# döndürmedi, 0.05'te kaldı. Normal direnç 0.01 (V = 12.6 - 0.01*I ölçüldü).
# ---------------------------------------------------------------------------
ARIZALAR = []                                   # boş liste; aşağıdaki 3 döngü 23 arızayı buraya ekler
for _v in (12.5, 12.2, 11.8, 11.5):             # 4 voltaj değeri için tekrarla; _v her turda sıradaki değer olur
    # etiket 'batt_volt_12.5' gibi olur | uygula: voltajı _v yap | geri al: voltajı normal 12.6'ya döndür
    ARIZALAR.append((f'batt_volt_{_v}', [('SIM_BATT_VOLTAGE', _v)], [('SIM_BATT_VOLTAGE', 12.6)]))
for _r in (0.02, 0.03, 0.05):                   # 3 iç direnç değeri için tekrarla
    # uygula: bataryanın iç direncini _r yap | geri al: normal 0.01'e döndür
    ARIZALAR.append((f'batt_res_{_r}', [('SIM_BATT_RES_OHM', _r)], [('SIM_BATT_RES_OHM', 0.01)]))
for _m in (1, 2, 3, 4):                         # her motor için
    for _mul in (0.95, 0.9, 0.85, 0.7):         # her güç çarpanı için (0.85 = motorun gücü %85'e düşer, yani %15 kayıp)
        ARIZALAR.append((f'motor{_m}_{_mul}',   # etiket, örn. 'motor3_0.85'
                         # uygula: önce güç çarpanını ayarla, sonra hangi motor olduğunu seç (1 << (_m-1) = 1, 2, 4 ya da 8)
                         [('SIM_ENGINE_MUL', _mul), ('SIM_ENGINE_FAIL', 1 << (_m - 1))],
                         [('SIM_ENGINE_FAIL', 0)]))  # geri al: motor seçimini sıfırla, arıza biter
# Toplam: 4 voltaj + 3 direnç + 4 motor x 4 çarpan = 23 arıza

# Uçuş bitince ya da hata olunca SITL'i temiz bırakmak için uygulanacak ayarlar (FAIL, MUL'dan önce kapanır)
TEMIZLIK = [('SIM_ENGINE_FAIL', 0), ('SIM_ENGINE_MUL', 0.0),
            ('SIM_BATT_VOLTAGE', 12.6), ('SIM_BATT_RES_OHM', 0.01)]


# ---------------------------------------------------------------------------
# KISIM 3: KENDİ HATA TÜRLERİMİZ
# Programın "neyin ters gittiğini" ayırt edebilmesi için iki ayrı hata türü
# ---------------------------------------------------------------------------
class Hata(Exception):    # bir adım başarısız olunca (mod geçilemedi, ayar onaylanmadı gibi) bunu fırlatırız
    pass                  # pass = içi boş, hata türünün sadece bir adı olsun yeter


class Dusme(Exception):   # havadayken irtifa DUSME_ESIGI'nin altına inerse bunu fırlatırız
    pass


# ---------------------------------------------------------------------------
# KISIM 4: PROGRAMIN HAFIZASI (global değişkenler)
# ---------------------------------------------------------------------------
son = {}              # değerlerin son bilinen hâli (kutular); sifirla() doldurur
param_degerleri = {}  # SITL'in onayladığı ayar değerleri (ayar adı -> değer)
durum = {}            # programın durumu (şu anki etiket, havada mı, vb.); sifirla() doldurur
baglanti = None       # drone bağlantısı; main() içinde oluşturulur
dosya = None          # veri CSV'si; main() içinde açılır
olay_dosya = None     # olay CSV'si; main() içinde açılır
T0 = 0.0              # kaydın başladığı an; olayların "kaçıncı sn" bilgisi bundan hesaplanır


def sifirla():                                  # kutuları ve durumu başlangıç hâline getirir
    global son, param_degerleri, durum          # global: fonksiyon içinde dışarıdaki bu üç değişkeni DEĞİŞTİREBİLMEK için gerekir
    son = {
        'armed': None, 'voltaj': None, 'akim': None, 'kalan': None,
        'irtifa': None, 'hiz': None, 'gaz': None, 'tirmanma': None,
        'm1': None, 'm2': None, 'm3': None, 'm4': None,
        'uydu': None, 'enlem': None, 'boylam': None, 'roll': None, 'pitch': None,  # kayit_ucus.py'den farkı: roll ve pitch (yatış ve eğim açıları)
    }
    param_degerleri = {}                        # henüz hiçbir ayar onaylanmadı
    durum = {
        'etiket': 'hazirlik',   # şu anki etiket; kalkıştan önce 'hazirlik'
        'havada': False,        # drone havada mı (düşme kontrolü sadece havadayken yapılır)
        'rc_aktif': False,      # gaz sinyali gönderiliyor mu (sadece daire modunda)
        'sonraki_rc': 0.0,      # gaz sinyalinin bir sonraki gönderim anı
        'sonraki_yazim': 0.0,   # bir sonraki CSV satırının yazılacağı an
        'son_yazdirma': 0.0,    # ekrana en son özet yazdığımız an
        'son_paket': 0.0,       # drone'dan en son mesajın geldiği an
        'satir': 0,             # dosyaya yazılan satır sayısı
    }


def olay(metin):                                # bir olayı ekrana ve olay dosyasına yazar
    t = time.monotonic() - T0                   # kayıt başladığından beri geçen süre (sn)
    # Ekrana: [süre] saat:dakika:saniye.milisaniye mesaj. {t:7.1f} = 7 karakter genişlikte, 1 ondalık.
    # [:-3] mikrosaniyenin son 3 hanesini atar, milisaniye kalır.
    print(f'[{t:7.1f} sn] {datetime.now().strftime("%H:%M:%S.%f")[:-3]} {metin}')
    if olay_dosya is not None:                  # olay dosyası açıksa
        olay_dosya.write(f'{t:.1f},{metin.replace(",", ";")}\n')  # dosyaya "süre,mesaj" yaz; mesajdaki virgülleri ';' yap ki CSV sütunları bozulmasın
        olay_dosya.flush()                      # satırı hemen diske yazdır


# ---------------------------------------------------------------------------
# KISIM 5: DRONE'DAN OKUMA
# ---------------------------------------------------------------------------
def guncelle(m):                   # gelen mesaja bakıp ilgili kutuları doldurur (kayit_ucus.py'deki ile aynı, 3 ek var)
    tip = m.get_type()             # mesajın türü, örn. 'BATTERY_STATUS'
    if tip == 'HEARTBEAT':         # kalp atışı mesajı
        if m.type != mavutil.mavlink.MAV_TYPE_GCS:   # yer istasyonunun kendi kalp atışını sayma, sadece drone'unkini
            son['armed'] = 1 if (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) else 0   # base_mode'daki ARMED basamağı açıksa 1, değilse 0
            baglanti.target_system = m.get_srcSystem()        # EK 1: drone'un kimlik numarasını öğren; komut gönderirken kime gönderdiğimizi bilmek için
            baglanti.target_component = m.get_srcComponent()  # drone'un içindeki alt birimin numarası
    elif tip == 'BATTERY_STATUS':  # batarya mesajı
        v = m.voltages[0]          # voltaj milivolt olarak gelir
        son['voltaj'] = None if v == 65535 else round(v / 1000.0, 3)   # 65535 = bilinmiyor -> None; değilse volta çevir, 3 basamağa yuvarla
        son['akim'] = None if m.current_battery == -1 else round(m.current_battery / 100.0, 2)   # akım santiamper gelir (/100 -> amper); -1 = bilinmiyor
        son['kalan'] = None if m.battery_remaining == -1 else m.battery_remaining   # kalan batarya yüzdesi; -1 = bilinmiyor
    elif tip == 'GLOBAL_POSITION_INT':   # konum mesajı
        son['irtifa'] = round(m.relative_alt / 1000.0, 2)   # yerden irtifa milimetre gelir (/1000 -> metre)
        son['enlem'] = m.lat / 1e7     # enlem ve boylam 10 milyonla çarpılmış tam sayı gelir; 1e7'ye bölüp dereceye çeviriyoruz
        son['boylam'] = m.lon / 1e7
    elif tip == 'VFR_HUD':         # uçuş göstergeleri mesajı
        son['hiz'] = round(m.groundspeed, 2)    # yer hızı (m/s)
        son['gaz'] = m.throttle                 # gaz yüzdesi
        son['tirmanma'] = round(m.climb, 2)     # dikey hız (m/s)
    elif tip == 'SERVO_OUTPUT_RAW':   # motorlara giden komut mesajı
        son['m1'] = m.servo1_raw      # PWM değeri: 1000 = durgun, 2000 = tam güç
        son['m2'] = m.servo2_raw
        son['m3'] = m.servo3_raw
        son['m4'] = m.servo4_raw
    elif tip == 'GPS_RAW_INT':     # GPS mesajı
        son['uydu'] = None if m.satellites_visible == 255 else m.satellites_visible   # 255 = bilinmiyor
    elif tip == 'ATTITUDE':        # EK 2: duruş (yatış) mesajı
        son['roll'] = round(math.degrees(m.roll), 1)     # roll = yana yatış; radyan gelir, math.degrees ile dereceye çevir
        son['pitch'] = round(math.degrees(m.pitch), 1)   # pitch = öne/arkaya eğim (derece)
    elif tip == 'PARAM_VALUE':     # EK 3: SITL'in "şu ayar şu değerde" cevabı; ayar gönderdiğimizde onay olarak bunu bekleriz
        pid = m.param_id           # ayarın adı
        if isinstance(pid, bytes): # bazen ad bytes türünde (ham veri) gelir; öyleyse yazıya çevir
            pid = pid.decode('utf-8', 'ignore')
        param_degerleri[pid.rstrip('\x00')] = m.param_value   # sondaki boş karakterleri ('\x00') at, ayarın adıyla değerini kaydet


def satir_yaz(simdi):              # CSV'ye bir satır yazar
    if simdi - durum['son_paket'] > 3:   # 3 sn'den uzun süredir mesaj yoksa veri bayattır, yazma
        return
    if son['armed'] is None or son['voltaj'] is None or son['irtifa'] is None:   # temel değerlerden biri henüz hiç gelmediyse yazma (eksik satır olmasın)
        return
    t = simdi - T0                 # kayıt başladığından beri geçen süre
    alanlar = [                    # bir CSV satırının değerleri; sıra BASLIK'taki sütunlarla aynı olmalı
        f'{t:.1f}', durum['etiket'], son['armed'], son['voltaj'], son['akim'], son['kalan'],   # zaman, ETİKET (normal ya da arıza adı), armed, voltaj, akım, kalan yüzde
        son['irtifa'], son['hiz'], son['gaz'], son['tirmanma'],
        son['m1'], son['m2'], son['m3'], son['m4'],
        son['uydu'], son['enlem'], son['boylam'], baglanti.flightmode,   # uydu, konum, uçuş modu
        son['roll'], son['pitch'],
    ]
    dosya.write(','.join('' if a is None else str(a) for a in alanlar) + '\n')   # her değeri yazıya çevir (None ise boş bırak), virgülle birleştir, dosyaya yaz
    dosya.flush()                  # hemen diske yazdır
    durum['satir'] += 1            # satır sayacını 1 artır
    if simdi - durum['son_yazdirma'] >= 10:   # son ekran özetinden 10 sn geçtiyse
        durum['son_yazdirma'] = simdi
        print(f"{t:5.0f} sn | {durum['etiket']:<16} | armed {son['armed']} | V {son['voltaj']} | "
              f"A {son['akim']} | irtifa {son['irtifa']} | gaz {son['gaz']} | "
              f"motor {son['m1']} {son['m2']} {son['m3']} {son['m4']} | "
              f"roll {son['roll']} | mod {baglanti.flightmode}")   # ekrana tek satır özet; {etiket:<16} etiketi 16 karakter genişliğe sola yaslar


def rc_gonder():
    # MAVProxy'deki "rc 3 1500" ile aynı şey: gaz kanalını ortada tut.
    # Neden: CIRCLE ve LOITER'da irtifayı gaz çubuğu belirler; SITL'de kumanda yok, çubuk en altta sayılıyor ve drone düşüyor.
    # Override en fazla birkaç sn geçerli kalır, bu yüzden sürekli tekrar gönderilir.
    baglanti.mav.rc_channels_override_send(      # kumanda kanallarını biz belirliyoruz
        baglanti.target_system, baglanti.target_component,   # kime: drone'a
        65535, 65535, 1500, 65535, 65535, 65535, 65535, 65535)   # 8 kanal: 65535 = "bu kanala dokunma"; 3. kanal (gaz) = 1500 (ortada)


# ---------------------------------------------------------------------------
# KISIM 6: ÇEKİRDEK DÖNGÜ VE BEKLEME
# ---------------------------------------------------------------------------
def adim():
    """Tek dongu: bir mesaj oku, gerekirse RC gonder, 1 Hz satir yaz, dusmeyi kontrol et."""
    m = baglanti.recv_match(blocking=True, timeout=0.2)    # bir mesaj bekle (en fazla 0,2 sn); gelmezse None
    simdi = time.monotonic()                               # şu anki an
    if m is not None:                                      # mesaj geldiyse
        durum['son_paket'] = simdi                         # "en son mesaj şimdi geldi" diye not et
        guncelle(m)                                        # kutuları güncelle
    if durum['rc_aktif'] and simdi >= durum['sonraki_rc']: # daire modunda ve gaz sinyalini gönderme zamanı geldiyse
        rc_gonder()                                        # gaz kanalını ortada tut
        durum['sonraki_rc'] = simdi + 0.5                  # bir sonrakini 0,5 sn sonra gönder
    if simdi >= durum['sonraki_yazim']:                    # satır yazma zamanı geldi mi? (saniyede bir)
        durum['sonraki_yazim'] += PERIYOT                  # bir sonrakinin zamanını 1 sn ileri al
        if simdi - durum['sonraki_yazim'] > 2 * PERIYOT:   # hâlâ 2 sn'den fazla geride isek (örn. bilgisayar durakladıysa)
            durum['sonraki_yazim'] = simdi + PERIYOT       # arka arkaya satır yazmaya çalışma, zamanı şimdiye ayarla
        satir_yaz(simdi)                                   # satırı yaz
    if durum['havada'] and son['irtifa'] is not None and son['irtifa'] < DUSME_ESIGI:   # havadayken irtifa 5 m'nin altına indiyse
        raise Dusme(f"irtifa {son['irtifa']} m (esik {DUSME_ESIGI} m)")   # düşme hatası fırlat; ucus() durur


def bekle_kosul(kosul, sn):        # "şu koşul gerçekleşene ya da süre dolana kadar bekle"; kosul: True/False döndüren bir fonksiyon
    bitis = time.monotonic() + sn  # bekleme bitiş anı
    while time.monotonic() < bitis:   # süre dolmadıkça
        adim()                     # beklerken bile okuma, kayıt ve düşme kontrolü sürer
        if kosul():                # koşul sağlandıysa
            return True            # beklemeyi bırak, "oldu" de
    return kosul()                 # süre doldu; koşul son kez kontrol edilir (True ya da False döner)


def bekle(sn):                     # koşulsuz bekleme: sadece süreyi doldur (kayıt sürer)
    bekle_kosul(lambda: False, sn) # lambda: False = her çağrıda False dönen küçük isimsiz fonksiyon; koşul hiç sağlanmaz, süre dolana kadar bekler


# ---------------------------------------------------------------------------
# KISIM 7: KOMUT GÖNDERME VE DOĞRULAMA
# Hepsi aynı kalıpta: komutu gönder, işe yaradığını doğrula, olmadıysa tekrar dene,
# yine olmadıysa Hata verip dur. Yani komut göndermekle yetinmiyoruz, etkisini kontrol ediyoruz.
# ---------------------------------------------------------------------------
def komut(cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):   # p1..p7 komutun parametreleri; verilmeyen 0 olur
    baglanti.mav.command_long_send(    # MAVLink'in genel komut mesajı
        baglanti.target_system, baglanti.target_component, cmd, 0, p1, p2, p3, p4, p5, p6, p7)   # kime, hangi komut, 0 = onay sayacı (kullanmıyoruz), parametreler


def mod_ayarla(ad):                # uçuş modunu değiştirir
    for _ in range(3):             # 3 kez dene (_ = sayacı kullanmıyoruz)
        komut(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, MODLAR[ad])   # mod değiştir komutu; 1 = özel mod numarası kullanıyoruz, sonraki sayı = modun numarası
        if bekle_kosul(lambda: baglanti.flightmode == ad, 4):       # uçuş modu istediğimiz mod oldu mu? en fazla 4 sn bekle
            olay(f'mod {ad}')      # oldu, olay olarak kaydet
            return                 # fonksiyondan çık
    raise Hata(f'{ad} moduna gecilemedi (mod su an {baglanti.flightmode})')   # 3 denemede olmadı: hata ver   


def param_ayarla(ad, deger):       # SITL ayarını değiştirir ve değiştiğini doğrular
    for _ in range(3):             # 3 kez dene
        param_degerleri.pop(ad, None)          # bu ayarın eski onayını sil ki yenisini bekleyebilelim (yoksa hata verme)
        baglanti.param_set_send(ad, float(deger))   # ayarı gönder (float: ondalıklı sayı)
        # SITL ayarın değerini bildirdi mi VE bizim gönderdiğimiz değere eşit mi? (1e-3 = 0.001 hata payı); en fazla 3 sn bekle
        if bekle_kosul(lambda: ad in param_degerleri and abs(param_degerleri[ad] - deger) < 1e-3, 3):
            olay(f'param {ad} = {deger}')      # onaylandı, olay olarak kaydet
            return
    raise Hata(f'{ad} = {deger} SITL tarafindan onaylanmadi')   # 3 denemede onaylanmadı: hata ver


def arm_et():                      # motorları çalıştırmaya hazır hâle getirir (arm)
    for _ in range(24):            # 24 deneme x 5 sn = yaklaşık 2 dakika
        komut(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)   # arm komutu (1 = arm et)
        if bekle_kosul(lambda: son['armed'] == 1, 5):            # armed oldu mu? 5 sn bekle
            olay('ARM tamam')
            return
        olay('ARM olmadi (GPS / pre-arm bekleniyor olabilir), tekrar deneniyor')   # GPS kilidi ya da ön kontroller bitmemiş olabilir
    raise Hata('arm edilemedi')


def kalkis():                      # kalkış yapar
    komut(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, IRTIFA)   # kalkış komutu; son parametre = hedef irtifa (m)
    olay(f'kalkis komutu, hedef {IRTIFA} m')
    if not bekle_kosul(lambda: son['irtifa'] is not None and son['irtifa'] >= IRTIFA - 1, 60):   # irtifa 9 m'ye ulaştı mı? en fazla 60 sn bekle
        raise Hata(f"{IRTIFA - 1} m'ye cikilamadi (irtifa {son['irtifa']})")
    olay(f"hedef irtifaya ulasildi ({son['irtifa']} m)")


# ---------------------------------------------------------------------------
# KISIM 8: GÜVENLİK KONTROLÜ
# ---------------------------------------------------------------------------
def batarya_normal_mi():
    """Voltaj, akimdan beklenen degere uyuyor mu? (SITL modeli: V = 12.6 - 0.01*I, 6 akimda olculdu)
    Etiketler komutlardan uretildigi icin, komut ise yaramazsa etiket yalan soyler.
    Bu kontrol, bataryanin gercekten normale dondugunu sinyalden dogrular."""
    if son['voltaj'] is None or son['akim'] is None:   # voltaj ya da akım henüz gelmediyse karar veremeyiz
        return True                                    # kaydı engellemesin diye "normal" say
    return abs(son['voltaj'] - (12.6 - 0.01 * son['akim'])) < BATARYA_TOL_V   # |gerçek voltaj - beklenen voltaj| 0.05'ten küçükse True (normal)


# ---------------------------------------------------------------------------
# KISIM 9: ANA SENARYO
# ---------------------------------------------------------------------------
def ucus(plan, daire):             # plan: arızaların sıralı listesi; daire: True ise daire modunda uçar
    param_ayarla('DISARM_DELAY', 60)   # drone'un kendiliğinden disarm olmasını geciktiren ayar (SITL her açılışta ayarları sildiği için script kendisi yapıyor)
    mod_ayarla('GUIDED')           # GUIDED mod: komutla kalkış için gerekli
    arm_et()                       # motorları hazırla
    kalkis()                       # 10 m'ye kalk
    durum['havada'] = True         # artık havadayız; düşme kontrolü başlasın
    if daire:                      # daire modu seçildiyse
        durum['rc_aktif'] = True   # gaz sinyali göndermeye başla (yoksa drone düşer)
        mod_ayarla('LOITER')       # önce LOITER (bulunduğu yerde sabit kal)
        bekle(5)
        mod_ayarla('CIRCLE')       # sonra CIRCLE (daire çiz)
    olay(f'kararlasma bekleniyor ({KARARLI_SN} sn)')
    bekle(KARARLI_SN)              # drone'un oturması için 10 sn bekle
    if not batarya_normal_mi():    # başlangıçta batarya normal değilse (önceki koşudan arıza kalmış olabilir)
        raise Hata(f"baslangicta batarya modeli normal degil (V {son['voltaj']}, A {son['akim']}). "
                   "Onceki kosudan kalan ariza olabilir: baslat.sh ile SITL'i yeniden baslat")
    durum['etiket'] = 'normal'     # buradan itibaren satırlar 'normal' etiketiyle yazılır
    olay(f'NORMAL ucus basladi ({BASLANGIC_SN} sn)')
    bekle(BASLANGIC_SN)            # 40 sn arızasız normal uçuş
    for ad, uygula, geri_al in plan:         # arızaları sırayla uygula (her arıza üç parçalı grup, burada üçüne ayrı isim veriyoruz)
        olay(f'ARIZA BASLIYOR: {ad}')
        for p, v in uygula:                  # arızanın ayarlarını sırayla uygula (p = ayar adı, v = değer)
            param_ayarla(p, v)               # SITL onaylayana kadar bekler
        durum['etiket'] = ad                 # etiket, SITL ayarı onayladıktan SONRA değişir; yani etiket arızanın gerçekten açık olduğu aralığı gösterir
        bekle(ARIZA_SN)                      # arıza 30 sn açık kalsın
        for p, v in geri_al:                 # arızayı geri al
            param_ayarla(p, v)
        durum['etiket'] = 'normal'           # geri alma onaylandıktan sonra etiket 'normal' olur
        olay(f'ARIZA BITTI: {ad}')
        bekle(TOPARLANMA_SN)                 # 30 sn toparlanma
        if not batarya_normal_mi():          # arıza geri alındığı hâlde batarya normale dönmediyse
            durum['etiket'] = 'bilinmiyor'   # bu satırlar güvenilmez, 'bilinmiyor' yaz
            raise Hata(f"{ad} sonrasi batarya modeli normale donmedi (V {son['voltaj']}, A {son['akim']}): "
                       "bu kosudaki 'normal' etiketleri guvenilmez")   # koşuyu durdur
    if daire:
        mod_ayarla('LOITER')       # daire bitti, sabit kal
        bekle(10)
    durum['rc_aktif'] = False      # gaz sinyalini kes
    durum['havada'] = False        # inişe geçeceğiz, düşme kontrolü kapansın
    mod_ayarla('LAND')             # iniş
    if not bekle_kosul(lambda: son['armed'] == 0, 120):   # yere inip disarm oldu mu? en fazla 120 sn bekle
        raise Hata('inis sonrasi disarm olmadi')
    olay('inis tamam, disarm')


def guvenli_son():
    """Normal bitis de hata da buradan gecer: SITL'i temiz birak, havadaysak indir."""
    durum['havada'] = False        # düşme kontrolünü kapat (artık inişe geçiyoruz)
    durum['rc_aktif'] = False      # gaz sinyalini kes
    try:                           # Ctrl+C'ye ikinci kez basılırsa temizlik yarıda kesilebilsin diye
        if son['armed'] == 1:      # hâlâ havadaysak (armed ise)
            durum['etiket'] = 'sonlandirma'
            komut(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, MODLAR['LAND'])   # inişe geç
            olay('LAND komutu gonderildi')
        for ad, v in TEMIZLIK:     # bütün arıza ayarlarını normale döndür
            try:
                param_ayarla(ad, v)
            except Hata as e:      # bir ayar onaylanmazsa program çökmesin, uyarı yazıp devam et (e = hata mesajı)
                olay(f'UYARI: temizlik basarisiz: {e}')
        if son['armed'] == 1:      # hâlâ armed ise disarm olmasını bekle
            bekle_kosul(lambda: son['armed'] == 0, 120)
    except KeyboardInterrupt:      # temizlik sırasında Ctrl+C'ye basıldı
        olay('ikinci Ctrl+C: temizlik yarida kesildi')


# ---------------------------------------------------------------------------
# KISIM 10: GİRİŞ NOKTASI
# ---------------------------------------------------------------------------
def main(argv=None):
    global baglanti, dosya, olay_dosya, T0   # fonksiyon içinde bu dış değişkenleri değiştireceğiz
    argv = sys.argv if argv is None else argv   # argüman verilmediyse terminalden gelen kelimeleri kullan
    args = argv[1:]                # 1. elemandan sonuna kadar: dosya adını (0. eleman) atıp sadece bizim yazdıklarımızı al
    if not args:                   # hiçbir şey yazılmadıysa
        print(__doc__)             # dosyanın başındaki kullanım yazısını ekrana bas
        return 2                   # 2 = yanlış kullanım (çıkış kodu)
    ad = args[0]                   # 1. kelime: dosya adında kullanılacak ad
    ekstra = args[1:]              # kalan kelimeler (filtre ve/veya 'daire')
    daire = 'daire' in ekstra      # 'daire' yazıldıysa True, yazılmadıysa False
    filtreler = [a for a in ekstra if a not in ('daire', 'hepsi')]   # kalan kelimelerden 'daire' ve 'hepsi' olmayanları topla (filtre bunlardan biri)
    filtre = filtreler[0] if filtreler else None   # varsa ilk filtre, yoksa None (hepsi)

    secili = [a for a in ARIZALAR if filtre is None or filtre in a[0]]   # filtre yoksa tüm arızalar; varsa etiketinde (a[0]) filtre yazısı geçenler
    if not secili:                 # hiçbir arıza eşleşmediyse
        print(f"'{filtre}' ile eslesen ariza yok. Etiketler:")
        print(', '.join(a[0] for a in ARIZALAR))   # bütün etiketleri listele
        return 2
    tohum = int(time.time())       # şu anki zaman (saniye); karıştırma için tohum
    plan = list(secili)            # seçilen arızaların kopyası
    random.Random(tohum).shuffle(plan)   # arıza sırasını karıştır; aynı tohum aynı sırayı verir (tohum olay dosyasına yazılıyor, sıra tekrar üretilebilir)

    os.makedirs(KLASOR, exist_ok=True)   # klasör yoksa oluştur
    damga = datetime.now().strftime('%Y%m%d_%H%M%S')   # tarih-saat damgası, örn. 20261017_150020
    dosya_adi = os.path.join(KLASOR, f'ucus_{ad}_{damga}.csv')            # veri dosyasının yolu
    olay_adi = os.path.join(KLASOR, f'ucus_{ad}_{damga}_olaylar.csv')     # olay dosyasının yolu

    sifirla()                      # kutuları ve durumu hazırla
    baglanti = mavutil.mavlink_connection(BAGLANTI)   # drone bağlantısını aç
    print(f'{BAGLANTI} dinleniyor, heartbeat bekleniyor...')
    if baglanti.wait_heartbeat(timeout=15) is None:   # 15 sn içinde kalp atışı gelmezse
        print("Heartbeat gelmedi. MAVProxy'de su komutu yaz: output add 127.0.0.1:14553")
        return 1                   # 1 = hata (çıkış kodu)

    dosya = open(dosya_adi, 'w')   # veri dosyasını yazmak için aç
    dosya.write(BASLIK + '\n')     # ilk satıra sütun adlarını yaz
    olay_dosya = open(olay_adi, 'w')   # olay dosyasını aç
    olay_dosya.write('zaman_sn,olay\n')   # olay dosyasının sütun adları
    bas = time.monotonic()         # başlangıç anı
    T0 = bas                       # olay() bunu kullanarak "kaçıncı sn" hesaplar
    durum['son_yazdirma'] = durum['sonraki_yazim'] = durum['son_paket'] = bas   # üç zaman değişkenine aynı değeri ata
    toplam = BASLANGIC_SN + len(plan) * (ARIZA_SN + TOPARLANMA_SN)   # tahmini uçuş süresi: 40 + arıza sayısı x (30 + 30) sn
    print(f'Kayit: {dosya_adi}')
    print(f'Olaylar: {olay_adi}')
    print(f"Mod: {'daire' if daire else 'hover'} | {len(plan)} ariza | sadece ucus kismi yaklasik {toplam} sn")
    olay(f'tohum {tohum}, plan: ' + ' '.join(a[0] for a in plan))   # tohumu ve arıza sırasını olay dosyasına yaz

    cikis = 0                      # çıkış kodu: 0 = sorunsuz
    try:
        ucus(plan, daire)          # asıl senaryoyu çalıştır
    except Dusme as e:             # düşme algılandıysa
        durum['etiket'] = 'dusme'
        olay(f'DUSME ALGILANDI: {e}')
        cikis = 1
    except KeyboardInterrupt:      # Ctrl+C'ye basıldıysa
        olay('kullanici durdurdu (Ctrl+C)')
        cikis = 1
    except Hata as e:              # bir adım başarısız olduysa (e = hata mesajı)
        olay(f'HATA: {e}')
        cikis = 1
    finally:                       # ne olursa olsun (başarı, hata, Ctrl+C) burası çalışır
        guvenli_son()              # SITL'i temiz bırak, havadaysak indir
        dosya.close()              # dosyaları kapat
        olay_dosya.close()
    print(f"Kaydedilen satir: {durum['satir']}")
    print(f'Dosya: {dosya_adi}')
    return cikis                   # çıkış kodunu döndür


if __name__ == '__main__':         # bu dosya doğrudan çalıştırıldıysa (başka dosyadan çağrılmadıysa)
    sys.exit(main())               # main()'i çalıştır; döndürdüğü sayı programın çıkış kodu olur (0 = sorunsuz) 