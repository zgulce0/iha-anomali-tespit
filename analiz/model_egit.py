"""
Normal ucus verisinden Mahalanobis modelini egitir ve model/mahalanobis_v1.json olarak kaydeder.

Kullanim (repo kokunden):
  venv/bin/python analiz/model_egit.py HOVER.csv DAIRE.csv BAGIMSIZ_NORMAL.csv

Model = her ucus rejimi (hover, daire) icin 3 sey:
  1) ortalama        : normal ucusta 6 sayinin (4 motor, voltaj, akim) tipik degeri
  2) ters kovaryans  : sayilarin birbiriyle nasil birlikte oynadigi (bulutun sekli)
  3) esik            : "bundan uzaktaysa tuhaf" siniri = egitimdeki en uzak normal noktanin 1,5 kati

Egitime SADECE normal satirlar girer. Ucuncu dosya (baska gunun normal ucusu) egitime girmez,
sadece sinav icin: modelin hic gormedigi normal ucusta yanlis alarm veriyor mu?
"""
# ---------------------------------------------------------------------------
# BU DOSYANIN ÖZETİ (açıklama satırları sonradan eklendi, kod aynı)
# Terminalde 3 CSV dosyası verilir. Dosya sırayla şunları yapar:
#   1) CSV'lerden "normal" satırları seçer          (segmentle + temiz_satirlar)
#   2) 6 sayının ortalamasını ve ilişkisini çıkarır (egit)  -> bu "model"dir
#   3) modeli model/mahalanobis_v1.json'a yazar     (main'in ilk yarısı)
#   4) iki sınav yapar: yanlış alarm var mı, arızalar yakalanıyor mu (main'in ikinci yarısı)
# ---------------------------------------------------------------------------
import os                          # dosya yolu işleri için
import sys                         # terminalde yazılan kelimelere (sys.argv) ulaşmak için
import json                        # modeli .json dosyasına yazmak için
import datetime                    # bugünün tarihini modele yazmak için
import numpy as np                 # matris hesapları (ortalama, kovaryans, ters alma) için
import pandas as pd                # CSV'yi tablo olarak okumak ve satır seçmek için

# ---------------------------------------------------------------------------
# KISIM 1: AYARLAR VE İÇE AKTARMA
# ---------------------------------------------------------------------------
# __file__ = bu dosyanın yolu (.../analiz/model_egit.py). dirname bir üst klasöre çıkar.
# İki kere çıkınca repo kök klasörüne gelinir (.../iha-anomali-tespit)
KOK = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# anomali_model.py 'exporter' klasöründe, bu dosya 'analiz' klasöründe. Python'a "exporter klasörüne de bak" diyoruz
sys.path.insert(0, os.path.join(KOK, 'exporter'))
# Canlı sistemin kullandığı AYNI sınıf. Sınavda kendi ayrı kodumuzu değil, canlıdaki kodu sınıyoruz
from anomali_model import MahalanobisModeli

# Modelin baktığı 6 sayı (CSV'deki sütun adları). Sıra önemli: model dosyası bu sırayla kaydedilir
OZELLIKLER = ['motor1_pwm', 'motor2_pwm', 'motor3_pwm', 'motor4_pwm', 'voltaj_v', 'akim_a']
# OZELLIKLER ile aynı sırada: her sayının ölçülebilen en küçük adımı (PWM 1, voltaj 0,01 V, akım 0,05 A)
COZUNURLUK = [1.0, 1.0, 1.0, 1.0, 0.01, 0.05]      # sensorun olcebildigi en kucuk adim
# Hangi rejim hangi uçuş modlarında geçerli. Hover GUIDED ve LOITER'da, daire CIRCLE'da
REJIMLER = {'hover': ['GUIDED', 'LOITER'], 'daire': ['CIRCLE']}
KIRPMA_SN = 5.0        # her etiket degisiminden sonraki ilk 5 sn alinmaz (gecis anlari)
ESIK_CARPANI = 1.5     # eşik = eğitimdeki en uzak normal mesafe x 1,5 (seçilmiş bir pay)
CIKTI = os.path.join(KOK, 'model', 'mahalanobis_v1.json')  # modelin yazılacağı dosya


# ---------------------------------------------------------------------------
# KISIM 2: segmentle() - her satıra "bu etiket kaç saniyedir sürüyor" bilgisini ekler
# ---------------------------------------------------------------------------
def segmentle(d)     :                  # d = CSV'den okunan tablo (pandas DataFrame)
    d = d.copy()                   # orijinal tabloyu bozmamak için kopyasını al
    # shift() sütunu 1 satır aşağı kaydırır; böylece her satırın etiketi bir üstündeki satırın etiketiyle karşılaştırılır.
    # Etiket değiştiyse True, değişmediyse False çıkar. cumsum() bu True'ları (True = 1) yukarıdan aşağı toplar:
    # aynı etiketli ardışık satırlar aynı numarayı alır, etiket değişince numara 1 artar.
    # Örnek: etiketler  normal normal normal motor1 motor1 normal  ->  seg numaraları  1 1 1 2 2 3
    d['seg'] = (d['etiket'] != d['etiket'].shift()).cumsum()
    # transform('min'): her segmentin İLK satırının zamanını, o segmentin tüm satırlarına yazar.
    # Satırın zamanından bunu çıkarınca "bu segment başladığından beri geçen saniye" kalır (0, 1, 2, ...)
    d['seg_t'] = d['zaman_sn'] - d.groupby('seg')['zaman_sn'].transform('min')
    return d                       # iki yeni sütun eklenmiş tabloyu geri ver


# ---------------------------------------------------------------------------                                                                                                                                         
# KISIM 3: temiz_satirlar() - eğitime/sınava girmeye uygun satırları süzer
# ---------------------------------------------------------------------------
def temiz_satirlar(d, modlar):     # modlar = bu rejimin uçuş modları, örn. ['GUIDED', 'LOITER']
    # Üç şartı da sağlayan satırlar kalır ('&' = ve):
    #   armed == 1            -> motorlar çalışıyor
    #   mod modlar listesinde -> bu rejimin uçuş modunda
    #   etiket 'hazirlik' değil -> uçuş öncesi hazırlık satırları alınmaz
    # Her şart ayrı parantez içinde olmak zorunda (Python '&' için böyle ister)
    d = d[(d['armed'] == 1) & (d['mod'].isin(modlar)) & (d['etiket'] != 'hazirlik')]
    d = d[d['seg_t'] >= KIRPMA_SN]  # etiket değişiminden sonraki ilk 5 saniyeyi at (geçiş anları karışık)
    return d.dropna(subset=OZELLIKLER)  # 6 sayıdan biri boş olan satırları at


# ---------------------------------------------------------------------------
# KISIM 4: egit() - normal satırlardan modeli hesaplar (asıl öğrenme burada)
# ---------------------------------------------------------------------------
def egit(normal_satirlar):         # sadece etiketi 'normal' olan, süzülmüş satırlar gelir
    # 6 sütunu al, tabloyu düz sayı matrisine çevir: her satır bir ölçüm, 6 sütun. astype(float) = ondalıklı sayı yap
    X = normal_satirlar[OZELLIKLER].values.astype(float)
    # axis=0: her SÜTUNUN ortalaması -> 6 sayılık liste (normal uçuşta 6 sayının tipik değeri)
    ort = X.mean(axis=0)
    # np.cov = kovaryans tablosu (6x6): köşegende her sayının kendi yayılımı, diğer yerlerde iki sayının birlikte değişimi.
    # X.T tabloyu yan çevirir; np.cov değişkenlerin satırlarda olmasını bekler.
    # '+ np.diag(...)': köşegene sensör adımının karesini ekler. Bir sayı neredeyse hiç değişmiyorsa (yayılımı ~0),
    # tersi alınınca çok büyük sayı çıkar ve en küçük sapma bile dev mesafe verirdi. Sensör zaten o adımdan
    # daha hassas ölçemediği için yayılıma bu alt sınırı koyuyoruz.
    C = np.cov(X.T) + np.diag(np.array(COZUNURLUK) ** 2)
    P = np.linalg.inv(C)           # kovaryans tablosunun tersi (matrisin tersi); mesafe formülü bunu kullanır
    D = X - ort                    # her satırdan ortalamayı çıkar: her ölçümün ortalamadan sapması
    # einsum('ij,jk,ik->i', D, P, D): her satır i için D[i] x P x D[i] işlemini yapar (Mahalanobis mesafesinin karesi).
    # np.sqrt karekökünü alır. Sonuç: her eğitim satırı için bir mesafe sayısı
    mesafeler = np.sqrt(np.einsum('ij,jk,ik->i', D, P, D))
    # Geri verilenler: ortalama, ters tablo, eşik (en büyük mesafe x 1,5), en büyük mesafe, satır sayısı
    return ort, P, ESIK_CARPANI * mesafeler.max(), mesafeler.max(), len(X)


# ---------------------------------------------------------------------------
# KISIM 5: main() - eğit, kaydet, sına
# ---------------------------------------------------------------------------
def main():
    # sys.argv[1:4] = terminalde program adından sonra yazılan ilk 3 kelime (sıra: hover, daire, bağımsız normal).
    # Sırayla üç değişkene konur
    hover_csv, daire_csv, bagimsiz_csv = sys.argv[1:4]
    hover = segmentle(pd.read_csv(hover_csv))   # hover CSV'sini oku, segment sütunlarını ekle
    daire = segmentle(pd.read_csv(daire_csv))   # daire CSV'sini oku, segment sütunlarını ekle

    # Model dosyasının iskeleti. 'rejimler' şimdilik boş, aşağıdaki döngüde dolacak
    model = {'versiyon': 1, 'tarih': datetime.date.today().isoformat(),
             'ozellikler': OZELLIKLER, 'cozunurluk': COZUNURLUK,
             'esik_carpani': ESIK_CARPANI, 'kirpma_sn': KIRPMA_SN, 'rejimler': {}}
    # Döngü iki kere döner: önce ad='hover', d=hover tablosu; sonra ad='daire', d=daire tablosu
    for ad, d in (('hover', hover), ('daire', daire)):
        # Önce etiketi 'normal' olan satırları seç, sonra temiz_satirlar ile süz. Arızalı satırlar buraya HİÇ girmez
        normal = temiz_satirlar(d[d['etiket'] == 'normal'], REJIMLER[ad])
        ort, P, esik, md_max, n = egit(normal)  # modeli hesapla
        # Bu rejimin sonuçlarını model sözlüğüne yaz. numpy sayıları json'a doğrudan yazılamaz;
        # tolist() ve float() onları normal Python listesine/sayısına çevirir
        model['rejimler'][ad] = {'modlar': REJIMLER[ad], 'egitim_satiri': n,
                                 'ortalama': ort.tolist(), 'ters_kovaryans': P.tolist(),
                                 'egitimdeki_en_uzak': float(md_max), 'esik': float(esik)}
        print(f"[{ad}] egitim satiri {n} | egitimdeki en uzak mesafe {md_max:.2f} | esik {esik:.2f}")  # ekrana özet

    os.makedirs(os.path.dirname(CIKTI), exist_ok=True)  # model klasörü yoksa oluştur
    with open(CIKTI, 'w', encoding='utf-8') as f:        # model dosyasını yazmak için aç; with bitince otomatik kapanır
        # indent=1: dosya okunabilir girintili olsun. ensure_ascii=False: Türkçe harfler bozulmadan yazılsın
        json.dump(model, f, indent=1, ensure_ascii=False)
    print("Model kaydedildi:", CIKTI)

    # ---- SINAV: kaydedilen dosyadan yukleyip, canli kullanimdaki kodla puanla ----
    # Burada egit()'in değişkenlerini değil, diskteki JSON dosyasını yüklüyoruz. Dosya bozuksa burada anlaşılır
    m = MahalanobisModeli.yukle(CIKTI)

    def puanla(satirlar):          # satirlar = tablo; her satır için (mesafe, eşik, alarm) listesi verir
        sonuc = []
        # satirlar[['mod'] + OZELLIKLER]: mod sütunu + 6 sayı sütunu. itertuples(index=False, name=None) her satırı düz bir demet yapar.
        # 'for mod, *x in': demetin ilk elemanı mod'a, geri kalan 6 sayı x listesine girer
        for mod, *x in satirlar[['mod'] + OZELLIKLER].itertuples(index=False, name=None):
            r = m.degerlendir(mod, [float(v) for v in x])  # canlıdaki puanlama: (mesafe, eşik, alarm) ya da None
            if r is not None:      # None = bu modda model yok ya da ölçüm eksik; o satırı sayma
                sonuc.append(r)
        return sonuc

    # -------- SINAV 1: modelin hiç görmediği normal uçuşta yanlış alarm var mı? --------
    print("\n=== SINAV 1: modelin hic gormedigi normal ucus (15 Ekim) -> yanlis alarm var mi? ===")
    eski = pd.read_csv(bagimsiz_csv)  # üçüncü dosya: başka günün normal uçuşu (eğitime girmedi)
    eski['grp'] = (eski['mod'] != eski['mod'].shift()).cumsum()  # segmentle'deki hile; burada uçuş modu değişince numara artar
    eski['t'] = eski['zaman_sn'] - eski.groupby('grp')['zaman_sn'].transform('min')  # mod başladığından beri geçen saniye
    # Motorlar çalışıyor ve moda geçtikten en az 10 sn geçmiş satırlar kalsın; 6 sayıdan biri boşsa at
    eski = eski[(eski['armed'] == 1) & (eski['t'] >= 10)].dropna(subset=OZELLIKLER)
    for mod in ('LOITER', 'CIRCLE'):   # hover rejimini LOITER, daire rejimini CIRCLE satırlarıyla sına
        r = puanla(eski[eski['mod'] == mod])
        if r:                          # liste boş değilse
            # sum(a for _, _, a in r): her (mesafe, eşik, alarm) üçlüsünün alarm kısmını topla (True = 1) = yanlış alarm sayısı.
            # '_' = o değeri kullanmıyorum demek. max(...) = en uzak mesafe. r[0][1] = eşik. ':7' ve ':4d' yazı hizalama biçimi
            print(f"{mod:7} {len(r):4d} satir | yanlis alarm {sum(a for _, _, a in r)} | "
                  f"en uzak mesafe {max(md for md, _, _ in r):.2f} (esik {r[0][1]:.2f})")

    # -------- SINAV 2: arızalı uçuşlarda arızalı saniyelerin kaçı yakalandı? --------
    print("\n=== SINAV 2: arizali ucuslar -> kac arizali saniye yakalandi? ===")
    for ad, d in (('hover', hover), ('daire', daire)):
        # Etiketi 'normal' OLMAYAN satırlar = arızalı satırlar (etiket burada cevap anahtarı)
        ar = temiz_satirlar(d[d['etiket'] != 'normal'], REJIMLER[ad])
        zayif = None               # en zor yakalanan arıza (henüz yok)
        top = yak = 0              # top = toplam arızalı satır, yak = alarm verilen satır
        for et, g in ar.groupby('etiket'):   # her arıza etiketi ayrı grup: et = arıza adı, g = o arızanın satırları
            r = puanla(g)
            a = sum(x for _, _, x in r)      # bu arızada alarm verilen satır sayısı
            top += len(r); yak += a          # ';' = aynı satırda iki işlem: toplamları büyüt
            oran = float(np.median([md / es for md, es, _ in r]))    # esigin kac kati
            # oran: her satırın mesafesi / eşik, bunların ortancası. 1'den büyükse eşiğin üstünde (alarm).
            # En küçük oranlı arıza en zor yakalanandır; ilk seferde ya da daha küçük oran çıkınca güncelle
            if zayif is None or oran < zayif[1]:
                zayif = (et, oran)
        print(f"{ad:6} {yak}/{top} arizali satir yakalandi (%{100 * yak / top:.1f}) | "
              f"en zor ariza: {zayif[0]}, mesafesi esigin {zayif[1]:.1f} kati")


if __name__ == '__main__':         # dosya doğrudan çalıştırılırsa main() çalışır; başka dosya import ederse çalışmaz
    main()