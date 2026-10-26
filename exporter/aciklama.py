"""
Alarmın "neden" açıklaması: hangi sinyal normalden ne kadar saptı, olası neden ne.

Sadece standart kütüphane kullanır (anomali_model.py gibi). Etiket ya da arıza bilgisi
KULLANMAZ: sadece 6 ölçüme (4 motor PWM, voltaj, akım) ve modelin öğrendiği normal
değerlere bakar.

Nasıl çalışır:
  1. Sapma = ölçülen - modelin öğrendiği normal ortalama (motor için yaklaşık 1604 PWM).
  2. Pay: Mahalanobis mesafesinin karesi sinyallere şöyle bölünür: c_i = d_i * (P d)_i.
     Toplamları mesafenin karesine eşittir; her sinyalin payı "alarmın ne kadarı bu sinyalden
     geliyor" demektir.
  3. Olası neden basit bir kuraldır:
       voltaj + akım payı >= B_ESIK          -> batarya
       tek bir motorun payı >= M_ESIK         -> o motor
       ikisi de değil                         -> neden ayırt edilemedi
  4. Telafi payı: motorun PWM aralığının (1000-2000) yüzde kaçını kullandığı.
  5. Teşhis payı ('guven'): seçilen nedenin (motor ya da voltaj+akım) alarm skorundaki payı.
     Bu bir olasılık DEĞİL; skorun hangi sinyalden geldiğini gösterir.
  6. Normal bant: voltaj ve akım için modelin öğrendiği ortalama ± BANT_SIGMA standart sapma
     (standart sapma, modelin kovaryans matrisinin tersinden hesaplanır).

Sınırlar (ekranda da "olası" diye yazılır):
  - Sadece simülatörde tanımladığımız arıza türlerini ayırt eder (tek motor, batarya).
  - Voltaj düşüşü ile iç direnç artışı aynı izi bırakır; ikisine de "batarya" denir.
  - B_ESIK ve M_ESIK elle seçildi; analiz/aciklama_olc.py kayıtlı verilerde ölçer.
"""

PWM_MIN = 1000.0
PWM_MAX = 2000.0          # MOT_PWM_MIN/MAX varsayılan aralığı kabul edilir
M_ESIK = 0.60             # tek motorun alarm içindeki payı bundan büyükse "o motor"
B_ESIK = 0.30             # voltaj + akım payı bundan büyükse "batarya"
DIGER_NORMAL_PAY = 0.05   # diğer sinyallerin toplam payı bunun altındaysa "diğerleri normal" denir
# Neden iddia etmek için en az bu kadar sapma gerekir; daha küçükse (ör. arıza geri alınırken
# değerler normale dönerken) "neden ayırt edilemedi" denir. İkisi de elle seçildi.
MIN_MOTOR_SAPMA = 10.0    # PWM
MIN_VOLTAJ_SAPMA = 0.10   # V (akım için aşağıda MIN_AKIM_SAPMA)
MIN_AKIM_SAPMA = 1.0      # A
BANT_SIGMA = 3.0          # "normal bant" = ortalama ± 3 standart sapma (voltaj ve akım için)

MOTOR_ADLARI = ['motor1_pwm', 'motor2_pwm', 'motor3_pwm', 'motor4_pwm']
ETIKETLER = {'motor1_pwm': 'Motor 1', 'motor2_pwm': 'Motor 2', 'motor3_pwm': 'Motor 3',
             'motor4_pwm': 'Motor 4', 'voltaj_v': 'Voltaj', 'akim_a': 'Akım'}
BIRIMLER = {'motor1_pwm': 'PWM', 'motor2_pwm': 'PWM', 'motor3_pwm': 'PWM',
            'motor4_pwm': 'PWM', 'voltaj_v': 'V', 'akim_a': 'A'}


def _sayi(x, basamak=0, isaretli=False):
    """Türkçe ondalık virgülü: 12.31 -> '12,31'. İşaretli ise '+33' / '−0,41' (gerçek eksi işareti)."""
    s = f"{abs(x):.{basamak}f}".replace('.', ',')
    if isaretli:
        if round(x, basamak) == 0:
            return s
        return ('+' if x > 0 else '−') + s
    return ('−' if x < 0 and round(x, basamak) != 0 else '') + s


def _katki(ortalama, ters_kov, x):
    n = len(x)
    d = [x[i] - ortalama[i] for i in range(n)]
    Pd = [sum(ters_kov[i][j] * d[j] for j in range(n)) for i in range(n)]
    return d, [d[i] * Pd[i] for i in range(n)]


def _ters(a):
    """Küçük kare matrisin tersi (Gauss-Jordan, kısmi pivot). Sadece standart kütüphane."""
    n = len(a)
    m = [list(map(float, a[i])) + [1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]
    for k in range(n):
        p = max(range(k, n), key=lambda i: abs(m[i][k]))
        if abs(m[p][k]) < 1e-12:
            raise ValueError('matris tersi alinamadi')
        m[k], m[p] = m[p], m[k]
        bol = m[k][k]
        m[k] = [v / bol for v in m[k]]
        for i in range(n):
            if i != k and m[i][k] != 0.0:
                f = m[i][k]
                m[i] = [vi - f * vk for vi, vk in zip(m[i], m[k])]
    return [row[n:] for row in m]


class Aciklayici:
    def __init__(self, model):
        self.model = model
        # Her rejim için sinyallerin standart sapması (kovaryansın köşegeni); normal bant için.
        self._sigma = {}
        for ad, r in model.rejimler.items():
            try:
                kov = _ters(r['ters_kovaryans'])
                self._sigma[ad] = [max(kov[i][i], 0.0) ** 0.5 for i in range(len(kov))]
            except (ValueError, KeyError):
                self._sigma[ad] = None

    def acikla(self, mod, x, alarm=False, skor=None, tur_kural=None):
        """
        mod       : uçuş modu (GUIDED/LOITER/CIRCLE); modelde yoksa None döner
        x         : [m1, m2, m3, m4, voltaj, akım] (model.ozellikler sırasıyla)
        alarm     : model alarm verdi mi (neden sadece alarmda aranır)
        skor      : mesafe/eşik oranı (ekranda göstermek için, isteğe bağlı)
        tur_kural : exporter'daki arıza türü kuralının sonucu ('batarya', 'motor1'..'motor4').
                    Verilirse çapraz kontrol yapılır: buradaki kendi kuralımızla aynı şeyi diyorsa
                    neden gösterilir (Telegram ve Grafana ile aynı); ayrışırsa 'uyumsuz' True olur
                    ve ekran "neden ayırt edilemedi" der (yanlış neden göstermemek için).
        Dönüş     : sözlük (JSON'a çevrilebilir) ya da None
        """
        rj = self.model.rejim(mod)
        if rj is None:
            return None
        r = self.model.rejimler[rj]
        isimler = self.model.ozellikler
        d, c = _katki(r['ortalama'], r['ters_kovaryans'], x)
        poz = [max(ci, 0.0) for ci in c]
        toplam = sum(poz) or 1e-12
        pay = [p / toplam for p in poz]

        sig = self._sigma.get(rj)
        sinyaller = []
        for i, ad in enumerate(isimler):
            sg = {'ad': ad, 'etiket': ETIKETLER.get(ad, ad), 'birim': BIRIMLER.get(ad, ''),
                  'olculen': x[i], 'normal': r['ortalama'][i], 'sapma': d[i], 'pay': pay[i],
                  'bant_alt': None, 'bant_ust': None, 'bantta': None}
            if sig is not None and ad in ('voltaj_v', 'akim_a'):
                sg['bant_alt'] = r['ortalama'][i] - BANT_SIGMA * sig[i]
                sg['bant_ust'] = r['ortalama'][i] + BANT_SIGMA * sig[i]
                sg['bantta'] = sg['bant_alt'] <= x[i] <= sg['bant_ust']
            sinyaller.append(sg)

        telafi = []
        for ad in MOTOR_ADLARI:
            if ad in isimler:
                i = isimler.index(ad)
                telafi.append({
                    'etiket': ETIKETLER[ad],
                    'kullanim_yuzde': (x[i] - PWM_MIN) / (PWM_MAX - PWM_MIN) * 100.0,
                    'normalde_yuzde': (r['ortalama'][i] - PWM_MIN) / (PWM_MAX - PWM_MIN) * 100.0,
                })

        sonuc = {'rejim': rj, 'alarm': bool(alarm), 'skor': skor, 'tur': 'normal', 'motor': None,
                 'baslik': 'Her şey normal',
                 'cumle': 'Model 6 sinyali izliyor. Hepsi öğrenilen normal aralıkta.',
                 'kisa': '', 'tur_kendi': None, 'tur_kural': None, 'uyumsuz': False,
                 'guven': None, 'guven_kaynak': None,
                 'oneri': None, 'anons': None,
                 'sinyaller': sinyaller, 'telafi': telafi}
        if not alarm:
            return sonuc

        mi = [isimler.index(a) for a in MOTOR_ADLARI]
        vi, ai = isimler.index('voltaj_v'), isimler.index('akim_a')
        batarya_pay = pay[vi] + pay[ai]
        en_iyi = max(mi, key=lambda i: pay[i])

        # Kendi kuralımız (pay tabanlı)
        if batarya_pay >= B_ESIK:
            kendi, kendi_ad = ('batarya', None), 'batarya'
        elif pay[en_iyi] >= M_ESIK:
            kendi, kendi_ad = ('motor', en_iyi), f"motor{mi.index(en_iyi) + 1}"
        else:
            kendi, kendi_ad = ('belirsiz', None), 'belirsiz'
        # Exporter'ın tür kuralı verildiyse iki kural birbirini doğrular: ikisi aynı şeyi diyorsa
        # neden gösterilir, ayrışıyorsa (ör. arıza geri alınırken değerler normale dönerken)
        # ekran neden iddia etmez.
        sonuc['tur_kendi'] = kendi_ad
        sonuc['tur_kural'] = tur_kural
        sonuc['uyumsuz'] = bool(tur_kural) and tur_kural != kendi_ad
        secim = ('belirsiz', None) if sonuc['uyumsuz'] else kendi
        # Seçilen nedenin sapması çok küçükse iddia etme
        if secim[0] == 'motor' and abs(d[secim[1]]) < MIN_MOTOR_SAPMA:
            secim = ('belirsiz', None)
        elif secim[0] == 'batarya' and abs(d[vi]) < MIN_VOLTAJ_SAPMA and abs(d[ai]) < MIN_AKIM_SAPMA:
            secim = ('belirsiz', None)

        if secim[0] == 'batarya':
            sonuc.update(self._batarya(x, d, r, mi, vi, ai))
            sonuc['guven'], sonuc['guven_kaynak'] = batarya_pay, 'Voltaj + akım'
            sonuc['oneri'] = {'metin': 'Güvenli bir yerde in.',
                              'alt': f"Voltaj {_sayi(x[vi], 2)} V ve akım {_sayi(x[ai], 1)} A normal bandın dışında."}
            sonuc['anons'] = 'Batarya sorunu olabilir. İnmeni öneriyorum.'
        elif secim[0] == 'motor':
            sonuc.update(self._motor(isimler[secim[1]], secim[1], x, d, r, pay))
            n = sonuc['motor']
            sonuc['guven'], sonuc['guven_kaynak'] = pay[secim[1]], f"Motor {n}"
            kul = telafi[n - 1]['kullanim_yuzde']
            if d[secim[1]] > 0:
                alt = f"Motor {n} PWM kullanımı %{_sayi(kul)}. %100'e kadar %{_sayi(max(0.0, 100.0 - kul))} pay kaldı."
            else:
                alt = f"Motor {n} PWM değeri normalden düşük."
            sonuc['oneri'] = {'metin': 'Güvenli bir yerde in.', 'alt': alt, 'kullanim_yuzde': kul}
            sonuc['anons'] = f"Motor {n} arızası olabilir. İnmeni öneriyorum."
        else:
            ilk = sorted(range(len(pay)), key=lambda i: -pay[i])[:3]
            liste = ', '.join(f"{ETIKETLER.get(isimler[i], isimler[i])} ({_sayi(d[i], 2 if i >= 4 else 0, True)})"
                              for i in ilk)
            sonuc.update({'tur': 'belirsiz',
                          'baslik': 'Anomali var, neden ayırt edilemedi',
                          'cumle': f"Birden fazla sinyal normalden saptı: {liste}.",
                          'kisa': liste,
                          'oneri': {'metin': 'Uçuşu kontrol et, gerekirse güvenli bir yerde in.',
                                    'alt': 'Model sapma görüyor ama nedenini güvenle ayırt edemiyor.'},
                          'anons': 'Anomali var, neden belirsiz. Uçuşu kontrol et.'})
        return sonuc

    def _motor(self, ad, i, x, d, r, pay):
        n = MOTOR_ADLARI.index(ad) + 1
        diger = 1.0 - pay[i]
        diger_cumle = ('Diğer motorlar, voltaj ve akım normal.' if diger < DIGER_NORMAL_PAY
                       else 'Başka sinyallerde de sapma var.')
        if d[i] > 0:
            neden = f"Olası neden: motor {n} gücü düşmüş, kontrolcü daha fazla PWM vererek telafi ediyor."
        else:
            neden = f"Motor {n} PWM değeri normalden düşük."
        cumle = (f"Motor {n} PWM değeri {_sayi(x[i])}; normalde {_sayi(r['ortalama'][i])} "
                 f"({_sayi(d[i], 0, True)}). {diger_cumle} {neden}")
        kisa = f"Motor {n} PWM {_sayi(x[i])}, normalden {_sayi(d[i], 0, True)}"
        return {'tur': 'motor', 'motor': n, 'baslik': f"Motor {n} arızalı (olası)", 'cumle': cumle, 'kisa': kisa}

    def _batarya(self, x, d, r, mi, vi, ai):
        ort_motor = sum(d[i] for i in mi) / len(mi)
        yon = 'Voltaj düştü ve akım arttı.' if (d[vi] < 0 and d[ai] > 0) else 'Voltaj ve akım normalden farklı.'
        cumle = (f"Voltaj {_sayi(x[vi], 2)} V (normalde {_sayi(r['ortalama'][vi], 2)}, {_sayi(d[vi], 2, True)}); "
                 f"akım {_sayi(x[ai], 1)} A (normalde {_sayi(r['ortalama'][ai], 1)}, {_sayi(d[ai], 1, True)}); "
                 f"motorlar ortalama {_sayi(ort_motor, 0, True)} PWM. {yon} "
                 f"Olası neden: batarya (voltaj düşüşü ya da yüksek iç direnç).")
        kisa = (f"Voltaj {_sayi(x[vi], 2)} V ({_sayi(d[vi], 2, True)}), "
                f"akım {_sayi(x[ai], 1)} A ({_sayi(d[ai], 1, True)})")
        return {'tur': 'batarya', 'motor': None, 'baslik': 'Batarya sorunu (olası)', 'cumle': cumle, 'kisa': kisa}


def jamming_aciklama(uydu, fix_tipi):
    """GPS jamming şüphesi için ekran metni. Kural exporter.py'deki ile aynı: fix_tipi >= 3 ve uydu >= 6 sağlıklı."""
    return {'tur': 'jamming', 'baslik': 'GPS jamming şüphesi',
            'cumle': f"Uydu sayısı {uydu} (en az 6 olmalı), fix tipi {fix_tipi} (en az 3 olmalı). "
                     f"Olası neden: GPS sinyali bozuluyor.",
            'kisa': f"Uydu sayısı {uydu} (en az 6 olmalı)",
            'oneri': {'metin': 'GPS konumuna güvenme, konum hatalı olabilir.',
                      'alt': f"Uydu sayısı {uydu}, fix tipi {fix_tipi}."},
            'anons': 'GPS jamming şüphesi. Konuma güvenme.'}