"""
Canli kullanim icin sade Mahalanobis modeli. numpy/pandas GEREKTIRMEZ, sadece standart kutuphane.

Modeli model/mahalanobis_v1.json dosyasindan yukler ve tek bir olcum satirini puanlar:
  mesafe : olcumun "normal ucus bulutunun merkezine" uzakligi (Mahalanobis mesafesi)
  esik   : bu uzakliktan buyukse tuhaf sayilir
  alarm  : mesafe > esik
"""
import json
import math

# Ariza turu kurali (model DEGIL, elle yazilmis basit kural; simulatorun normal bataryasi icin).
# Normal batarya: voltaj = 12,6 - 0,01 x akim. Batarya arizasinda bu iliski bozulur.
# Motor arizasinda bozulmaz, sadece bir motorun PWM'i digerlerinden ayrilir.
BATARYA_BOS_V = 12.6          # akim cekilmiyorken voltaj
BATARYA_DIRENC_OHM = 0.01     # normal ic direnc: akim basina voltaj dusumu
BATARYA_ARTIK_ESIK_V = 0.05   # |olculen voltaj - beklenen voltaj| bundan buyukse batarya arizasi


class MahalanobisModeli:
    def __init__(self, veri):
        self.ozellikler = veri['ozellikler']          # sayilarin sirasi: bu sirayla verilmeli
        self.rejimler = veri['rejimler']
        self.mod_rejim = {}
        for ad, r in self.rejimler.items():
            for m in r['modlar']:
                self.mod_rejim[m] = ad

    @classmethod
    def yukle(cls, yol):
        with open(yol, encoding='utf-8') as f:
            return cls(json.load(f))

    def rejim(self, mod):
        """Ucus modundan hangi model kullanilacak? (hover / daire / yok)"""
        return self.mod_rejim.get(mod)

    def mesafe(self, rejim, x):
        r = self.rejimler[rejim]
        d = [xi - mi for xi, mi in zip(x, r['ortalama'])]
        P = r['ters_kovaryans']
        n = len(d)
        toplam = 0.0
        for i in range(n):
            satir = P[i]
            toplam += d[i] * sum(satir[j] * d[j] for j in range(n))
        return math.sqrt(max(toplam, 0.0))

    def degerlendir(self, mod, x):
        """(mesafe, esik, alarm) dondurur. Bu modda model yoksa ya da olcum eksikse None."""
        rj = self.rejim(mod)
        if rj is None:
            return None
        if any(v is None or (isinstance(v, float) and math.isnan(v)) for v in x):
            return None
        md = self.mesafe(rj, x)
        esik = self.rejimler[rj]['esik']
        return md, esik, md > esik

    def tur(self, mod, x):
        """
        Alarm veren olcumun ariza turu: 'batarya' ya da 'motor1'..'motor4'. Bu modda model yoksa None.
        x = degerlendir() ile ayni 6 sayi, ayni sirada (4 motor PWM, voltaj, akim).
          1) voltaj-akim iliskisi bozulduysa -> batarya
          2) bozulmadiysa -> normal ortalamadan en cok sapan motor
        """
        rj = self.rejim(mod)
        if rj is None:
            return None
        voltaj, akim = x[4], x[5]
        if abs(voltaj - (BATARYA_BOS_V - BATARYA_DIRENC_OHM * akim)) > BATARYA_ARTIK_ESIK_V:
            return 'batarya'
        ort = self.rejimler[rj]['ortalama']
        sapma = [abs(x[i] - ort[i]) for i in range(4)]
        return 'motor%d' % (sapma.index(max(sapma)) + 1)


class AnomaliDedektoru:
    """
    Canli kullanim: modeli bir "kapi" ile sarar.

    Model sadece havada, sabit hover/daire ucusunda ogrendi. Yerde, kalkista, inista ve
    mod degisimlerinden sonraki ilk saniyelerde motor degerleri farkli olur; orada karar vermek
    yanlis alarm uretir. Bu yuzden karar sadece kapi acikken verilir:
      - motorlar calisiyor (armed)
      - irtifa en az MIN_IRTIFA_M (kalkis bitti)
      - ucus modu modelde taniniyor (GUIDED/LOITER/CIRCLE)
      - bu kosullar en az BEKLEME_SN saniyedir kesintisiz sagliyor (gecis anlari bitti)
    """
    MIN_IRTIFA_M = 9.0
    BEKLEME_SN = 5.0

    def __init__(self, model):
        self.model = model
        self.hazir_baslangic = None
        self.onceki_rejim = None

    def karar(self, simdi, d):
        """
        simdi : zaman (saniye, geri sarmayan saat)
        d     : {'mod','armed','irtifa','m1'..'m4','voltaj','akim'} son bilinen degerler
        Donus : (sebep, mesafe, esik, alarm)
                sebep None ise karar verildi; degilse karar yok ve sebep yazili (mesafe/esik/alarm None).
        """
        rj = self.model.rejim(d.get('mod'))
        kapi = None
        if not d.get('armed'):
            kapi = 'motorlar kapali'
        elif d.get('irtifa') is None or d['irtifa'] < self.MIN_IRTIFA_M:
            kapi = 'irtifa dusuk'
        elif rj is None:
            kapi = 'mod taninmiyor'

        if kapi is not None or rj != self.onceki_rejim:
            self.hazir_baslangic = None if kapi is not None else simdi
        self.onceki_rejim = rj if kapi is None else None

        if kapi is not None:
            return kapi, None, None, None
        if simdi - self.hazir_baslangic < self.BEKLEME_SN:
            return 'gecis bekleniyor', None, None, None

        x = [d['m1'], d['m2'], d['m3'], d['m4'], d['voltaj'], d['akim']]
        r = self.model.degerlendir(d['mod'], x)
        if r is None:
            return 'olcum eksik', None, None, None
        md, esik, alarm = r
        return None, md, esik, alarm

    def tur(self, d):
        """Alarm anindaki ariza turu ('batarya' / 'motor1'..'motor4'). d: karar() ile ayni sozluk."""
        x = [d['m1'], d['m2'], d['m3'], d['m4'], d['voltaj'], d['akim']]
        return self.model.tur(d['mod'], x)