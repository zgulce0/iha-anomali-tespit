"""
Canli kullanim icin sade Mahalanobis modeli. numpy/pandas GEREKTIRMEZ, sadece standart kutuphane.

Modeli model/mahalanobis_v1.json dosyasindan yukler ve tek bir olcum satirini puanlar:
  mesafe : olcumun "normal ucus bulutunun merkezine" uzakligi (Mahalanobis mesafesi)
  esik   : bu uzakliktan buyukse tuhaf sayilir
  alarm  : mesafe > esik
"""
import json
import math


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