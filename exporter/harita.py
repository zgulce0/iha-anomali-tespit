"""
Canlı 3B harita için veri: exporter MAVLink mesajlarını buraya da verir, sayfa (harita_sayfa.html)
saniyede birkaç kez /harita/veri adresinden okur.

Haritada üç konum ayrı çizilir:
  gerçek  : aracın GERÇEK konumu. Sadece simülasyonda bilinir (SITL'in SIMSTATE mesajı).
            Gerçek bir araçta bu mesaj yoktur; harita o zaman sadece GPS ve ArduPilot tahminini çizer.
  gps     : GPS'in söylediği konum (GPS_RAW_INT). Spoofing'te bu kayar.
  tahmin  : ArduPilot'un kendi tahmini (GLOBAL_POSITION_INT). Spoofing'te GPS'i izler.
  filtre  : BİZİM filtremizin aracın gerçek konumu için tahmini (GPS eksi kestirilen GPS hatası).
            Gerçek bir araçta da vardır. Yavaş kaymada GPS'ten ayrılıp gerçeğe yakın kalır; ani
            sıçramadan sonra filtre yeni konuma yeniden başladığı için GPS'e oturur (orada
            "son güvenilir konum" işe yarar).
Yükseklik olarak ArduPilot'un yerden yüksekliği (barometre ağırlıklı) kullanılır.

Ayrıca: iki konumun izleri, alarm anlarının iğneleri (spoofing, jamming, arıza) ve
"son güvenilir konum" (GPS alarmı başlamadan hemen önceki GPS konumu).

Sadece standart kütüphane; diske bir şey yazmaz.
"""
import json
import math
import os
import threading
import time
from collections import deque

M_DERECE = 111320.0
IZ_HZ = 2.0                 # iz noktası sıklığı
IZ_MAKS = 1800              # 15 dakikalık iz
GUVENILIR_GERI_SN = 2.0     # son güvenilir konum: alarm başlangıcından bu kadar önceki GPS konumu
ANAHTAR_YOLU = os.path.expanduser(os.environ.get('CESIUM_ANAHTAR_DOSYASI', '~/.iha_cesium_token'))


def mesafe_m(a, b):
    """İki (enlem, boylam) arası yaklaşık mesafe (m); kısa mesafeler için yeterli."""
    if a is None or b is None:
        return None
    dn = (a[0] - b[0]) * M_DERECE
    de = (a[1] - b[1]) * M_DERECE * math.cos(math.radians(a[0]))
    return math.hypot(dn, de)


def cesium_anahtari():
    """Cesium ion anahtarı: ortam değişkeni ya da ~/.iha_cesium_token dosyası. Yoksa None.
    Anahtar proje klasöründe tutulmaz, git'e girmez."""
    a = os.environ.get('CESIUM_ANAHTARI', '').strip()
    if a:
        return a
    try:
        with open(ANAHTAR_YOLU, encoding='utf-8') as f:
            return f.read().strip() or None
    except OSError:
        return None


class HaritaDurumu:
    def __init__(self):
        self.kilit = threading.Lock()
        self.gercek = None       # (enlem, boylam)
        self.gps = None          # (enlem, boylam)
        self.tahmin = None       # (enlem, boylam)
        self.filtre = None       # (enlem, boylam): bizim filtrenin gerçek konum tahmini
        self.irtifa = None       # yerden yükseklik (m)
        self.tutum = (0.0, 0.0, 0.0)   # roll, pitch, yaw (radyan)
        self.uydu = None
        self.fix = None
        self.iz_gercek = deque(maxlen=IZ_MAKS)
        self.iz_gps = deque(maxlen=IZ_MAKS)    # (saat, enlem, boylam)
        self._son_iz = 0.0
        self.olaylar = deque(maxlen=60)
        self.ev = None
        self.son_guvenilir = None
        self.durum = {}
        self._onceki = {'spoof': False, 'jam': False, 'model': False, 'armed': 0}
        self.simdi = time.monotonic

    # --- MAVLink mesajları ---
    def mesaj(self, m):
        tip = m.get_type()
        with self.kilit:
            g = lambda ad, v=0: getattr(m, ad, v)
            if tip == 'SIMSTATE':
                if g('lat') or g('lng'):
                    self.gercek = (g('lat') / 1e7, g('lng') / 1e7)
            elif tip == 'GPS_RAW_INT':
                self.uydu, self.fix = g('satellites_visible', None), g('fix_type', None)
                if (self.fix or 0) >= 3 and (g('lat') or g('lon')):
                    self.gps = (g('lat') / 1e7, g('lon') / 1e7)
            elif tip == 'GLOBAL_POSITION_INT':
                if g('lat') or g('lon'):
                    self.tahmin = (g('lat') / 1e7, g('lon') / 1e7)
                self.irtifa = g('relative_alt') / 1000.0
            elif tip == 'ATTITUDE':
                self.tutum = (g('roll', 0.0), g('pitch', 0.0), g('yaw', 0.0))
            else:
                return
            an = self.simdi()
            if an - self._son_iz >= 1.0 / IZ_HZ and self.irtifa is not None:
                self._son_iz = an
                h = round(self.irtifa, 2)
                if self.gercek:
                    self.iz_gercek.append((round(an, 1), round(self.gercek[0], 7), round(self.gercek[1], 7), h))
                if self.gps:
                    self.iz_gps.append((round(an, 1), round(self.gps[0], 7), round(self.gps[1], 7), h))

    def filtre_konum(self, konum):
        with self.kilit:
            self.filtre = konum

    # --- saniyede bir: alarm durumları ---
    def kare(self, saat, armed, spoof, jam, model, tur=None, baslik=None, voltaj=None, batarya=None, mod=None):
        """saat: datetime. tur: arıza türü ('motor1'..'motor4', 'batarya') ya da None."""
        with self.kilit:
            an = self.simdi()
            o = self._onceki
            if armed and not o['armed']:        # yeni uçuş: izleri ve iğneleri temizle
                self.iz_gercek.clear(); self.iz_gps.clear(); self.olaylar.clear()
                self.son_guvenilir = None
                self.ev = self.gercek or self.gps
            konum = self.gercek or self.tahmin or self.gps
            if (spoof and not o['spoof']) or (jam and not o['jam']):
                # aynı GPS olayının 2 sn öncesinden kalan kısa "arıza" iğnesi: o da bu olayın parçası, kaldır
                for x in [x for x in self.olaylar if x['tur'] == 'ariza' and an - x.get('_an', -99) <= 2.5]:
                    self.olaylar.remove(x)
            if spoof and not o['spoof']:
                self._olay(saat, 'spoofing', 'GPS spoofing şüphesi', konum)
                self.son_guvenilir = self._geriden_gps(an, GUVENILIR_GERI_SN)
            if jam and not o['jam']:
                self._olay(saat, 'jamming', 'GPS jamming şüphesi', konum)
                self.son_guvenilir = self._geriden_gps(an, GUVENILIR_GERI_SN)
            if model and not o['model']:
                self._olay(saat, 'ariza', baslik or 'Arıza', konum)
            self._onceki = {'spoof': spoof, 'jam': jam, 'model': model, 'armed': armed}
            self.durum = {'armed': armed, 'spoof': spoof, 'jam': jam, 'model': model, 'tur': tur,
                          'baslik': baslik, 'voltaj': voltaj, 'batarya': batarya, 'mod': mod,
                          'saat': saat.strftime('%H:%M:%S')}

    def _olay(self, saat, tur, metin, konum):
        if konum is None:
            return
        self.olaylar.append({'tur': tur, 'metin': metin, 'saat': saat.strftime('%H:%M:%S'), '_an': self.simdi(),
                             'enlem': round(konum[0], 7), 'boylam': round(konum[1], 7),
                             'irtifa': round(self.irtifa or 0.0, 1),
                             'gps': [round(self.gps[0], 7), round(self.gps[1], 7)] if self.gps else None})

    def _geriden_gps(self, an, geri):
        hedef = an - geri
        aday = None
        for s, la, lo, h in self.iz_gps:
            if s <= hedef:
                aday = (la, lo, h)
            else:
                break
        if aday is None and self.iz_gps:
            aday = self.iz_gps[0][1:]
        return None if aday is None else {'enlem': aday[0], 'boylam': aday[1], 'irtifa': aday[2]}

    def konum_ozeti(self):
        """Uçuş kaydı (rapor) için: gerçek konum, GPS konumu, son güvenilir konum."""
        with self.kilit:
            return {'gercek': self.gercek, 'gps': self.gps, 'son_guvenilir': self.son_guvenilir}

    def json(self):
        with self.kilit:
            veri = {
                'gercek': self.gercek, 'gps': self.gps, 'tahmin': self.tahmin, 'filtre': self.filtre,
                'filtre_hatasi_m': None if (self.gercek is None or self.filtre is None) else round(mesafe_m(self.gercek, self.filtre), 2),
                'irtifa': self.irtifa, 'tutum': [round(x, 4) for x in self.tutum],
                'uydu': self.uydu, 'fix': self.fix,
                'gps_hatasi_m': None if self.gercek is None else (
                    None if self.gps is None else round(mesafe_m(self.gercek, self.gps), 2)),
                'iz_gercek': [p[1:] for p in self.iz_gercek],
                'iz_gps': [p[1:] for p in self.iz_gps],
                'olaylar': [{k: v for k, v in x.items() if k != '_an'} for x in self.olaylar],
                'son_guvenilir': self.son_guvenilir,
                'ev': self.ev, 'durum': self.durum,
                'simulasyon': self.gercek is not None,
            }
        return json.dumps(veri, ensure_ascii=False).encode('utf-8')
