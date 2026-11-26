"""
Operatör ekranı: exporter'ın yanında çalışan küçük, SADECE OKUMA yapan web sayfası.

  http://localhost:8088/              canlı ekran (ekran_sayfa.html)
  http://localhost:8088/durum         anlık durum (JSON)
  http://localhost:8088/rapor         son uçuşun olay raporu (ekran_rapor.html); ?n=1 bir önceki uçuş
  http://localhost:8088/gecmis        rapor verisi (JSON); ?n=0 son uçuş, ?n=1 bir önceki
  http://localhost:8088/harita        canlı 3B harita (harita_sayfa.html, veri: harita.py)

Sadece standart kütüphane. exporter.py her karar anında (saniyede bir) EkranDurumu.guncelle()
çağırır; sunucu ayrı bir iş parçacığında çalışır. Sayfa kendisi saniyede bir /durum'u okur.

Uçuş kaydı: motorlar çalışınca (armed) başlar, motorlar durunca biter. Bellekte tutulur
(en fazla MAKS_UCUS uçuş); exporter kapanınca silinir. Hiçbir şey diske yazılmaz.

Ortam değişkenleri (hepsi isteğe bağlı):
  EKRAN_PORT         varsayılan 8088
  EKRAN_HOST         varsayılan 127.0.0.1 (sadece bu bilgisayar)
  GRAFANA_PANEL_URL  Grafana "Share > Embed" adresi; boşsa sayfada yer tutucu görünür
"""
import json
import os
import threading
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from aciklama import jamming_aciklama
from harita import mesafe_m

try:
    from anomali_model import AnomaliDedektoru
    MIN_IRTIFA_M = AnomaliDedektoru.MIN_IRTIFA_M
    BEKLEME_SN = AnomaliDedektoru.BEKLEME_SN
except Exception:           # ekran tek başına da çalışsın (testler)
    MIN_IRTIFA_M, BEKLEME_SN = 9.0, 5.0

KLASOR = os.path.dirname(os.path.abspath(__file__))
SAYFA_YOLU = os.path.join(KLASOR, 'ekran_sayfa.html')
RAPOR_YOLU = os.path.join(KLASOR, 'ekran_rapor.html')
HARITA_YOLU = os.path.join(KLASOR, 'harita_sayfa.html')
MAKS_OLAY = 30
MAKS_UCUS = 5              # bellekte tutulan uçuş sayısı
MAKS_SERI = 7200           # uçuş başına skor örneği (saniyede 1 örnek ~ 2 saat)
GERCEK_UCUS_M = 1.0        # bundan alçak kalan "uçuş" sadece yerde arm edilmiştir; kayda alınmaz
# GPS alarmı sürerken (ya da bittikten sonraki GPS_BAGLI_SN içinde) başlayan model alarmı, ilk GPS_BAGLI_SN
# boyunca "dikkat" olarak gösterilir: araç GPS'teki değişikliğe göre konumunu düzeltirken motorlar kısa bir
# ayar yapar, model bunu görür. Bundan uzun süren model alarmı normal arıza alarmı olur (gerçek arıza gizlenmez).
GPS_BAGLI_SN = 5.0
# Aynı GPS sıçraması model alarmını GPS alarmından biraz ÖNCE de başlatabilir (model saniyede bir karar
# verir, spoofing testi 0,2-0,6 sn sürer). GPS alarmı başladığında son GPS_ONCE_SN içinde başlamış kısa
# model alarmı da GPS'e bağlı sayılır.
GPS_ONCE_SN = 2.0
IZ_MAKS = 3600              # rapordaki yol haritası için saniyede bir konum (1 saat)
GPS_BAGLI = {
    'baslik': 'Kısa sapma (GPS olayına bağlı)',
    'kisa': 'Araç konumu düzeltirken motorlar kısa bir ayar yaptı',
    'cumle': ('GPS alarmı sırasında motorlarda kısa bir sapma görüldü. Araç, GPS konumundaki değişikliğe göre '
              'kendini düzeltirken motorlara kısa bir ayar yaptırıyor. 5 saniyeden uzun sürerse arıza alarmına döner.'),
    'oneri': {'metin': 'Şimdilik izle.', 'alt': '5 saniyeden uzun sürerse ekran arıza alarmı verir.'},
}


def _hms(dt):
    return dt.strftime('%H:%M:%S')


def _birlesim_sn(araliklar):
    """[(bas, bit), ...] aralıklarının birleşiminin uzunluğu (üst üste binenler iki kez sayılmaz)."""
    toplam, son_bit = 0.0, None
    for bas, bit in sorted(araliklar):
        if son_bit is None or bas > son_bit:
            toplam += bit - bas
            son_bit = bit
        elif bit > son_bit:
            toplam += bit - son_bit
            son_bit = bit
    return toplam


class Ucus:
    """Tek bir uçuşun kaydı: alarmlar, skor örnekleri, küçük istatistikler."""

    def __init__(self, saat, numara):
        self.numara = numara
        self.baslangic = saat
        self.bitis = None
        self.son_t = 0.0
        self.alarmlar = []                      # kayıt sözlükleri, başlangıç sırasıyla
        self.aktif = {'model': None, 'gps': None}
        self.seri = deque(maxlen=MAKS_SERI)     # (t_sn, skor ya da None)
        self.irtifa_max = 0.0
        self.batarya_ilk = None
        self.batarya_son = None
        self.iz = deque(maxlen=IZ_MAKS)         # (t, gerçek enlem, boylam, GPS enlem, boylam); gerçek sadece simülasyonda
        self.son_konum = None                   # son bilinen konum (alarm iğnesi için)

    def t(self, saat):
        t = max((saat - self.baslangic).total_seconds(), self.son_t)   # saat geri sıçrarsa geri gitme
        self.son_t = t
        return t

    # --- alarm kayıtları -------------------------------------------------------------
    def alarm_ac(self, kaynak, saat, t):
        r = {'id': len(self.alarmlar) + 1, 'kaynak': kaynak, 'tip': None, 'motor': None,
             'etiket': '', 'baslik': '', 'ne': '', 'bas': _hms(saat), 'bit': None,
             'bas_sn': round(t, 1), 'bit_sn': None, 'tepe_skor': None, 'tepe_t': None,
             'kanit': None, 'kanit_skor': None, 'kanit_net': False}
        self.alarmlar.append(r)
        self.aktif[kaynak] = r
        return r

    def alarm_kapat(self, kaynak, saat, t):
        r = self.aktif[kaynak]
        if r is not None:
            r['bit'] = _hms(saat)
            r['bit_sn'] = round(t, 1)
            self.aktif[kaynak] = None

    def model_alarm_isle(self, saat, t, aciklama, skor, gps_bagli=False):
        r = self.aktif['model'] or self.alarm_ac('model', saat, t)
        if r.get('gps_bagli') and not gps_bagli:
            # 5 sn'yi aştı: artık gerçek alarm; kanıtı baştan topla
            r['kanit'], r['kanit_skor'], r['kanit_net'] = None, None, False
        r['gps_bagli'] = gps_bagli
        if skor is not None and (r['tepe_skor'] is None or skor > r['tepe_skor']):
            r['tepe_skor'], r['tepe_t'] = round(skor, 3), round(t, 1)
        net = aciklama['tur'] != 'belirsiz'
        # Kanıt olarak alarm boyunca en iyi anı sakla: net açıklama, belirsizden önce gelir;
        # aynı türde en yüksek skorlu an (en büyük sapma arızanın kendisidir; ilk ve son
        # saniyelerde değerler henüz oturmamış ya da normale dönüyor olabilir).
        daha_iyi = (r['kanit'] is None or (net and not r['kanit_net'])
                    or (net == r['kanit_net'] and skor is not None and skor > (r['kanit_skor'] or 0.0)))
        if daha_iyi:
            r['tip'], r['motor'] = aciklama['tur'], aciklama.get('motor')
            r['etiket'] = (f"motor {aciklama['motor']}" if aciklama['tur'] == 'motor'
                           else 'batarya' if aciklama['tur'] == 'batarya' else 'belirsiz')
            r['baslik'], r['ne'] = aciklama['baslik'], aciklama['kisa']
            r['kanit_skor'], r['kanit_net'] = skor, net
            r['kanit'] = {
                'cumle': aciklama['cumle'], 'rejim': aciklama['rejim'], 'saat': _hms(saat),
                'guven': aciklama.get('guven'), 'guven_kaynak': aciklama.get('guven_kaynak'),
                'oneri': aciklama.get('oneri'), 'skor': skor,
                'sinyaller': [{k: sg.get(k) for k in ('etiket', 'birim', 'olculen', 'normal', 'sapma', 'pay',
                                                       'bant_alt', 'bant_ust', 'bantta')}
                              for sg in aciklama['sinyaller']],
                'telafi': aciklama['telafi'],
            }
        if gps_bagli:
            r['tip'], r['motor'], r['etiket'] = 'gps_bagli', None, "GPS'e bağlı"
            r['baslik'], r['ne'] = GPS_BAGLI['baslik'], GPS_BAGLI['kisa']
            r['kanit'] = dict(r['kanit'], cumle=GPS_BAGLI['cumle'], oneri=GPS_BAGLI['oneri'],
                              guven=None, guven_kaynak=None)

    def gps_bagli_yap(self, r):
        """Bir model alarmı kaydını 'GPS olayına bağlı kısa sapma' olarak işaretle."""
        r['gps_bagli'] = True
        r['tip'], r['motor'], r['etiket'] = 'gps_bagli', None, "GPS'e bağlı"
        r['baslik'], r['ne'] = GPS_BAGLI['baslik'], GPS_BAGLI['kisa']
        if r['kanit'] is not None:
            r['kanit'] = dict(r['kanit'], cumle=GPS_BAGLI['cumle'], oneri=GPS_BAGLI['oneri'],
                              guven=None, guven_kaynak=None)

    def gps_alarm_isle(self, saat, t, jam, uydu, fix):
        yeni = self.aktif['gps'] is None
        r = self.aktif['gps'] or self.alarm_ac('gps', saat, t)
        if yeni:
            r['konum'] = self.son_konum
            r['gps_hata_maks'] = None
            r['guvenilir_hata_m'] = None
            # GPS alarmından hemen önce başlamış kısa model alarmları aynı olayın parçası
            for a in reversed(self.alarmlar):
                if a is r or a['kaynak'] != 'model':
                    continue
                if t - a['bas_sn'] > GPS_ONCE_SN:
                    break
                sure = (a['bit_sn'] if a['bit_sn'] is not None else t) - a['bas_sn']
                if sure < GPS_BAGLI_SN and not a.get('gps_bagli'):
                    self.gps_bagli_yap(a)
        # Bir GPS alarmı sürerken başka bir belirti de görülebilir (ör. önce konum sıçradı, sonra yükseklik
        # tuttu). Hepsi sırayla saklanır; birden fazlaysa başlık genel olur, rapor hepsini listeler.
        gorulen = r.setdefault('gorulen', [])
        if not any(g['ne'] == jam['kisa'] for g in gorulen):
            gorulen.append({'saat': _hms(saat), 'ne': jam['kisa']})
        baslik = jam['baslik'] if len(gorulen) == 1 else (
            'GPS spoofing şüphesi (birden fazla belirti)' if jam['tur'] == 'spoofing' else jam['baslik'])
        r['tip'], r['etiket'], r['baslik'], r['ne'] = jam['tur'], 'GPS', baslik, jam['kisa']
        r['kanit'] = {'cumle': jam['cumle'], 'saat': _hms(saat), 'uydu': uydu, 'fix': fix, 'cipler': jam.get('cipler'),
                      'oneri': jam['oneri'], 'guven': None, 'guven_kaynak': None, 'sinyaller': [], 'telafi': []}

    def konum_isle(self, t, konum):
        """konum: {'gercek': (enlem, boylam) ya da None, 'gps': ..., 'son_guvenilir': {...} ya da None}"""
        if not konum:
            return
        g, p = konum.get('gercek'), konum.get('gps')
        if g is None and p is None:
            return
        self.son_konum = g or p
        if not self.iz or t - self.iz[-1][0] >= 0.9:
            self.iz.append((round(t, 1), g[0] if g else None, g[1] if g else None,
                            p[0] if p else None, p[1] if p else None))
        r = self.aktif['gps']
        if r is not None:
            if r.get('konum') is None:
                r['konum'] = self.son_konum
            if g and p:
                h = mesafe_m(g, p)
                if r.get('gps_hata_maks') is None or h > r['gps_hata_maks']:
                    r['gps_hata_maks'] = round(h, 1)
            sg = konum.get('son_guvenilir')
            if g and sg:
                r['guvenilir_hata_m'] = round(mesafe_m((sg['enlem'], sg['boylam']), g), 1)
        for r in self.alarmlar:                       # model alarmlarına da başladıkları yeri yaz
            if r.get('konum') is None and r['bit_sn'] is None:
                r['konum'] = self.son_konum

    # --- özet ------------------------------------------------------------------------
    def sure_sn(self):
        return self.son_t

    def ozet(self):
        simdi = self.son_t
        asil = [a for a in self.alarmlar if not a.get('gps_bagli')]     # GPS'e bağlı kısa sapmalar alarm sayılmaz
        araliklar = [(a['bas_sn'], a['bit_sn'] if a['bit_sn'] is not None else simdi) for a in asil]
        sureler = [b - a for a, b in araliklar]
        kapali = [a for a in asil if a['bit_sn'] is not None]
        son = kapali[-1] if kapali else None
        return {
            'numara': self.numara,
            'tarih': self.baslangic.strftime('%Y-%m-%d'),
            'kalkis': _hms(self.baslangic),
            'bitis': _hms(self.bitis) if self.bitis else None,
            'devam': self.bitis is None,
            'sure_sn': int(simdi),
            'alarm_adet': len(asil),
            'kisa_sapma_adet': len(self.alarmlar) - len(asil),
            'alarm_toplam_sn': int(round(_birlesim_sn(araliklar))),
            'alarm_en_uzun_sn': int(round(max(sureler))) if sureler else 0,
            'son_alarm': ({'etiket': son['etiket'], 'sure_sn': int(round(son['bit_sn'] - son['bas_sn'])),
                           'bit': son['bit']} if son else None),
            'irtifa_max': round(self.irtifa_max, 1),
            'batarya_ilk': self.batarya_ilk, 'batarya_son': self.batarya_son,
        }

    def segmentler(self):
        return [{'bas_sn': a['bas_sn'], 'bit_sn': a['bit_sn'], 'etiket': a['etiket'] or 'alarm',
                 'tip': a['tip'], 'devam': a['bit_sn'] is None} for a in self.alarmlar]


class EkranDurumu:
    """Ekranın göstereceği her şeyi tutar; iş parçacığı güvenli."""

    def __init__(self, aciklayici, drone_id='sitl_1', grafana_url=''):
        self.aciklayici = aciklayici
        self.kilit = threading.Lock()
        self.olaylar = []                 # en yeni başta
        self.alarm_olayi = None           # sürmekte olan model alarmının olay kaydı
        self.alarm_olayi_skor = None      # olay satırındaki açıklamanın skoru (yoksa henüz net açıklama yok)
        self.jam_olayi = None             # sürmekte olan jamming olayı
        self.alarm_baslangic = None
        self.uyari_verildi = False
        self.gps_son = None               # GPS alarmının (jamming/spoofing) en son görüldüğü an
        self.model_bas = None             # sürmekte olan model alarmının başladığı an
        self.model_gps_bagli = False      # o alarm GPS olayına bağlı kısa sapma mı (ilk 5 sn)
        self.ucuslar = []                 # en yeni başta
        self.ucus_sayaci = 0
        self.drone_id = drone_id
        self.durum = {'drone_id': drone_id, 'grafana_url': grafana_url, 'baglanti': False,
                      'saat': None, 'durum': 'baglanti_yok', 'baslik': 'Bağlantı yok',
                      'cumle': 'İHA verisi gelmiyor.', 'olaylar': [], 'ucus': None,
                      'oneri': None, 'min_irtifa': MIN_IRTIFA_M, 'bekleme_sn': BEKLEME_SN}

    # --- exporter'dan çağrılanlar -------------------------------------------------------
    def baglanti_yok(self, saat):
        with self.kilit:
            self.durum.update({'baglanti': False, 'saat': _hms(saat),
                               'durum': 'baglanti_yok', 'baslik': 'Bağlantı yok',
                               'cumle': 'İHA verisi gelmiyor. SITL açık mı?',
                               'aciklama': None, 'skor': None, 'alarm_suresi_sn': None, 'oneri': None})

    def guncelle(self, saat, son, mod, sebep, skor, alarm, jam_aktif, uydu, fix, tur_kural=None, spoof=None, konum=None):
        """
        saat     : datetime (olay saatleri için)
        son      : exporter'daki "defter" ('armed','irtifa','voltaj','akim','m1'..'m4'; isteğe bağlı
                   'batarya' (kalan %) ve 'hiz' (yer hızı m/s))
        mod      : uçuş modu metni
        sebep    : model karar vermiyorsa nedeni (ör. 'irtifa dusuk'), verdiyse None
        skor     : mesafe/eşik oranı (karar yoksa None)
        alarm    : model alarmı
        jam_aktif: GPS jamming kuralı alarmda mı
        uydu,fix : son GPS değerleri (jamming açıklaması için)
        tur_kural: exporter'daki dedektor.tur() sonucu ('batarya'/'motor1'..'motor4'); alarm yoksa None
        konum    : rapordaki yol haritası için {'gercek','gps','son_guvenilir'} (harita.HaritaDurumu.konum_ozeti())
        spoof    : GPS spoofing alarmı varsa spoof_tespit.spoofing_aciklama() sözlüğü, yoksa None.
                   Ekranda jamming ile aynı "GPS alarmı" yerini kullanır; ikisi birden varsa jamming gösterilir.
        """
        aciklama = None
        if sebep is None and skor is not None:
            x = [son['m1'], son['m2'], son['m3'], son['m4'], son['voltaj'], son['akim']]
            aciklama = self.aciklayici.acikla(mod, x, alarm, skor, tur_kural)
            # Ekrandaki neden exporter'ın tür kuralından geliyor; kendi kuralımız ayrışıyorsa log'a düş
            if alarm and aciklama is not None and aciklama['uyumsuz'] and not self.uyari_verildi:
                self.uyari_verildi = True
                print(f"{saat:%H:%M:%S} UYARI: tür kuralı '{tur_kural}' dedi, ekran açıklama kuralı "
                      f"'{aciklama['tur_kendi']}' diyor. İkisi ayrışıyor.", flush=True)
            if not alarm:
                self.uyari_verildi = False
        jam = jamming_aciklama(uydu, fix) if jam_aktif else spoof
        jam_aktif = jam is not None
        s = _hms(saat)

        with self.kilit:
            # GPS olayına bağlı kısa model sapması mı?
            gps_yeni = jam_aktif and (self.gps_son is None or (saat - self.gps_son).total_seconds() > GPS_BAGLI_SN)
            if jam_aktif:
                self.gps_son = saat
            if gps_yeni:
                # GPS alarmı az önce başladı: ondan hemen önce başlamış kısa model sapması da bu olayın parçası
                if self.model_bas is not None and (saat - self.model_bas).total_seconds() <= GPS_ONCE_SN:
                    self.model_gps_bagli = True
                    if self.alarm_olayi is not None:
                        self.alarm_olayi.update({'tur': GPS_BAGLI['baslik'], 'ne': GPS_BAGLI['kisa'], 'gps_bagli': True})
                for o in self.olaylar[:4]:          # az önce bitmiş olanı da düzelt
                    if (o.get('model') and not o.get('gps_bagli') and o.get('t') is not None
                            and (saat - o['t']).total_seconds() <= GPS_ONCE_SN + 1.5):
                        o.update({'tur': GPS_BAGLI['baslik'], 'ne': GPS_BAGLI['kisa'], 'gps_bagli': True})
            model_alarm = alarm and aciklama is not None
            if model_alarm:
                if self.model_bas is None:
                    self.model_bas = saat
                    self.model_gps_bagli = (self.gps_son is not None
                                            and (saat - self.gps_son).total_seconds() <= GPS_BAGLI_SN)
                elif self.model_gps_bagli and (saat - self.model_bas).total_seconds() >= GPS_BAGLI_SN:
                    self.model_gps_bagli = False
            else:
                self.model_bas, self.model_gps_bagli = None, False
            gecici = model_alarm and self.model_gps_bagli

            self._olaylari_isle(s, alarm, aciklama, jam_aktif, jam, gecici, saat)
            self._ucus_isle(saat, son, alarm, skor, aciklama, jam_aktif, jam, uydu, fix, gecici, konum)
            dikkat = {'baslik': GPS_BAGLI['baslik'], 'kisa': GPS_BAGLI['kisa']} if gecici else None
            if model_alarm and not gecici:
                durum, baslik, cumle = 'alarm', aciklama['baslik'], aciklama['cumle']
                oneri = aciklama['oneri']
                if self.alarm_baslangic is None:
                    self.alarm_baslangic = self.model_bas or saat
            elif jam_aktif:
                durum, baslik, cumle, oneri = 'jamming', jam['baslik'], jam['cumle'], jam['oneri']
            elif gecici:
                durum, baslik, cumle, oneri = 'dikkat', GPS_BAGLI['baslik'], GPS_BAGLI['cumle'], GPS_BAGLI['oneri']
            elif sebep == 'motorlar kapali':
                # Yerde: bu bir arıza ya da belirsizlik değil, model zaten yalnızca uçuşta karar verir
                durum, baslik = 'izlenemiyor', 'Yerde, motorlar kapalı'
                cumle = 'İzleme, motorlar çalışıp drone havalandıktan sonra başlar.'
                oneri = {'metin': 'İzleme uçuşta başlar.',
                         'alt': 'Koşullar sağlanınca model kendiliğinden devreye girer.'}
            elif sebep is not None:
                durum, baslik = 'izlenemiyor', 'Sistem şu an karar veremiyor'
                cumle = SEBEP_METNI.get(sebep, f"Sebep: {sebep}.")
                oneri = {'metin': 'Bu "güvenli" demek değildir.',
                         'alt': 'Koşullar sağlanınca izleme kendiliğinden başlar.'}
            else:
                durum, baslik, cumle = 'normal', aciklama['baslik'], aciklama['cumle']
                oneri = {'metin': 'Bir şey yapmana gerek yok.',
                         'alt': 'Koşullar bozulursa ekran kendiliğinden uyarır.'}
            if not alarm or gecici:
                self.alarm_baslangic = None
            sure = int((saat - self.alarm_baslangic).total_seconds()) if self.alarm_baslangic else None
            u = self.ucuslar[0] if self.ucuslar else None
            self.durum.update({
                'baglanti': True, 'saat': s, 'durum': durum, 'baslik': baslik, 'cumle': cumle,
                'mod': mod, 'irtifa': son.get('irtifa'), 'voltaj': son.get('voltaj'), 'akim': son.get('akim'),
                'batarya': son.get('batarya'), 'hiz': son.get('hiz'), 'armed': bool(son.get('armed')),
                'motorlar': [son.get('m1'), son.get('m2'), son.get('m3'), son.get('m4')],
                'uydu': uydu, 'fix': fix, 'sebep': sebep, 'skor': skor,
                'aciklama': aciklama, 'jamming': jam, 'alarm_suresi_sn': sure, 'oneri': oneri,
                'dikkat': dikkat,
                'rejim': aciklama['rejim'] if aciklama else None,
                'olaylar': [{k: v for k, v in o.items() if k != 't'} for o in self.olaylar],
                'ucus': (dict(u.ozet(), segmentler=u.segmentler()) if u else None),
            })

    # --- uçuş kaydı ---------------------------------------------------------------------
    def _ucus_isle(self, saat, son, alarm, skor, aciklama, jam_aktif, jam, uydu, fix, gecici=False, konum=None):
        armed = bool(son.get('armed'))
        u = self.ucuslar[0] if self.ucuslar and self.ucuslar[0].bitis is None else None
        if u is None:
            if not armed:
                return
            self.ucus_sayaci += 1
            u = Ucus(saat, self.ucus_sayaci)
            self.ucuslar.insert(0, u)
            del self.ucuslar[MAKS_UCUS:]
        t = u.t(saat)
        if not armed:                                   # motorlar durdu: uçuş bitti
            for k in ('model', 'gps'):
                u.alarm_kapat(k, saat, t)
            u.bitis = saat
            if u.irtifa_max < GERCEK_UCUS_M:            # havalanmadı, sadece arm edildi: kaydı at
                self.ucuslar.remove(u)
                self.ucus_sayaci -= 1
            return
        irt = son.get('irtifa')
        if irt is not None and irt > u.irtifa_max:
            u.irtifa_max = irt
        b = son.get('batarya')
        if b is not None:
            if u.batarya_ilk is None:
                u.batarya_ilk = b
            u.batarya_son = b
        u.seri.append((round(t, 1), None if skor is None else round(skor, 3)))
        u.konum_isle(t, konum)
        if alarm and aciklama is not None:
            u.model_alarm_isle(saat, t, aciklama, skor, gecici)
        else:
            u.alarm_kapat('model', saat, t)
        if jam_aktif and jam is not None:
            u.gps_alarm_isle(saat, t, jam, uydu, fix)
        else:
            u.alarm_kapat('gps', saat, t)

    def _olaylari_isle(self, s, alarm, aciklama, jam_aktif, jam, gecici=False, saat=None):
        # model alarmı: başlangıçta bir olay satırı aç, bitince kapat
        if alarm and aciklama is not None and self.alarm_olayi is None:
            if gecici:
                self.alarm_olayi = {'saat': s, 'tur': GPS_BAGLI['baslik'], 'ne': GPS_BAGLI['kisa'],
                                    'durum': 'devam', 'bitis': None, 'gps_bagli': True}
                self.alarm_olayi_skor = None
            else:
                self.alarm_olayi = {'saat': s, 'tur': aciklama['baslik'], 'ne': aciklama['kisa'],
                                    'durum': 'devam', 'bitis': None, 'model': True}
                self.alarm_olayi_skor = aciklama['skor'] if aciklama['tur'] != 'belirsiz' else None
            self.alarm_olayi['t'] = saat
            self._ekle(self.alarm_olayi)
        elif alarm and aciklama is not None and self.alarm_olayi is not None and gecici:
            pass                                       # GPS'e bağlı kısa sapma sürüyor: satır aynı kalır
        elif alarm and aciklama is not None and self.alarm_olayi is not None and self.alarm_olayi.get('gps_bagli'):
            # 5 sn'yi aştı: artık gerçek arıza alarmı olarak göster
            self.alarm_olayi.update({'tur': aciklama['baslik'], 'ne': aciklama['kisa'], 'gps_bagli': False})
            self.alarm_olayi_skor = aciklama['skor'] if aciklama['tur'] != 'belirsiz' else None
        elif alarm and aciklama is not None and self.alarm_olayi is not None:
            # Alarmın ilk ve son saniyelerinde değerler henüz oturmamış / normale dönüyor olabilir;
            # o anların açıklaması güvenilmez. Olay satırı, alarm boyunca NET açıklama verilen
            # en yüksek skorlu anı gösterir (en büyük sapma, arızanın kendisidir).
            if aciklama['tur'] != 'belirsiz' and (self.alarm_olayi_skor is None
                                                  or aciklama['skor'] > self.alarm_olayi_skor):
                self.alarm_olayi['tur'] = aciklama['baslik']
                self.alarm_olayi['ne'] = aciklama['kisa']
                self.alarm_olayi_skor = aciklama['skor']
        elif not alarm and self.alarm_olayi is not None:
            self.alarm_olayi['durum'] = 'bitti'
            self.alarm_olayi['bitis'] = s
            self.alarm_olayi = None
            self.alarm_olayi_skor = None
        # jamming
        if jam_aktif and self.jam_olayi is None:
            self.jam_olayi = {'saat': s, 'tur': jam['baslik'], 'ne': jam['kisa'], 'durum': 'devam', 'bitis': None}
            self._ekle(self.jam_olayi)
        elif not jam_aktif and self.jam_olayi is not None:
            self.jam_olayi['durum'] = 'bitti'
            self.jam_olayi['bitis'] = s
            self.jam_olayi = None

    def _ekle(self, olay):
        self.olaylar.insert(0, olay)
        del self.olaylar[MAKS_OLAY:]

    # --- JSON ---------------------------------------------------------------------------
    def json(self):
        with self.kilit:
            return _dump(self.durum)

    def gecmis(self, n=0):
        """Rapor verisi: n=0 son uçuş, n=1 bir önceki... Uçuş yoksa 'ucus' None."""
        with self.kilit:
            adet = len(self.ucuslar)
            if not (0 <= n < adet):
                return _dump({'adet': adet, 'n': n, 'drone_id': self.drone_id, 'ucus': None})
            u = self.ucuslar[n]
            ucus = dict(u.ozet(), alarmlar=[{k: v for k, v in a.items() if k not in ('kanit_skor', 'kanit_net')}
                                            for a in u.alarmlar],
                        seri=[[t, v] for t, v in u.seri], min_irtifa=MIN_IRTIFA_M,
                        iz=[list(p) for p in u.iz], simulasyon=any(p[1] is not None for p in u.iz))
            return _dump({'adet': adet, 'n': n, 'drone_id': self.drone_id, 'ucus': ucus})


def _dump(o):
    return json.dumps(o, ensure_ascii=False, allow_nan=False, default=_temiz).encode('utf-8')


def _temiz(o):
    raise TypeError(f"JSON'a çevrilemedi: {type(o)}")


SEBEP_METNI = {
    'motorlar kapali': 'Sebep: motorlar kapalı.',
    'irtifa dusuk': f'Sebep: irtifa düşük. Model {MIN_IRTIFA_M:g} m üstünde ve {BEKLEME_SN:g} saniye sabit uçuştan sonra devreye girer.',
    'mod taninmiyor': 'Sebep: uçuş modu modelde yok. Model sadece GUIDED, LOITER ve CIRCLE modlarında karar verir.',
    'gecis bekleniyor': f'Sebep: mod ya da irtifa yeni değişti. Model {BEKLEME_SN:g} saniye sabit uçuş bekliyor.',
    'olcum eksik': 'Sebep: bir ölçüm henüz gelmedi.',
}


def sunucu_baslat(durum, port=None, host=None, harita=None):
    """Sunucuyu arka planda başlatır; (sunucu, port) döner."""
    port = int(port or os.environ.get('EKRAN_PORT', 8088))
    host = host or os.environ.get('EKRAN_HOST', '127.0.0.1')

    class Isleyici(BaseHTTPRequestHandler):
        def do_GET(self):
            url = urlparse(self.path)
            yol = url.path
            if yol == '/durum':
                self._gonder(200, 'application/json; charset=utf-8', durum.json())
            elif yol == '/gecmis':
                try:
                    n = int(parse_qs(url.query).get('n', ['0'])[0])
                except ValueError:
                    n = 0
                self._gonder(200, 'application/json; charset=utf-8', durum.gecmis(n))
            elif yol in ('/', '/index.html'):
                self._dosya(SAYFA_YOLU, 'ekran_sayfa.html')
            elif yol in ('/rapor', '/rapor.html'):
                self._dosya(RAPOR_YOLU, 'ekran_rapor.html')
            elif yol in ('/harita', '/harita.html'):
                self._dosya(HARITA_YOLU, 'harita_sayfa.html')
            elif yol == '/harita/veri' and harita is not None:
                self._gonder(200, 'application/json; charset=utf-8', harita.json())
            elif yol == '/harita/anahtar':
                # Cesium anahtarı sadece bu bilgisayardan açılan sayfaya verilir (sunucu 127.0.0.1'de dinler).
                from harita import cesium_anahtari
                a = cesium_anahtari()
                govde = json.dumps({'anahtar': a}).encode('utf-8')
                self._gonder(200 if a else 404, 'application/json; charset=utf-8', govde)
            else:
                self._gonder(404, 'text/plain; charset=utf-8', b'yok')

        def _dosya(self, yol, ad):
            try:
                with open(yol, 'rb') as f:
                    self._gonder(200, 'text/html; charset=utf-8', f.read())
            except OSError:
                self._gonder(500, 'text/plain; charset=utf-8', f'{ad} bulunamadı'.encode('utf-8'))

        def _gonder(self, kod, tip, govde):
            self.send_response(kod)
            self.send_header('Content-Type', tip)
            self.send_header('Content-Length', str(len(govde)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(govde)

        def log_message(self, *a):
            pass

    srv = ThreadingHTTPServer((host, port), Isleyici)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, port