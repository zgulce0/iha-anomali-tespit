"""
GPS spoofing tespiti: kendi Kalman filtremiz (dogrusal, 6 durumlu) + NIS (chi-kare) testleri.

Bu dosya hem canli sistemde (exporter.py) hem cevrimdisi analizde (analiz/spoof_analiz.py)
kullanilir; iki yerde ayni kod calisir.

Not: Bu bir EKF degil, duz (dogrusal) Kalman filtresi. Model dogrusal oldugu icin "genisletilmis"
surume gerek yok: IMU ivmesi govde ekseninden kuzey/dogu eksenine filtreden ONCE cevrilir.
ArduPilot'un kendi filtresi ise EKF'dir (EKF3).

Durum x = [kuzey, dogu, v_kuzey, v_dogu, b_kuzey, b_dogu]  (metre, m/s, metre)
  b: GPS'in kendi konum hatasi. Gercek bir GPS alicisi birkac metre YAVASCA gezinir (NEO-M8N
     veri sayfasi: 2,0-2,5 m CEP). Bu hata ayri bir durum olarak modellenir (birinci derece
     Gauss-Markov: std SIG_B, zaman sabiti TAU_B). Boylece filtre bu dogal gezinmeyi saldiri sanmaz.
  Tahmin   : IMU ivmesi ile konum/hiz; b yavasca sifira doner.
  Duzeltme : once GPS hizi (olcum = v), sonra GPS konumu (olcum = konum + b).
Yenilik nu = olcum - tahmin,  S = H P H' + R,  NIS = nu' S^-1 nu  (2 serbestlik derecesi).

Uc test (herhangi biri alarm acar, alarm kilit_sn saniye acik kalir):
  hiz   : GPS HIZ yeniligi NIS_v > ESIK_V, art arda 2 olcum. GPS hizi bir anda degisip ivmeolcer
          bunu gormezse yakalar (konum ve hizi birlikte kaydiran saldiri). Tek olcumluk celiski
          (ornegin agir motor arizasinin ilk sarsintisi) alarm acmaz.
  anlik : GPS KONUM yeniligi NIS > 9.21, art arda 3 olcum. Ani sicrama.
  yavas : kestirilen GPS hatasi b, alicinin kendi hatasi icin beklenenden buyuk:
          |b|^2 / SIG_B^2 > ESIK_B  (|b| > ~7 m). Yavas kayma b'de birikir.

Esikler ve neden:
  9.21  : chi-kare (2 sd) %99. Art arda 3 olcum sarti tek olcumluk gurultuyu eler.
  ESIK_V: chi-kare (2 sd) %99.9 = 13.82, art arda 2 olcum: olcum basina yanlis alarm olasiligi
          ~1e-6 (filtre kalibreyse; 5 Hz'de saatte ~0,02).
  ESIK_B: chi-kare (2 sd) %99.99 = 18.42.

Kalibrasyon (analiz/spoof_kalibrasyon.py): simulatorun GPS'i hatasizdir, bu yuzden kayitli ucuslara
gercekci GPS hatasi (NEO-M8N: 2,0 m CEP -> eksen basina ~1,7 m, yavas gezinme; hiz 0,05 m/s)
eklenerek denendi. Normal ucusta hiz NIS ortalamasi ~2,05 (teori: 2) cikti, yani hiz testi
chi-kare varsayimina uyuyor. Gezinmenin hizi (zaman sabiti) bilinmiyor; 30, 100 ve 300 sn denendi.
Onceki surum (4 durum, pencere ortalamasi + CUSUM) hatasiz GPS'te cok hizliydi ama gercekci GPS
hatasiyla saatte 40-70 yanlis alarm verdi; bu yuzden degistirildi.
"""
import math
import time
from collections import deque

import numpy as np

G = 9.80665
M_DERECE = 111320.0          # 1 derece enlem ~ 111.32 km
CHI2_99_2SD = 9.21
ESIK_V = 13.82
SIG_A_SEYREK = 1.0           # iki GPS olcumu arasinda ivme ornegi azsa tahmin bu kadar belirsiz sayilir (m/s2)
ESIK_B = 18.42


class SpoofDedektor:
    def __init__(self, sig_a=0.1, sig_w=0.15, sig_v=0.05, sig_b=1.7, tau_b=300.0,
                 esik_v=ESIK_V, esik_b=ESIK_B, sifirla_sn=0.8, anlik_n=3, hiz_n=2, kilit_sn=10.0, k_a=0.15):
        """sig_a: IMU ivme belirsizligi (m/s2), sig_w: GPS konum titresimi (m), sig_v: GPS hiz gurultusu (m/s),
        sig_b / tau_b: GPS'in yavas gezinen hatasi (m / sn)."""
        self.sig_a, self.sig_b, self.tau_b, self.k_a = sig_a, sig_b, tau_b, k_a
        self.R_p = np.eye(2) * sig_w ** 2
        self.R_v = np.eye(2) * sig_v ** 2
        self.Hp = np.zeros((2, 6)); self.Hp[0, 0] = self.Hp[1, 1] = self.Hp[0, 4] = self.Hp[1, 5] = 1.0
        self.Hv = np.zeros((2, 6)); self.Hv[0, 2] = self.Hv[1, 3] = 1.0
        self.esik_v, self.esik_b = esik_v, esik_b
        self.sifirla_sn, self.anlik_n, self.hiz_n, self.kilit_sn = sifirla_sn, anlik_n, hiz_n, kilit_sn
        self.asim_v = 0
        self.sapma_siniri = math.sqrt(esik_b) * sig_b     # metre (~7,3)
        self.x = None
        self.t = None
        self.kilit_bitis = -1.0
        self.son_tetik = None
        self.tepe = None             # alarm boyunca testlerin en yuksek degerleri (ekranda kanit olarak)
        self.nis_onceki = 0.0
        self.son_sicrama = None      # konum NIS'inin bir adimda sinirin cok ustune firladigi son an
        self.son_asim = None         # konum NIS'inin sinirin ustunde oldugu son an
        self.asim, self.asim_bas = 0, None

    def _baslat(self, zp, zv):
        self.x = np.array([zp[0], zp[1], zv[0], zv[1], 0.0, 0.0])
        self.P = np.diag([self.sig_b ** 2] * 2 + [self.R_v[0, 0]] * 2 + [self.sig_b ** 2] * 2)
        self.asim, self.asim_bas = 0, None
        self.asim_v = 0
        self.nis_onceki = 0.0

    def sifirla(self):
        """Izleme kesilince (yere inis, GPS kaybi) cagrilir: filtre bir sonraki olcumde bastan baslar.
        Acik alarm kilidi korunur."""
        self.x = None

    def _guncelle(self, z, H, R):
        nu = z - H @ self.x
        S = H @ self.P @ H.T + R
        nis = float(nu @ np.linalg.solve(S, nu))
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ nu
        self.P = (np.eye(6) - K @ H) @ self.P
        return nis

    def adim(self, t, zp, zv, a_ne, imu_seyrek=False, ivme_std=0.0):
        """t: sn, zp: GPS konum (kuzey, dogu) m, zv: GPS hiz (kuzey, dogu) m/s,
        a_ne: IMU ivmesi (kuzey, dogu) m/s2. imu_seyrek: bu aralikta ivme ornegi az geldi (tahmin
        daha belirsiz sayilir). ivme_std: bu araliktaki ivme orneklerinin standart sapmasi (m/s2).
        Manevra sirasinda IMU tahmini daha belirsizdir (aci ve zamanlama hatasi ivmeyle buyur); bu yuzden
        ivme belirsizligi sig_a^2 + (k_a*|a|)^2 + ivme_std^2 alinir. Duz ucusta (ivme ~0) sig_a kalir; spoofing
        ivme yaratmadigi icin tespit hassasiyeti duz ucusta degismez.
        Sonuc sozlugu dondurur."""
        zp = np.asarray(zp, dtype=float); zv = np.asarray(zv, dtype=float); a_ne = np.asarray(a_ne, dtype=float)
        if self.x is None:
            self.t = t
            self._baslat(zp, zv)
            return self._sonuc(t, 0.0, 0.0, 0.0, False, False, False, True)
        dt = t - self.t
        if dt <= 1e-3:                       # ayni olcum tekrar geldi
            return self._sonuc(t, 0.0, 0.0, self._sapma(), False, False, False, False)
        self.t = t
        phi = math.exp(-dt / self.tau_b)
        F = np.eye(6); F[0, 2] = F[1, 3] = dt; F[4, 4] = F[5, 5] = phi
        B = np.zeros((6, 2)); B[0, 0] = B[1, 1] = dt * dt / 2; B[2, 0] = B[3, 1] = dt
        if imu_seyrek:
            sig_a2 = SIG_A_SEYREK ** 2
        else:
            sig_a2 = self.sig_a ** 2 + (self.k_a * float(np.hypot(*a_ne))) ** 2 + ivme_std ** 2
        Q = B @ B.T * sig_a2
        Q[4, 4] = Q[5, 5] = self.sig_b ** 2 * (1 - phi * phi)
        self.x = F @ self.x + B @ a_ne
        self.P = F @ self.P @ F.T + Q
        nis_v = self._guncelle(zv, self.Hv, self.R_v)     # test: hiz
        nis = self._guncelle(zp, self.Hp, self.R_p)       # test: anlik
        # Sicrama mi kayma mi? Sicramada NIS, en az 1 sn sakin gectikten sonra tek adimda sinirin ustune
        # firlar (onceki degerin en az 4 kati). Hizli kaymada ise sinir sik sik asilir, sakin donem olmaz.
        # Bu ayrim sadece ekrandaki aciklama icindir, alarm kararini degistirmez.
        if nis > CHI2_99_2SD:
            sakin = self.son_asim is None or t - self.son_asim > 1.0
            if sakin and nis >= 4 * max(self.nis_onceki, 1.0):
                self.son_sicrama = t
            self.son_asim = t
        self.nis_onceki = nis
        if nis > CHI2_99_2SD:
            self.asim += 1
            if self.asim_bas is None:
                self.asim_bas = t
        else:
            self.asim, self.asim_bas = 0, None
        anlik = self.asim >= self.anlik_n            # tek bir gurultu sicramasi degil, art arda 3 olcum
        # Ivme verisi seyrekse hiz testi karar vermez: IMU'nun gormedigi bir sarsinti saldiri sanilmasin.
        self.asim_v = self.asim_v + 1 if (nis_v > self.esik_v and not imu_seyrek) else 0
        hiz = self.asim_v >= self.hiz_n
        sapma = self._sapma()
        yavas = (sapma / self.sig_b) ** 2 > self.esik_b
        sifirlandi = False
        if self.asim_bas is not None and t - self.asim_bas >= self.sifirla_sn - 1e-6:
            self._baslat(zp, zv)                     # 0,8 sn surekli buyuk sicrama: yeni konuma yeniden basla
            sifirlandi = True
        return self._sonuc(t, nis, nis_v, sapma, anlik, yavas, hiz, sifirlandi)

    def _sapma(self):
        """Kestirilen GPS hatasinin buyuklugu (metre)."""
        return 0.0 if self.x is None else float(math.hypot(self.x[4], self.x[5]))

    def _sonuc(self, t, nis, nis_v, sapma, anlik, yavas, hiz, sifirlandi):
        if anlik or yavas or hiz:
            yeni = t > self.kilit_bitis              # onceki alarm bitmisti: yeni bir olay
            self.kilit_bitis = t + self.kilit_sn
            # Ne oldu? Son 1 sn icinde ani bir sicrama goruldiyse 'anlik', yoksa konum adim adim kaymistir.
            sicrama = self.son_sicrama is not None and t - self.son_sicrama <= 1.0
            tetik = 'hiz' if hiz else ('anlik' if sicrama else 'yavas')
            if yeni or self.tepe is None:
                self.son_tetik = tetik
                self.tepe = dict(nis=nis, nis_v=nis_v, sapma=sapma)
        alarm = t <= self.kilit_bitis
        if alarm and self.tepe is not None:
            self.tepe = dict(nis=max(self.tepe['nis'], nis), nis_v=max(self.tepe['nis_v'], nis_v),
                             sapma=max(self.tepe['sapma'], sapma))
        return dict(nis=nis, nis_v=nis_v, sapma=sapma, anlik=anlik, yavas=yavas, hiz=hiz,
                    alarm=alarm, tetik=self.son_tetik if alarm else None,
                    tepe=dict(self.tepe) if (alarm and self.tepe) else None, sifirlandi=sifirlandi)


def govde_to_kd(ax_mg, ay_mg, az_mg, roll, pitch, yaw):
    """Govde eksenindeki ivme (milli-g) ve acilar (radyan) -> kuzey/dogu ivmesi (m/s2)."""
    fx, fy, fz = ax_mg * G / 1000, ay_mg * G / 1000, az_mg * G / 1000
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    an = cy * cp * fx + (cy * sp * sr - sy * cr) * fy + (cy * sp * cr + sy * sr) * fz
    ae = sy * cp * fx + (sy * sp * sr + cy * cr) * fy + (sy * sp * cr - cy * sr) * fz
    return an, ae


class SpoofIzleyici:
    """Canli sistem icin: MAVLink mesajlarini toplar, her yeni GPS olcumunde dedektoru bir adim calistirir.

    Izleme kosulu (kapi): motorlar calisiyor, irtifa >= MIN_IRTIFA_M, GPS saglikli (3D fix, >= 6 uydu)
    ve jamming alarmi yok. Kosul bozulunca filtre sifirlanir, kosul gelince bastan baslar."""
    MIN_IRTIFA_M = 2.0
    MIN_IMU = 3              # iki GPS olcumu arasinda en az bu kadar ivme ornegi (50 Hz'de ~10 gelir)
    GPS_GECIKME_SN = 0.1     # GPS olcumu ~100 ms gecikmeli gelir (SITL: SIM_GPS1_LAG_MS=100; u-blox benzer).
                             # Ivme de ayni kadar geriden alinir; yoksa manevra anlarinda IMU ile GPS celisir.

    def __init__(self, kilit_sn=30.0):
        self.det = SpoofDedektor(kilit_sn=kilit_sn)
        self.ref = None                  # (enlem0, boylam0, cos(enlem0)): yerel koordinat merkezi
        self.aci = None                  # (roll, pitch, yaw) radyan
        self.ivme = None                 # (kuzey, dogu) m/s2, iki GPS olcumu arasindaki ivmelerin ortalamasi
        self._ivmeler = deque()          # (saat, kuzey, dogu): bilgisayar saatiyle damgali ivme ornekleri
        self._son_gps_saat = None
        self.imu_sayisi = 0              # son GPS araligindaki ivme ornegi sayisi
        self.ivme_std = 0.0              # o orneklerin standart sapmasi (m/s2)
        self.son = None                  # son adimin sonucu
        self.aktif = False

    def attitude(self, roll, pitch, yaw):
        self.aci = (roll, pitch, yaw)

    def raw_imu(self, xacc, yacc, zacc, saat=None):
        if self.aci is not None:
            an, ae = govde_to_kd(xacc, yacc, zacc, *self.aci)
            self._ivmeler.append((time.monotonic() if saat is None else saat, an, ae))
            while len(self._ivmeler) > 500:
                self._ivmeler.popleft()

    def _ivme_al(self, saat):
        """Bir onceki GPS olcumu ile bu olcum arasindaki ivmelerin ortalamasi, GPS gecikmesi kadar geriden.
        Tek ornek yerine ortalama: tek ornek titresimden etkilenir."""
        bit = saat - self.GPS_GECIKME_SN
        bas = (self._son_gps_saat - self.GPS_GECIKME_SN) if self._son_gps_saat is not None else bit - 1.0
        self._son_gps_saat = saat
        sec = [(a, b) for s_, a, b in self._ivmeler if bas < s_ <= bit]
        self.imu_sayisi = len(sec)
        self.ivme_std = 0.0
        while self._ivmeler and self._ivmeler[0][0] <= bit:
            self._ivmeler.popleft()
        if sec:
            self.ivme = (sum(a for a, _ in sec) / len(sec), sum(b for _, b in sec) / len(sec))
            if len(sec) > 1:
                self.ivme_std = math.sqrt(sum((a - self.ivme[0]) ** 2 + (b - self.ivme[1]) ** 2 for a, b in sec)
                                          / (len(sec) - 1) / 2)
        return self.ivme if self.ivme is not None else (0.0, 0.0)

    def gps(self, t, lat_e7, lon_e7, vel_cms, cog_cdeg, kapi_acik, saat=None):
        """GPS_RAW_INT geldiginde cagrilir. Sonuc sozlugu (ya da kapi kapaliysa None) dondurur.
        t: GPS'in kendi zaman damgasi (sn); saat: mesajin geldigi an (bilgisayar saati, ivme eslemesi icin)."""
        saat = time.monotonic() if saat is None else saat
        if not kapi_acik or vel_cms == 65535:
            if self.aktif:
                self.det.sifirla()
            self.aktif = False
            self.son = None
            return None
        lat, lon = lat_e7 / 1e7, lon_e7 / 1e7
        if self.ref is None:
            self.ref = (lat, lon, math.cos(math.radians(lat)))
        n = (lat - self.ref[0]) * M_DERECE
        e = (lon - self.ref[1]) * M_DERECE * self.ref[2]
        v = vel_cms / 100.0
        rota = math.radians(cog_cdeg / 100.0) if cog_cdeg != 65535 else 0.0
        zv = (v * math.cos(rota), v * math.sin(rota))
        self.aktif = True
        ivme = self._ivme_al(saat)
        self.son = self.det.adim(t, (n, e), zv, ivme, imu_seyrek=self.imu_sayisi < self.MIN_IMU,
                                 ivme_std=self.ivme_std)
        return self.son

    def tahmin_konum(self):
        """Filtrenin aracın GERÇEK konumu için tahmini (enlem, boylam): GPS'ten, filtrenin kestirdiği GPS
        hatası çıkarılmış hali. Gerçek bir araçta da hesaplanabilir. Filtre çalışmıyorsa None."""
        x = getattr(self.det, 'x', None)
        if not self.aktif or x is None or self.ref is None:
            return None
        return (self.ref[0] + x[0] / M_DERECE, self.ref[1] + x[1] / (M_DERECE * self.ref[2]))

    def alarm(self, t):
        """Kilit suresi dolmadiysa True (izleme kesilse bile acik alarm kilit suresince surer)."""
        return t <= self.det.kilit_bitis


TETIK_METNI = {
    'hiz': 'GPS hızı bir anda değişti ama ivmeölçer bunu görmedi.',
    'anlik': 'GPS konumu bir anda sıçradı; ivmeölçer böyle bir hareket görmedi.',
    'yavas': 'GPS konumu yavaşça kaydı; kayma, bir GPS alıcısının kendi hatasından beklenenden büyük ve hız ile ivme bunu açıklamıyor.',
}


SAPMA_SINIRI = SpoofDedektor().sapma_siniri


def _tr(x, basamak):
    """Sayiyi Turkce yazar: 12.7 -> '12,7'."""
    return f'{x:.{basamak}f}'.replace('.', ',')


def spoofing_aciklama(sonuc):
    """Operatör ekranı için metin (aciklama.jamming_aciklama ile aynı yapı).
    Değerler, alarm boyunca görülen EN YÜKSEK değerlerdir: filtre sıçramadan sonra yeni konuma
    oturunca anlık değerler sıfıra iner, kanıt olarak onlar gösterilirse anlamsız olur."""
    sonuc = sonuc or {}
    tetik = sonuc.get('tetik') or 'yavas'
    tepe = sonuc.get('tepe') or sonuc
    sapma = tepe.get('sapma') or 0.0
    nis = tepe.get('nis') or 0.0
    nis_v = tepe.get('nis_v') or 0.0
    return {'tur': 'spoofing', 'baslik': 'GPS spoofing şüphesi',
            'cumle': TETIK_METNI[tetik] + ' Olası neden: sahte GPS sinyali.',
            'kisa': {'hiz': 'GPS hızı ile ivmeölçer çelişiyor', 'anlik': 'GPS konumu sıçradı',
                     'yavas': 'GPS konumu sürekli kayıyor'}[tetik],
            'oneri': {'metin': 'GPS konumuna güvenme, araç sandığı yerde olmayabilir.',
                      'alt': 'Konumu başka bir kaynakla (görüş, kamera) doğrula.'},
            'anons': 'GPS spoofing şüphesi. Konuma güvenme.',
            'cipler': [['TEST', {'hiz': 'hız yeniliği', 'anlik': 'ani sıçrama', 'yavas': 'yavaş kayma'}[tetik]],
                       ['EN YÜKSEK KONUM NIS', f'{_tr(nis, 1)} (sınır 9,21)'],
                       ['EN YÜKSEK HIZ NIS', f'{_tr(nis_v, 1)} (sınır {_tr(ESIK_V, 1)})'],
                       ['EN YÜKSEK GPS SAPMASI', f'{_tr(sapma, 1)} m (sınır {_tr(SAPMA_SINIRI, 1)})']]}
