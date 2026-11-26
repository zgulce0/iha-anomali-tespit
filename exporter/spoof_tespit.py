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

Yukseklik (irtifa) spoofing'i: IrtifaDedektor (asagida). GPS'in bildirdigi yukseklik, barometre ve IMU'nun
dikey ivmesiyle karsilastirilir. Ayni yontemin dikey hali: durum [h, v_yukari, b_z], olcumler barometre (h)
ve GPS yuksekligi (h + b_z). b_z, GPS ile barometrenin yavas gezinen yukseklik farki.
  anlik : GPS yukseklik yeniligi NIS_z > 6.63 (chi-kare 1 sd %99), art arda 3 olcum.
  yavas : |b_z|^2 / SIG_BZ^2 > 15.14 (chi-kare 1 sd %99.99), yani |b_z| > ~11.7 m.

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
# Yukseklik testi (1 serbestlik derecesi)
CHI2_99_1SD = 6.63
ESIK_BZ = 15.14
# GPS'in dikey hatasi yatayindan buyuktur (uydular hep ustte). NEO-M8N veri sayfasi dikey dogruluk vermiyor;
# yaygin kabul dikeyin yatayin ~1,5-2 kati oldugu (eksen basina 1,7 m -> ~3 m). Barometrenin yavas kaymasi da
# bu farkin icinde. Varsayimdir; analiz/spoof_roc.py bunu farkli degerlerle dener.
SIG_BZ = 3.0


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


class IrtifaDedektor:
    """Yukseklik spoofing'i: GPS yuksekligi barometre ve IMU ile karsilastirilir (dogrusal Kalman, 3 durum).

    Durum x = [h, v, b_z]: yukseklik (m, yukari +), dikey hiz (m/s), GPS-barometre yavas farki (m).
      Tahmin   : IMU'nun dikey ivmesi ile h ve v; b_z yavasca sifira doner (Gauss-Markov).
      Duzeltme : once barometre (olcum = h), sonra GPS yuksekligi (olcum = h + b_z).
    Barometre ve IMU disaridan sahte sinyalle degistirilemez; GPS yuksekligi degistirilebilir.
    Testler (SpoofDedektor ile ayni mantik, 1 serbestlik derecesi):
      anlik : GPS yukseklik yeniligi NIS_z > esik_z, art arda anlik_n olcum (ani sicrama)
      yavas : |b_z| / sig_bz kare > esik_bz (yavas kayma b_z'de birikir)
    Not: ArduPilot yuksekligi varsayilan olarak barometreden alir (EK3_SRC1_POSZ = 1). Bu yuzden yukseklik
    spoofing'i bu ayarla ucusu bozmaz; ama GPS yuksekligi kullanan bir arac (EK3_SRC1_POSZ = 3, RTK)
    ya da GPS yuksekligini gosteren bir yer istasyonu yaniltilir. Test, saldirinin varligini gosterir."""

    REF_SN = 3.0             # GPS ile barometrenin sifir farki: arac 3 sn dikeyde sabitken olculur
    REF_OYNAMA_M = 0.5       # o 3 sn'de barometre yuksekligi en fazla bu kadar degismis olmali
    GECIKME_BELIRSIZ_SN = 0.1  # GPS olcumunun zamanlama belirsizligi: tirmanirken/alcalirken GPS yuksekligi
                               # bu kadar eski olabilir; olcum belirsizligine (0,1 sn x dikey hiz) eklenir

    def __init__(self, sig_a=0.1, sig_baro=0.3, sig_wz=0.2, sig_bz=SIG_BZ, tau_bz=300.0,
                 esik_z=CHI2_99_1SD, esik_bz=ESIK_BZ, anlik_n=3, sifirla_sn=0.8, kilit_sn=10.0, k_a=0.15):
        self.sig_a, self.sig_bz, self.tau_bz, self.k_a = sig_a, sig_bz, tau_bz, k_a
        self._ref = deque()      # (t, h_baro, h_gps): sifir farkini bulmak icin son olcumler
        self.R_baro, self.R_gps = sig_baro ** 2, sig_wz ** 2
        self.esik_z, self.esik_bz, self.anlik_n, self.sifirla_sn, self.kilit_sn = esik_z, esik_bz, anlik_n, sifirla_sn, kilit_sn
        self.fark_siniri = math.sqrt(esik_bz) * sig_bz      # metre (~11,7)
        self.x = None
        self.t = None
        self.ofs = None              # ilk anda GPS yuksekligi - barometre yuksekligi (ikisinin sifiri farkli)
        self.kilit_bitis = -1.0
        self.tepe = None
        self.asim, self.asim_bas = 0, None

    def _baslat(self, h_baro, h_gps):
        self.x = np.array([h_baro, 0.0, h_gps - self.ofs - h_baro])
        self.P = np.diag([self.R_baro, 0.5 ** 2, self.R_baro + self.R_gps])
        self.asim, self.asim_bas = 0, None

    def sifirla(self):
        """Izleme kesilince: bastan baslar (yukseklik farkinin sifiri da yeniden olculur)."""
        self.x = None
        self.ofs = None
        self._ref.clear()

    def hazir(self):
        return self.ofs is not None

    def _referans(self, t, h_baro, h_gps):
        """GPS yuksekligi deniz seviyesinden, barometre kalkis noktasindan olcer: aradaki sabit farki bul.
        Arac dikeyde hareket ederken GPS'in gecikmesi farki bozar; bu yuzden REF_SN boyunca barometre
        REF_OYNAMA_M'den az degistiyse farklarin ortalamasi alinir."""
        self._ref.append((t, h_baro, h_gps))
        while self._ref and t - self._ref[0][0] > self.REF_SN:
            self._ref.popleft()
        if t - self._ref[0][0] < self.REF_SN - 0.25:
            return False
        bar = [b for _, b, _ in self._ref]
        if max(bar) - min(bar) > self.REF_OYNAMA_M:
            return False
        self.ofs = sum(g - b for _, b, g in self._ref) / len(self._ref)
        return True

    def _guncelle(self, z, H, R):
        nu = z - H @ self.x
        S = float(H @ self.P @ H) + R
        K = self.P @ H / S
        self.x = self.x + K * nu
        self.P = self.P - np.outer(K, H @ self.P)
        return nu * nu / S

    def adim(self, t, h_baro, h_gps, a_yukari, imu_seyrek=False, ivme_std=0.0):
        """t: sn, h_baro: barometre yuksekligi (m), h_gps: GPS yuksekligi (m), a_yukari: IMU dikey ivme (m/s2)."""
        if self.x is None:
            if self.ofs is None and not self._referans(t, h_baro, h_gps):
                return None                          # sifir farki henuz olculmedi: test calismiyor
            self.t = t
            self._baslat(h_baro, h_gps)
            return self._sonuc(t, 0.0, False, False)
        dt = t - self.t
        if dt <= 1e-3:
            return self._sonuc(t, 0.0, False, False)
        self.t = t
        phi = math.exp(-dt / self.tau_bz)
        F = np.array([[1.0, dt, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, phi]])
        B = np.array([dt * dt / 2, dt, 0.0])
        sig_a2 = SIG_A_SEYREK ** 2 if imu_seyrek else self.sig_a ** 2 + (self.k_a * a_yukari) ** 2 + ivme_std ** 2
        Q = np.outer(B, B) * sig_a2
        Q[2, 2] = self.sig_bz ** 2 * (1 - phi * phi)
        self.x = F @ self.x + B * a_yukari
        self.P = F @ self.P @ F.T + Q
        self._guncelle(h_baro, np.array([1.0, 0.0, 0.0]), self.R_baro)
        R_gps = self.R_gps + (self.GECIKME_BELIRSIZ_SN * self.x[1]) ** 2
        nis = self._guncelle(h_gps - self.ofs, np.array([1.0, 0.0, 1.0]), R_gps)
        if nis > self.esik_z:
            self.asim += 1
            if self.asim_bas is None:
                self.asim_bas = t
        else:
            self.asim, self.asim_bas = 0, None
        anlik = self.asim >= self.anlik_n
        yavas = (self.x[2] / self.sig_bz) ** 2 > self.esik_bz
        if self.asim_bas is not None and t - self.asim_bas >= self.sifirla_sn - 1e-6:
            self.x[0], self.x[2] = h_baro, h_gps - self.ofs - h_baro     # yeni farka yeniden basla
            self.P = np.diag([self.R_baro, self.P[1, 1], self.R_baro + self.R_gps])
            self.asim, self.asim_bas = 0, None
        return self._sonuc(t, nis, anlik, yavas)

    def fark(self):
        """Kestirilen GPS-barometre yukseklik farki (m, isaretli: + GPS daha yukarida)."""
        return 0.0 if self.x is None else float(self.x[2])

    def _sonuc(self, t, nis, anlik, yavas):
        fark = self.fark()
        if anlik or yavas:
            if t > self.kilit_bitis or self.tepe is None:
                self.tepe = dict(nis_z=nis, fark_z=abs(fark))
            self.kilit_bitis = t + self.kilit_sn
        alarm = t <= self.kilit_bitis
        if alarm and self.tepe is not None:
            self.tepe = dict(nis_z=max(self.tepe['nis_z'], nis), fark_z=max(self.tepe['fark_z'], abs(fark)))
        return dict(nis_z=nis, fark_z=fark, anlik_z=anlik, yavas_z=yavas, alarm=alarm,
                    tepe=dict(self.tepe) if (alarm and self.tepe) else None)


def basinc_yukseklik(p_hpa, p0_hpa):
    """Barometre: basinctan yukseklik (m), p0 aninda 0 (standart atmosfer formulu)."""
    return 44330.77 * (1.0 - (p_hpa / p0_hpa) ** 0.190263)


def govde_to_kd(ax_mg, ay_mg, az_mg, roll, pitch, yaw):
    """Govde eksenindeki ivme (milli-g) ve acilar (radyan) -> kuzey/dogu ivmesi (m/s2)."""
    fx, fy, fz = ax_mg * G / 1000, ay_mg * G / 1000, az_mg * G / 1000
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    an = cy * cp * fx + (cy * sp * sr - sy * cr) * fy + (cy * sp * cr + sy * sr) * fz
    ae = sy * cp * fx + (sy * sp * sr + cy * cr) * fy + (sy * sp * cr - cy * sr) * fz
    return an, ae


def govde_to_yukari(ax_mg, ay_mg, az_mg, roll, pitch):
    """Govde eksenindeki ivme (milli-g) -> dikey ivme (m/s2, yukari +, yercekimi cikarilmis)."""
    fx, fy, fz = ax_mg * G / 1000, ay_mg * G / 1000, az_mg * G / 1000
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    return -(-sp * fx + cp * sr * fy + cp * cr * fz + G)


class SpoofIzleyici:
    """Canli sistem icin: MAVLink mesajlarini toplar, her yeni GPS olcumunde dedektoru bir adim calistirir.

    Izleme kosulu (kapi): motorlar calisiyor, irtifa >= MIN_IRTIFA_M, GPS saglikli (3D fix, >= 6 uydu)
    ve jamming alarmi yok. Kosul bozulunca filtre sifirlanir, kosul gelince bastan baslar."""
    MIN_IRTIFA_M = 2.0
    MIN_IMU = 3              # iki GPS olcumu arasinda en az bu kadar ivme ornegi (50 Hz'de ~10 gelir)
    GPS_GECIKME_SN = 0.1     # GPS olcumu ~100 ms gecikmeli gelir (SITL: SIM_GPS1_LAG_MS=100; u-blox benzer).
                             # Ivme de ayni kadar geriden alinir; yoksa manevra anlarinda IMU ile GPS celisir.

    def __init__(self, kilit_sn=30.0, det=None, det_z=None):
        """det, det_z: istenirse farkli ayarli dedektorler (analiz icin); verilmezse varsayilanlar."""
        self.det = det or SpoofDedektor(kilit_sn=kilit_sn)
        self.det_z = det_z or IrtifaDedektor(kilit_sn=kilit_sn)
        self.ref = None                  # (enlem0, boylam0, cos(enlem0)): yerel koordinat merkezi
        self.aci = None                  # (roll, pitch, yaw) radyan
        self.ivme = None                 # (kuzey, dogu) m/s2, iki GPS olcumu arasindaki ivmelerin ortalamasi
        self.ivme_z = 0.0                # ayni araliktaki dikey ivme ortalamasi (yukari +)
        self.ivme_std_z = 0.0
        self._ivmeler = deque()          # (saat, kuzey, dogu, yukari): zaman damgali ivme ornekleri
        self._basinclar = deque()        # (saat, basinc hPa): barometre ornekleri
        self.p0 = None                   # ilk barometre basinci: barometre yuksekligi buna gore
        self._son_baro = None
        self._son_gps_saat = None
        self.imu_sayisi = 0              # son GPS araligindaki ivme ornegi sayisi
        self.ivme_std = 0.0              # o orneklerin standart sapmasi (m/s2)
        self.son = None                  # son adimin sonucu
        self.son_z = None                # yukseklik testinin son sonucu (barometre ya da GPS yuksekligi yoksa None)
        self.aktif = False

    def attitude(self, roll, pitch, yaw):
        self.aci = (roll, pitch, yaw)

    def raw_imu(self, xacc, yacc, zacc, saat=None):
        if self.aci is not None:
            an, ae = govde_to_kd(xacc, yacc, zacc, *self.aci)
            au = govde_to_yukari(xacc, yacc, zacc, self.aci[0], self.aci[1])
            self._ivmeler.append((time.monotonic() if saat is None else saat, an, ae, au))
            while len(self._ivmeler) > 500:
                self._ivmeler.popleft()

    def _ivme_al(self, saat):
        """Bir onceki GPS olcumu ile bu olcum arasindaki ivmelerin ortalamasi, GPS gecikmesi kadar geriden.
        Tek ornek yerine ortalama: tek ornek titresimden etkilenir."""
        bit = saat - self.GPS_GECIKME_SN
        bas = (self._son_gps_saat - self.GPS_GECIKME_SN) if self._son_gps_saat is not None else bit - 1.0
        self._son_gps_saat = saat
        sec = [(a, b, c) for s_, a, b, c in self._ivmeler if bas < s_ <= bit]
        self.imu_sayisi = len(sec)
        self.ivme_std = self.ivme_std_z = 0.0
        while self._ivmeler and self._ivmeler[0][0] <= bit:
            self._ivmeler.popleft()
        if sec:
            n = len(sec)
            self.ivme = (sum(a for a, _, _ in sec) / n, sum(b for _, b, _ in sec) / n)
            self.ivme_z = sum(c for _, _, c in sec) / n
            if n > 1:
                self.ivme_std = math.sqrt(sum((a - self.ivme[0]) ** 2 + (b - self.ivme[1]) ** 2 for a, b, _ in sec)
                                          / (n - 1) / 2)
                self.ivme_std_z = math.sqrt(sum((c - self.ivme_z) ** 2 for _, _, c in sec) / (n - 1))
        self._baro_bas, self._baro_bit = bas, bit
        return self.ivme if self.ivme is not None else (0.0, 0.0)

    def baro(self, basinc_hpa, saat=None):
        """SCALED_PRESSURE geldiginde cagrilir (press_abs, hPa). saat: aracin zaman damgasi (sn)."""
        if not basinc_hpa or basinc_hpa <= 0:
            return
        if self.p0 is None:
            self.p0 = basinc_hpa
        self._basinclar.append((time.monotonic() if saat is None else saat, basinc_hpa))
        while len(self._basinclar) > 200:
            self._basinclar.popleft()

    def _baro_al(self):
        """Ivme ile ayni aralikta (GPS gecikmesi kadar geriden) barometre yuksekliklerinin ortalamasi."""
        sec = [p for s_, p in self._basinclar if self._baro_bas < s_ <= self._baro_bit]
        while self._basinclar and self._basinclar[0][0] <= self._baro_bit - 1.0:
            self._basinclar.popleft()
        if sec:
            self._son_baro = basinc_yukseklik(sum(sec) / len(sec), self.p0)
        elif self._basinclar:
            self._son_baro = basinc_yukseklik(self._basinclar[-1][1], self.p0)
        return self._son_baro

    def gps(self, t, lat_e7, lon_e7, vel_cms, cog_cdeg, kapi_acik, saat=None, alt_mm=None):
        """GPS_RAW_INT geldiginde cagrilir. Sonuc sozlugu (ya da kapi kapaliysa None) dondurur.
        t: GPS'in kendi zaman damgasi (sn); saat: mesajin geldigi an (bilgisayar saati, ivme eslemesi icin).
        alt_mm: GPS yuksekligi (mm); verilirse ve barometre geliyorsa yukseklik testi de calisir."""
        saat = time.monotonic() if saat is None else saat
        if not kapi_acik or vel_cms == 65535:
            if self.aktif:
                self.det.sifirla()
                self.det_z.sifirla()
            self.aktif = False
            self.son = None
            self.son_z = None
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
        seyrek = self.imu_sayisi < self.MIN_IMU
        sonuc = self.det.adim(t, (n, e), zv, ivme, imu_seyrek=seyrek, ivme_std=self.ivme_std)
        h_baro = self._baro_al() if self.p0 is not None else None
        if alt_mm is not None and h_baro is not None:
            self.son_z = self.det_z.adim(t, h_baro, alt_mm / 1000.0, self.ivme_z, imu_seyrek=seyrek,
                                         ivme_std=self.ivme_std_z)
        self.son = birlestir(sonuc, self.son_z)
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
        return t <= self.det.kilit_bitis or t <= self.det_z.kilit_bitis


def birlestir(sonuc, sonuc_z):
    """Yatay ve dikey testin sonucunu tek sozlukte toplar. Yatay alarm yoksa ve dikey varsa tetik 'irtifa'."""
    s = dict(sonuc)
    if sonuc_z is None:
        s.update(nis_z=0.0, fark_z=0.0, irtifa=False)
        return s
    s.update(nis_z=sonuc_z['nis_z'], fark_z=sonuc_z['fark_z'], irtifa=sonuc_z['anlik_z'] or sonuc_z['yavas_z'])
    if sonuc_z['alarm']:
        if not s['alarm']:
            s['alarm'], s['tetik'], s['tepe'] = True, 'irtifa', None
        s['tepe'] = dict(s['tepe'] or {}, **sonuc_z['tepe'])
    return s


TETIK_METNI = {
    'hiz': 'GPS hızı bir anda değişti ama ivmeölçer bunu görmedi.',
    'anlik': 'GPS konumu bir anda sıçradı; ivmeölçer böyle bir hareket görmedi.',
    'yavas': 'GPS konumu yavaşça kaydı; kayma, bir GPS alıcısının kendi hatasından beklenenden büyük ve hız ile ivme bunu açıklamıyor.',
    'irtifa': 'GPS\'in bildirdiği yükseklik, barometre ve ivmeölçerle çelişiyor.',
}


SAPMA_SINIRI = SpoofDedektor().sapma_siniri
FARK_Z_SINIRI = IrtifaDedektor().fark_siniri


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
    if tetik == 'irtifa':
        return {'tur': 'spoofing', 'baslik': 'GPS spoofing şüphesi (yükseklik)',
                'cumle': TETIK_METNI[tetik] + ' Olası neden: sahte GPS sinyali.',
                'kisa': 'GPS yüksekliği barometreyle çelişiyor',
                'oneri': {'metin': 'GPS yüksekliğine güvenme; barometre yüksekliğini kullan.',
                          'alt': 'Yatay konumu da başka bir kaynakla doğrula.'},
                'anons': 'GPS spoofing şüphesi. Yükseklik tutmuyor.',
                'cipler': [['TEST', 'yükseklik'],
                           ['EN YÜKSEK YÜKSEKLİK NIS', f"{_tr(tepe.get('nis_z') or 0.0, 1)} (sınır {_tr(CHI2_99_1SD, 2)})"],
                           ['EN BÜYÜK GPS–BAROMETRE FARKI', f"{_tr(tepe.get('fark_z') or 0.0, 1)} m (sınır {_tr(FARK_Z_SINIRI, 1)})"]]}
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
