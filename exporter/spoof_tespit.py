"""
GPS spoofing tespiti: kendi Kalman filtremiz + NIS (chi-kare) + pencere ortalamasi + CUSUM.

Bu dosya hem canli sistemde (exporter.py) hem cevrimdisi analizde (analiz/spoof_ekf.py)
kullanilir; iki yerde ayni kod calisir.

Durum x = [kuzey, dogu, v_kuzey, v_dogu] (metre, m/s).
  Tahmin   : IMU ivmesi (govde -> kuzey/dogu, ATTITUDE acilariyla) ile dead-reckoning
  Duzeltme : once GPS hizi, sonra GPS konumu
Yenilik (innovation) nu = olcum - tahmin,  S = H P H' + R,  NIS = nu' S^-1 nu  (2 serbestlik derecesi).

Uc test (herhangi biri alarm acar, alarm KILIT_SN saniye acik kalir):
  hiz   : GPS HIZ yeniligi icin NIS_v > 9.21. GPS hizi bir anda degisip IMU bunu gormezse
          (ivme yok) yakalar: konum ve hizi birlikte kaydiran saldirinin baslangici.
  anlik : GPS KONUM yeniligi icin NIS > 9.21, art arda 3 olcum. Ani sicrama.
  yavas : son 12.6 sn'deki konum yeniliklerinin ORTALAMASI m (metre). Gurultu yon degistirdigi
          icin ortalamada sonuyor; kayma hep ayni yone ittigi icin birikiyor.
          m uzerine CUSUM: g = min(2h, max(0, g + (m - k) * dt/0.2)),  g > h ise alarm.
9.21: 2 serbestlik dereceli chi-kare dagiliminin %99 degeri.
Anlik test 0.8 sn (5 olcum) boyunca surekli asilirsa filtre GPS'in yeni konumuna yeniden baslatilir
(ArduPilot EKF'sinin de yaptigi gibi); boylece olay bittikten sonra eski hata birikmez.

Esikler (k, h) ve gurultu degerleri (sig_*) SITL deneylerinde normal ucustan secildi:
veri/spoof_ucuncu_*.csv, veri/spoof_dorduncu_*.csv, veri/spoof_besinci_*.csv.
"""
import math
from collections import deque

import numpy as np

G = 9.80665
M_DERECE = 111320.0          # 1 derece enlem ~ 111.32 km
CHI2_99_2SD = 9.21
REF_DT = 0.2                 # esikler 5 Hz (0.2 sn) ornekle secildi; CUSUM artisi buna gore olceklenir


class SpoofDedektor:
    def __init__(self, sig_a=0.3, sig_p=0.5, sig_v=0.1, pencere_sn=12.6, k=0.18, h=0.5,
                 sifirla_sn=0.8, anlik_n=3, kilit_sn=10.0):
        self.sig_a, self.k, self.h = sig_a, k, h
        self.R_p = np.eye(2) * sig_p ** 2
        self.R_v = np.eye(2) * sig_v ** 2
        self.Hp = np.array([[1, 0, 0, 0], [0, 1, 0, 0.]])
        self.Hv = np.array([[0, 0, 1, 0], [0, 0, 0, 1.]])
        self.pencere_sn = pencere_sn
        self.sifirla_sn, self.anlik_n, self.kilit_sn = sifirla_sn, anlik_n, kilit_sn
        self.x = None
        self.t = None
        self.kilit_bitis = -1.0
        self.son_tetik = None
        self.tepe = None             # alarm boyunca testlerin en yuksek degerleri (ekranda kanit olarak)
        self.nis_onceki = 0.0
        self.son_sicrama = None      # konum NIS'inin bir adimda sinirin cok ustune firladigi son an
        self._temizle()

    def _temizle(self):
        self.pencere = deque()       # (t, nu)
        self.g = 0.0
        self.asim = 0
        self.asim_bas = None

    def _baslat(self, zp, zv):
        self.x = np.array([zp[0], zp[1], zv[0], zv[1]], dtype=float)
        self.P = np.diag([self.R_p[0, 0]] * 2 + [self.R_v[0, 0]] * 2)
        self._temizle()

    def sifirla(self):
        """Izleme kesilince (yere inis, GPS kaybi) cagrilir: filtre bir sonraki olcumde bastan baslar.
        Acik alarm kilidi korunur."""
        self.x = None

    def adim(self, t, zp, zv, a_ne):
        """t: sn, zp: GPS konum (kuzey, dogu) m, zv: GPS hiz (kuzey, dogu) m/s,
        a_ne: IMU ivmesi (kuzey, dogu) m/s2. Sonuc sozlugu dondurur."""
        zp = np.asarray(zp, dtype=float); zv = np.asarray(zv, dtype=float); a_ne = np.asarray(a_ne, dtype=float)
        if self.x is None:
            self.t = t
            self._baslat(zp, zv)
            return self._sonuc(t, 0.0, 0.0, 0.0, False, False, False, True)
        dt = t - self.t
        if dt <= 1e-3:                       # ayni olcum tekrar geldi
            return self._sonuc(t, 0.0, 0.0, self._m(), False, False, False, False)
        self.t = t
        F = np.eye(4); F[0, 2] = F[1, 3] = dt
        B = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
        self.x = F @ self.x + B @ a_ne
        self.P = F @ self.P @ F.T + B @ B.T * self.sig_a ** 2
        # GPS hiz yeniligi (test: hiz)
        nu_v = zv - self.Hv @ self.x
        S = self.Hv @ self.P @ self.Hv.T + self.R_v
        nis_v = float(nu_v @ np.linalg.solve(S, nu_v))
        K = self.P @ self.Hv.T @ np.linalg.inv(S)
        self.x = self.x + K @ nu_v
        self.P = (np.eye(4) - K @ self.Hv) @ self.P
        hiz = nis_v > CHI2_99_2SD
        # GPS konum yeniligi (test: anlik)
        nu = zp - self.Hp @ self.x
        S = self.Hp @ self.P @ self.Hp.T + self.R_p
        nis = float(nu @ np.linalg.solve(S, nu))
        K = self.P @ self.Hp.T @ np.linalg.inv(S)
        self.x = self.x + K @ nu
        self.P = (np.eye(4) - K @ self.Hp) @ self.P
        # Sicrama mi kayma mi? Sicramada NIS tek adimda sinirin ustune firlar (onceki degerin en az 4 kati);
        # kaymada adim adim buyur. Bu ayrim sadece ekrandaki aciklama icindir, alarm kararini degistirmez.
        if nis > CHI2_99_2SD and nis >= 4 * max(self.nis_onceki, 1.0):
            self.son_sicrama = t
        self.nis_onceki = nis
        if nis > CHI2_99_2SD:
            self.asim += 1
            if self.asim_bas is None:
                self.asim_bas = t
        else:
            self.asim, self.asim_bas = 0, None
        anlik = self.asim >= self.anlik_n            # tek bir gurultu sicramasi degil, art arda 3 olcum
        # pencere ortalamasi + CUSUM (test: yavas)
        self.pencere.append((t, nu))
        while self.pencere and t - self.pencere[0][0] > self.pencere_sn:
            self.pencere.popleft()
        m = self._m()
        self.g = min(2 * self.h, max(0.0, self.g + (m - self.k) * dt / REF_DT))
        yavas = self.g > self.h
        sifirlandi = False
        if self.asim_bas is not None and t - self.asim_bas >= self.sifirla_sn - 1e-6:
            self._baslat(zp, zv)                     # 1 sn surekli buyuk sicrama: yeni konuma yeniden basla
            self.nis_onceki = 0.0
            sifirlandi = True
        return self._sonuc(t, nis, nis_v, m, anlik, yavas, hiz, sifirlandi)

    def _m(self):
        """Pencere dolmussa (en az pencere_sn'nin %95'i) yeniliklerin ortalamasinin buyuklugu, yoksa 0."""
        if len(self.pencere) < 2 or self.pencere[-1][0] - self.pencere[0][0] < 0.95 * self.pencere_sn:
            return 0.0
        ort = np.mean([nu for _, nu in self.pencere], axis=0)
        return float(math.hypot(ort[0], ort[1]))

    def _sonuc(self, t, nis, nis_v, m, anlik, yavas, hiz, sifirlandi):
        if anlik or yavas or hiz:
            yeni = t > self.kilit_bitis              # onceki alarm bitmisti: yeni bir olay
            self.kilit_bitis = t + self.kilit_sn
            # Ne oldu? Son 1 sn icinde ani bir sicrama goruldiyse 'anlik', yoksa konum adim adim kaymistir.
            sicrama = self.son_sicrama is not None and t - self.son_sicrama <= 1.0
            tetik = 'hiz' if hiz else ('anlik' if sicrama else 'yavas')
            if yeni or self.tepe is None:
                self.son_tetik = tetik
                self.tepe = dict(nis=nis, nis_v=nis_v, m=m)
        alarm = t <= self.kilit_bitis
        if alarm and self.tepe is not None:
            self.tepe = dict(nis=max(self.tepe['nis'], nis), nis_v=max(self.tepe['nis_v'], nis_v),
                             m=max(self.tepe['m'], m))
        return dict(nis=nis, nis_v=nis_v, m=m, g=self.g, anlik=anlik, yavas=yavas, hiz=hiz,
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

    def __init__(self, kilit_sn=30.0):
        self.det = SpoofDedektor(kilit_sn=kilit_sn)
        self.ref = None                  # (enlem0, boylam0, cos(enlem0)): yerel koordinat merkezi
        self.aci = None                  # (roll, pitch, yaw) radyan
        self.ivme = None                 # (kuzey, dogu) m/s2, son RAW_IMU'dan
        self.son = None                  # son adimin sonucu
        self.aktif = False

    def attitude(self, roll, pitch, yaw):
        self.aci = (roll, pitch, yaw)

    def raw_imu(self, xacc, yacc, zacc):
        if self.aci is not None:
            self.ivme = govde_to_kd(xacc, yacc, zacc, *self.aci)

    def gps(self, t, lat_e7, lon_e7, vel_cms, cog_cdeg, kapi_acik):
        """GPS_RAW_INT geldiginde cagrilir. Sonuc sozlugu (ya da kapi kapaliysa None) dondurur."""
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
        self.son = self.det.adim(t, (n, e), zv, self.ivme if self.ivme is not None else (0.0, 0.0))
        return self.son

    def alarm(self, t):
        """Kilit suresi dolmadiysa True (izleme kesilse bile acik alarm kilit suresince surer)."""
        return t <= self.det.kilit_bitis


TETIK_METNI = {
    'hiz': 'GPS hızı bir anda değişti ama ivmeölçer bunu görmedi.',
    'anlik': 'GPS konumu bir anda sıçradı; ivmeölçer böyle bir hareket görmedi.',
    'yavas': 'GPS konumu hep aynı yöne kayıyor; hız ve ivme bu kaymayı açıklamıyor.',
}


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
    m = tepe.get('m') or 0.0
    nis = tepe.get('nis') or 0.0
    nis_v = tepe.get('nis_v') or 0.0
    return {'tur': 'spoofing', 'baslik': 'GPS spoofing şüphesi',
            'cumle': TETIK_METNI[tetik] + ' Olası neden: sahte GPS sinyali.',
            'kisa': {'hiz': 'GPS hızı ile ivmeölçer çelişiyor', 'anlik': 'GPS konumu sıçradı',
                     'yavas': 'GPS konumu sürekli kayıyor'}[tetik],
            'oneri': {'metin': 'GPS konumuna güvenme, araç sandığı yerde olmayabilir.',
                      'alt': 'Konumu başka bir kaynakla (görüş, kamera) doğrula.'},
            'anons': 'GPS spoofing şüphesi. Konuma güvenme.',
            'cipler': [['TEST', {'hiz': 'hız yeniliği', 'anlik': 'ani sıçrama', 'yavas': 'kayma (CUSUM)'}[tetik]],
                       ['EN YÜKSEK KONUM NIS', f'{_tr(nis, 1)} (sınır 9,21)'],
                       ['EN YÜKSEK HIZ NIS', f'{_tr(nis_v, 1)} (sınır 9,21)'],
                       ['EN YÜKSEK ORT. KAYMA', f'{_tr(m, 2)} m (sınır 0,18)']]}
