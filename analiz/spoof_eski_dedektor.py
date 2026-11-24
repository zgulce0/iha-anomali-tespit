"""Onceki spoofing dedektoru (4 durumlu Kalman filtresi + pencere ortalamasi + CUSUM).

SADECE KARSILASTIRMA ICIN: analiz/spoof_kalibrasyon.py yeni dedektoru bununla karsilastirir.
Canli sistemde kullanilmaz. Neden birakildi: simulatorun hatasiz GPS'inde cok hizliydi, ama
gercekci GPS hatasi eklenince saatte 40-70 yanlis alarm verdi (yavas kayma testi).
"""
import math
from collections import deque

import numpy as np

CHI2_99_2SD = 9.21
REF_DT = 0.2


class EskiSpoofDedektor:
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
