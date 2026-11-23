"""GPS spoofing tespiti: kendi Kalman filtremiz + NIS (chi-kare) + pencere ortalamasi + CUSUM.

Durum x = [kuzey, dogu, v_kuzey, v_dogu] (metre, m/s).
  Tahmin   : IMU ivmesi (govde -> kuzey/dogu, ATTITUDE acilariyla) ile dead-reckoning
  Duzeltme : once GPS hizi, sonra GPS konumu
GPS konum yeniligi  nu = z_gps - H x,  S = H P H' + R,  NIS = nu' S^-1 nu  (2 serbestlik derecesi).

Uc test:
  0) Hiz testi   : GPS HIZ yeniligi icin NIS_v > 9.21. GPS hizi bir anda degisip IMU bunu
                   gormezse (ivme yok) yakalar: konum+hizi birlikte kaydiran saldirinin baslangici.
  1) Anlik test  : NIS > 9.21 (chi-kare, 2 sd, %99), art arda 3 olcum -> ani sicrama
  2) Yavas test  : son 12.6 sn'deki yeniliklerin ORTALAMASI m (metre). Gurultu yon degistirdigi
                   icin ortalamada sonuyor; kayma hep ayni yone ittigi icin ortalamada birikiyor.
                   m uzerine CUSUM: g = min(2h, max(0, g + m - k)),  g > h ise alarm.
Anlik test 1 sn boyunca surekli asilirsa filtre GPS'in yeni konumuna yeniden baslatilir
(ArduPilot EKF'sinin de yaptigi gibi); boylece olay bittikten sonra eski hata birikmez."""
from collections import deque
import sys
import numpy as np, pandas as pd

M = 111320.0; G = 9.80665
CHI2_99_2SD = 9.21

class SpoofDedektor:
    def __init__(self, sig_a=0.3, sig_p=0.5, sig_v=0.1, pencere=63, k=0.18, h=0.5,
                 sifirla_n=5, anlik_n=3, kilit_sn=10.0):
        self.sig_a, self.k, self.h = sig_a, k, h
        self.R_p = np.eye(2) * sig_p**2; self.R_v = np.eye(2) * sig_v**2
        self.Hp = np.array([[1, 0, 0, 0], [0, 1, 0, 0.]]); self.Hv = np.array([[0, 0, 1, 0], [0, 0, 0, 1.]])
        self.pencere = deque(maxlen=pencere)
        self.sifirla_n, self.anlik_n, self.kilit_sn = sifirla_n, anlik_n, kilit_sn
        self.x = None; self.t = None; self.g = 0.0; self.asim = 0; self.kilit_bitis = -1.0

    def _baslat(self, zp, zv):
        self.x = np.array([zp[0], zp[1], zv[0], zv[1]]); self.P = np.diag([self.R_p[0, 0]] * 2 + [self.R_v[0, 0]] * 2)
        self.pencere.clear(); self.g = 0.0; self.asim = 0

    def adim(self, t, zp, zv, a_ne):
        """t: sn, zp: GPS konum (kuzey, dogu) m, zv: GPS hiz (kuzey, dogu) m/s, a_ne: IMU ivmesi (kuzey, dogu) m/s2"""
        if self.x is None:
            self.t = t; self._baslat(zp, zv)
            return dict(nis=0.0, nis_v=0.0, m=0.0, g=0.0, anlik=False, yavas=False, hiz=False,
                        alarm=False, sifirlandi=True)
        dt = max(1e-3, t - self.t); self.t = t
        F = np.eye(4); F[0, 2] = F[1, 3] = dt
        B = np.array([[dt*dt/2, 0], [0, dt*dt/2], [dt, 0], [0, dt]])
        self.x = F @ self.x + B @ a_ne
        self.P = F @ self.P @ F.T + B @ B.T * self.sig_a**2
        # GPS hiz guncellemesi
        nu_v = zv - self.Hv @ self.x
        S = self.Hv @ self.P @ self.Hv.T + self.R_v; K = self.P @ self.Hv.T @ np.linalg.inv(S)
        nis_v = float(nu_v @ np.linalg.solve(S, nu_v))
        self.x = self.x + K @ nu_v; self.P = (np.eye(4) - K @ self.Hv) @ self.P
        hiz = nis_v > CHI2_99_2SD
        # GPS konum yeniligi ve NIS
        nu = zp - self.Hp @ self.x; S = self.Hp @ self.P @ self.Hp.T + self.R_p
        nis = float(nu @ np.linalg.solve(S, nu))
        K = self.P @ self.Hp.T @ np.linalg.inv(S)
        self.x = self.x + K @ nu; self.P = (np.eye(4) - K @ self.Hp) @ self.P
        # Test 1: anlik
        self.asim = self.asim + 1 if nis > CHI2_99_2SD else 0
        anlik = self.asim >= self.anlik_n        # tek bir gurultu sicramasi degil, art arda 3 olcum (0.6 sn)
        # Test 2: pencere ortalamasi + CUSUM
        self.pencere.append(nu)
        m = float(np.hypot(*np.mean(self.pencere, axis=0))) if len(self.pencere) == self.pencere.maxlen else 0.0
        self.g = min(2 * self.h, max(0.0, self.g + m - self.k))
        yavas = self.g > self.h
        if anlik or yavas or hiz:
            self.kilit_bitis = t + self.kilit_sn
        sifirlandi = False
        if self.asim >= self.sifirla_n:          # 1 sn surekli buyuk sicrama -> yeni konuma yeniden basla
            self._baslat(zp, zv); sifirlandi = True
        return dict(nis=nis, nis_v=nis_v, m=m, g=self.g, anlik=anlik, yavas=yavas, hiz=hiz,
                    alarm=t <= self.kilit_bitis, sifirlandi=sifirlandi)

def yerel(d):
    lat0, lon0 = d.gps_enlem.iloc[0], d.gps_boylam.iloc[0]; c = np.cos(np.radians(lat0))
    for p in ['gps', 'ekf', 'gercek']:
        d[p + '_n'] = (d[p + '_enlem'] - lat0) * M
        d[p + '_e'] = (d[p + '_boylam'] - lon0) * M * c
    rot = np.radians(d.gps_rota_der)
    d['gvn'] = d.gps_hiz_ms * np.cos(rot); d['gve'] = d.gps_hiz_ms * np.sin(rot)
    r, p, y = [np.radians(d[c_]) for c_ in ['roll_der', 'pitch_der', 'yaw_der']]
    f = np.vstack([d.ivme_x_mg, d.ivme_y_mg, d.ivme_z_mg]).T * G / 1000
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    d['an'] = cy*cp*f[:, 0] + (cy*sp*sr - sy*cr)*f[:, 1] + (cy*sp*cr + sy*sr)*f[:, 2]
    d['ae'] = sy*cp*f[:, 0] + (sy*sp*sr + cy*cr)*f[:, 1] + (sy*sp*cr - cy*sr)*f[:, 2]
    d.attrs['imu'] = 'anlik ornek (5 Hz)'
    # Yeni kayitlarda (spoof_deney.py, 50 Hz IMU) satir arasi ortalama ivme var: varsa onu kullan
    if 'imu_an_ort' in d.columns and d.imu_an_ort.notna().mean() > 0.95:
        d['an'] = d.imu_an_ort.ffill().fillna(0.0); d['ae'] = d.imu_ae_ort.ffill().fillna(0.0)
        d.attrs['imu'] = 'satir arasi ortalama (50 Hz)'
    return d

def calistir(d, **kw):
    det = SpoofDedektor(**kw); cikti = []
    for r in d.itertuples():
        cikti.append(det.adim(r.zaman_sn, np.array([r.gps_n, r.gps_e]), np.array([r.gvn, r.gve]), np.array([r.an, r.ae])))
    sonuc = pd.concat([d, pd.DataFrame(cikti, index=d.index)], axis=1)
    sonuc.attrs = dict(d.attrs)
    return sonuc

def yukle(yol, **kw):
    d = pd.read_csv(yol)
    d = d[d.irtifa_m > 8].reset_index(drop=True)   # sadece havadaki kisim
    return calistir(yerel(d), **kw)


# ---------------------------------------------------------------- degerlendirme
def bolge(d):
    """temiz = normal + hazirlik (havada) + toparlanmanin son 10 sn'si (arac gercek konumuna donmus)."""
    d['bas'] = (d.etiket != d.etiket.shift()).cumsum()
    d['gecen'] = d.zaman_sn - d.groupby('bas').zaman_sn.transform('min')
    d.attrs = dict(d.attrs)
    d['temiz'] = d.etiket.isin(['normal', 'hazirlik']) | ((d.etiket == 'toparlanma') & (d.gecen > 30))
    return d

def rapor(ad, d):
    ya, nt = int(d[d.temiz].alarm.sum()), int(d.temiz.sum())
    print(f'\n{ad}   [IMU: {d.attrs.get("imu", "?")}]\n  temiz satirlarda yanlis alarm: {ya}/{nt}')
    print(f'  {"senaryo":20s} {"tespit":>8s} {"kapsama":>8s} {"ilk tetik":>10s} {"o andaki gercek sapma":>22s}')
    for et, grp in d[d.etiket.str.startswith('spoof')].groupby('etiket', sort=False):
        hit = grp[grp.alarm]
        if len(hit) == 0:
            print(f'  {et:20s} {"YOK":>8s}'); continue
        h = hit.iloc[0]
        print(f'  {et:20s} {h.zaman_sn - grp.zaman_sn.iloc[0]:7.1f}s {grp.alarm.mean()*100:7.0f}% '
              f'{"hiz" if h.hiz else ("anlik" if h.anlik else "yavas"):>10s} {h.gercek_sapma_m:20.2f} m')

if __name__ == '__main__':
    imusuz = '--imusuz' in sys.argv          # IMU'nun katkisini olcmek icin: ivmeyi sifirla
    for yol in [a for a in sys.argv[1:] if not a.startswith('--') and not a.endswith('_olaylar.csv')]:
        d = pd.read_csv(yol); d = d[d.irtifa_m > 8].reset_index(drop=True); d = yerel(d)
        if imusuz:
            d['an'] = 0.0; d['ae'] = 0.0
        rapor(yol, bolge(calistir(d)))
