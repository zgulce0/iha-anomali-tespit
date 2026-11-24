"""GPS spoofing tespiti, CEVRIMDISI: kaydedilmis spoof_deney.py CSV'leri uzerinde calisir.

Dedektorun kendisi exporter/spoof_tespit.py icinde (canli sistemle AYNI kod). Bu dosya sadece
CSV'yi okur, dedektoru satir satir calistirir ve her senaryo icin tespit suresini yazar.

Kullanim (proje kokunden):
    venv/bin/python analiz/spoof_ekf.py veri/spoof_ucuncu_*.csv [--imusuz]
        --imusuz : IMU ivmesini sifirla (IMU'nun katkisini olcmek icin)
"""
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'exporter'))
from spoof_tespit import SpoofDedektor, M_DERECE, G  # noqa: E402


def yerel(d):
    """Enlem/boylam -> metre (kuzey, dogu); GPS hizi -> kuzey/dogu bilesenleri; IMU -> kuzey/dogu ivme."""
    lat0, lon0 = d.gps_enlem.iloc[0], d.gps_boylam.iloc[0]; c = np.cos(np.radians(lat0))
    for p in ['gps', 'ekf', 'gercek']:
        d[p + '_n'] = (d[p + '_enlem'] - lat0) * M_DERECE
        d[p + '_e'] = (d[p + '_boylam'] - lon0) * M_DERECE * c
    rot = np.radians(d.gps_rota_der)
    d['gvn'] = d.gps_hiz_ms * np.cos(rot); d['gve'] = d.gps_hiz_ms * np.sin(rot)
    r, p, y = [np.radians(d[c_]) for c_ in ['roll_der', 'pitch_der', 'yaw_der']]
    f = np.vstack([d.ivme_x_mg, d.ivme_y_mg, d.ivme_z_mg]).T * G / 1000
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    d['an'] = cy*cp*f[:, 0] + (cy*sp*sr - sy*cr)*f[:, 1] + (cy*sp*cr + sy*sr)*f[:, 2]
    d['ae'] = sy*cp*f[:, 0] + (sy*sp*sr + cy*cr)*f[:, 1] + (sy*sp*cr - cy*sr)*f[:, 2]
    d.attrs['imu'] = 'anlik ornek'
    # Yeni kayitlarda (spoof_deney.py, 50 Hz IMU) satir arasi ortalama ivme var: varsa onu kullan
    if 'imu_an_ort' in d.columns and d.imu_an_ort.notna().mean() > 0.95:
        d['an'] = d.imu_an_ort.ffill().fillna(0.0); d['ae'] = d.imu_ae_ort.ffill().fillna(0.0)
        d.attrs['imu'] = 'satir arasi ortalama (50 Hz)'
    return d


def calistir(d, **kw):
    det = SpoofDedektor(**kw); cikti = []
    for r in d.itertuples():
        cikti.append(det.adim(r.zaman_sn, (r.gps_n, r.gps_e), (r.gvn, r.gve), (r.an, r.ae)))
    sonuc = pd.concat([d, pd.DataFrame(cikti, index=d.index)], axis=1)
    sonuc.attrs = dict(d.attrs)
    return sonuc


def yukle(yol, **kw):
    d = pd.read_csv(yol)
    d = d[d.irtifa_m > 8].reset_index(drop=True)   # sadece havadaki kisim
    return calistir(yerel(d), **kw)


def bolge(d):
    """temiz = normal + hazirlik (havada) + toparlanmanin son 10 sn'si (arac gercek konumuna donmus)."""
    d['bas'] = (d.etiket != d.etiket.shift()).cumsum()
    d['gecen'] = d.zaman_sn - d.groupby('bas').zaman_sn.transform('min')
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
              f'{h.tetik:>10s} {h.gercek_sapma_m:20.2f} m')


if __name__ == '__main__':
    imusuz = '--imusuz' in sys.argv          # IMU'nun katkisini olcmek icin: ivmeyi sifirla
    for yol in [a for a in sys.argv[1:] if not a.startswith('--') and not a.endswith('_olaylar.csv')]:
        d = pd.read_csv(yol); d = d[d.irtifa_m > 8].reset_index(drop=True); d = yerel(d)
        if imusuz:
            d['an'] = 0.0; d['ae'] = 0.0
        rapor(yol, bolge(calistir(d)))
