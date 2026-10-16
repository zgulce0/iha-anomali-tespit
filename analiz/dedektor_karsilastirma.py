"""
Normal ucus verisiyle ogrenen ariza dedektorlerinin karsilastirilmasi (rejim bazli: hover ve daire).

Kullanim (exporter/ klasorunden):
  ../venv/bin/python ../analiz/dedektor_karsilastirma.py HOVER.csv DAIRE.csv ESKI_NORMAL.csv

Dedektorler (hepsi sadece NORMAL satirlardan ogrenir, ariza gormez):
  1  sabit V<11.1          : naif sabit voltaj esigi
  2  yayilim               : motor PWM max-min, esik normalden ogrenilir
  3  kural(cift,artik)     : |m1-m2|, |m3-m4|, |V-(12.6-0.01*I)| esikleri normalden ogrenilir
  3b zarf (ham)            : her ozellik (4 PWM, V, A) kendi normal araligindan sapinca alarm
  7  Mahalanobis (ham)     : 6 boyutlu normal bulutuna uzaklik
  4/5/6 Isolation Forest   : ham / muhendislik ozellikleri / gurultu eklenmis egitim
Degerlendirme: normal segmentler sirayla egitim/test'e bolunur (sizinti yok); ayrica baska gunun normal ucusunda yanlis alarm olculur.
Gecis kirpma: her segmentin ilk 5 sn'si dahil edilmez (olculen en yavas toparlanma 4 sn).
"""
import sys, numpy as np, pandas as pd
from sklearn.ensemble import IsolationForest
M = ['motor1_pwm','motor2_pwm','motor3_pwm','motor4_pwm']
TRIM = 5.0
V0, R0 = 12.6, 0.01

def hazirla(d):
    d = d.copy()
    d['seg'] = (d['etiket'] != d['etiket'].shift()).cumsum()
    d['seg_t'] = d['zaman_sn'] - d.groupby('seg')['zaman_sn'].transform('min')
    d['seg_bas'] = d.groupby('seg')['zaman_sn'].transform('min')
    d['artik'] = d['voltaj_v'] - (V0 - R0 * d['akim_a'])
    d['d12'] = d['motor1_pwm'] - d['motor2_pwm']
    d['d34'] = d['motor3_pwm'] - d['motor4_pwm']
    d['yayilim'] = d[M].max(axis=1) - d[M].min(axis=1)
    d['ort_pwm'] = d[M].mean(axis=1)
    return d

HAM = M + ['voltaj_v', 'akim_a']
OZ = ['d12', 'd34', 'artik', 'ort_pwm']

class Esik:
    """Normal egitim verisinden esik ogrenen kural: |x| > carpan*max(egitim) (alt sinir tabanli)"""
    def __init__(self, kolonlar, taban, carpan=1.5): self.k, self.t, self.c = kolonlar, taban, carpan
    def fit(self, X):
        self.e = {k: max(self.c * X[k].abs().max(), t) for k, t in zip(self.k, self.t)}; return self
    def tahmin(self, X):
        r = np.zeros(len(X), bool)
        for k in self.k: r |= (X[k].abs() > self.e[k]).values
        return r


FLOOR = {'motor1_pwm':1.0,'motor2_pwm':1.0,'motor3_pwm':1.0,'motor4_pwm':1.0,'voltaj_v':0.01,'akim_a':0.05}

class Zarf:
    """Rejim bazli ham ozellik zarfi: her ozellik icin egitim medyanindan sapma > max(1.5*max_sapma, 3*cozunurluk)"""
    def fit(self, X):
        self.med = X[HAM].median()
        self.e = {k: max(1.5 * (X[k] - self.med[k]).abs().max(), 3 * FLOOR[k]) for k in HAM}; return self
    def tahmin(self, X):
        r = np.zeros(len(X), bool)
        for k in HAM: r |= ((X[k] - self.med[k]).abs() > self.e[k]).values
        return r

class Mahal:
    def fit(self, X):
        A = X[HAM].values; self.mu = A.mean(0)
        C = np.cov(A.T) + np.diag([FLOOR[k]**2 for k in HAM]); self.P = np.linalg.inv(C)
        self.thr = 1.5 * self.md(A).max(); return self
    def md(self, A):
        D = A - self.mu; return np.sqrt(np.einsum('ij,jk,ik->i', D, self.P, D))
    def tahmin(self, X): return self.md(X[HAM].values) > self.thr

def modeller(tr):
    out = {}
    out['1 sabit V<11.1'] = lambda X: (X['voltaj_v'] < 11.1).values
    ytr = tr['yayilim']
    ysin = max(1.5 * ytr.max(), 5)
    out['2 yayilim'] = lambda X, s=ysin: (X['yayilim'] > s).values
    e = Esik(['d12','d34','artik'], [3.0, 3.0, 0.03]).fit(tr)
    out['3 kural(cift,artik)'] = e.tahmin
    out['3b zarf (ham)'] = Zarf().fit(tr).tahmin
    out['7 Mahalanobis (ham)'] = Mahal().fit(tr).tahmin
    rng = np.random.default_rng(0)
    A = np.tile(tr[HAM].values, (10, 1)); A = A + rng.normal(0, 1, A.shape) * np.array([FLOOR[k] for k in HAM])
    mg = IsolationForest(n_estimators=300, random_state=0).fit(A)
    thg = np.percentile(mg.score_samples(A), 0.5)
    out['6 IF ham+gurultu'] = lambda X, mg=mg, thg=thg: mg.score_samples(X[HAM].values) < thg
    for ad, kol in (('4 IF ham', HAM), ('5 IF ozellik', OZ)):
        m = IsolationForest(n_estimators=300, random_state=0).fit(tr[kol])
        sk = m.score_samples(tr[kol])
        thr = np.percentile(sk, 0.5)          # egitim normalinin %0.5'i alarm olsun
        out[ad] = (lambda X, m=m, kol=kol, thr=thr: m.score_samples(X[kol]) < thr)
    return out

def rejim_calis(ad, d, mod, harici):
    d = hazirla(d)
    d = d[(d['armed'] == 1) & (d['mod'] == mod) & (d['etiket'] != 'hazirlik')]
    # normal segmentleri sirala
    nseg = [s for s, g in d.groupby('seg') if g['etiket'].iloc[0] == 'normal']
    egit_seg, test_seg = set(nseg[0::2]), set(nseg[1::2])
    temiz = d[d['seg_t'] >= TRIM]
    tr = temiz[temiz['seg'].isin(egit_seg)]
    tn = temiz[temiz['seg'].isin(test_seg)]
    ar = temiz[temiz['etiket'] != 'normal']
    print(f"\n######## {ad}  (mod={mod})  egitim normal {len(tr)} | test normal {len(tn)} | ariza satiri {len(ar)}")
    mdl = modeller(tr)
    ext = None
    if harici is not None:
        ext = hazirla(harici); ext = ext[ext['mod'].isin(harici_modlar[mod])]
        ext = ext.assign(grp=(ext['mod'] != ext['mod'].shift()).cumsum())
        ext['t'] = ext['zaman_sn'] - ext.groupby('grp')['zaman_sn'].transform('min')
        ext = ext[(ext['t'] >= 10) & (ext['armed'] == 1)]
    sonuc = {}
    for dn, f in mdl.items():
        fp_test = f(tn).mean() if len(tn) else np.nan
        fp_ext = f(ext).mean() if ext is not None and len(ext) else np.nan
        pa, pn = f(ar), f(tn)
        tp = pa.sum(); fn = (~pa).sum(); fp = pn.sum()
        prec = tp / (tp + fp) if tp + fp else np.nan
        rec = tp / (tp + fn); f1 = 2*prec*rec/(prec+rec) if prec == prec and prec+rec else np.nan
        sonuc[dn] = dict(prec=prec, rec=rec, f1=f1, fp_test=fp_test, fp_ext=fp_ext)
    print(pd.DataFrame(sonuc).T.round(3).to_string())
    # arıza bazlı yakalama orani
    tab = {}
    for et, g in ar.groupby('etiket'):
        tab[et] = {dn: f(g).mean() for dn, f in mdl.items()}
    tab = pd.DataFrame(tab).T
    tab['tur'] = [('batarya' if e.startswith('batt') else 'motor') for e in tab.index]
    print("\nAriza bazli yakalama orani (satir yuzdesi):")
    sirali = sorted(tab.index, key=lambda e: (e.split('_')[0] if e.startswith('motor') else 'z'+e.split('_')[1], float(e.split('_')[-1])))
    print(tab.loc[sirali].drop(columns='tur').round(2).to_string())
    # gecikme: ilk alarm (kirpmadan)
    ham_ar = d[d['etiket'] != 'normal']
    lat = {dn: [] for dn in mdl}
    for s, g in ham_ar.groupby('seg'):
        for dn, f in mdl.items():
            hit = np.where(f(g))[0]
            lat[dn].append(g['zaman_sn'].iloc[hit[0]] - g['seg_bas'].iloc[0] if len(hit) else np.nan)
    L = pd.DataFrame(lat)
    print("\nIlk alarma kadar gecen sure (sn), arizalar uzerinden:")
    print(pd.DataFrame({'yakalanan': L.notna().sum(), 'medyan': L.median(), 'max': L.max()}).round(1).to_string())
    return tab

harici_modlar = {'CIRCLE': ['CIRCLE'], 'GUIDED': ['LOITER']}
if __name__ == '__main__':
    hover, daire, eski = sys.argv[1:4]
    eskid = pd.read_csv(eski)
    rejim_calis('HOVER', pd.read_csv(hover), 'GUIDED', eskid)
    rejim_calis('DAIRE', pd.read_csv(daire), 'CIRCLE', eskid)