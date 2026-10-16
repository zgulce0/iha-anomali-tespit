"""
Gurultu dayanikliligi: normal ve ariza verisine yapay Gaussian gurultu ekleyip dedektorlerin yakalama oranini olcer.
Kullanim: ../venv/bin/python ../analiz/gurultu_testi.py HOVER.csv DAIRE.csv
"""
import os, sys; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, pandas as pd
from dedektor_karsilastirma import hazirla, modeller, TRIM, M
SEVIYE = [(0,0,0),(2,0.02,0.1),(5,0.05,0.3),(10,0.1,0.6)]
SINIF = {'motor %5':['motor%d_0.95'%i for i in (1,2,3,4)], 'motor %10':['motor%d_0.9'%i for i in (1,2,3,4)],
         'motor %15':['motor%d_0.85'%i for i in (1,2,3,4)], 'motor %30':['motor%d_0.7'%i for i in (1,2,3,4)],
         'volt 12.5':['batt_volt_12.5'], 'res 0.02':['batt_res_0.02']}
KULLAN = ['3 kural(cift,artik)','3b zarf (ham)','7 Mahalanobis (ham)','4 IF ham']
def calis(d, mod, sg, rng):
    d = d.copy()
    for k in M: d[k] = d[k] + rng.normal(0, sg[0], len(d))
    d['voltaj_v'] += rng.normal(0, sg[1], len(d)); d['akim_a'] += rng.normal(0, sg[2], len(d))
    d = hazirla(d); d = d[(d.armed==1)&(d['mod']==mod)&(d.etiket!='hazirlik')]
    nseg = [s for s,g in d.groupby('seg') if g.etiket.iloc[0]=='normal']
    t = d[d.seg_t>=TRIM]; tr = t[t.seg.isin(set(nseg[0::2]))]; tn = t[t.seg.isin(set(nseg[1::2]))]
    mdl = modeller(tr); res = {}
    for dn in KULLAN:
        f = mdl[dn]; r = {'FP': f(tn).mean()}
        for sn, lst in SINIF.items(): r[sn] = f(t[t.etiket.isin(lst)]).mean()
        res[dn] = r
    return pd.DataFrame(res).T
hover, daire = sys.argv[1:3]
H = pd.read_csv(hover); D = pd.read_csv(daire)
for sg in SEVIYE:
    rng = np.random.default_rng(1)
    a = calis(H, 'GUIDED', sg, rng); b = calis(D, 'CIRCLE', sg, rng)
    print(f"\n--- gurultu: PWM s={sg[0]}, V s={sg[1]}, A s={sg[2]}  (hover+daire ortalamasi, satir bazli) ---")
    print(((a + b) / 2).round(2).to_string())