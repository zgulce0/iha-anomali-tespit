"""
Açıklama kuralının kayıtlı arıza verisinde ölçümü: model alarm verdiğinde "olası neden" doğru mu?

Kullanım (proje kökünden):
  python3 analiz/aciklama_olc.py veri/HOVER.csv veri/DAIRE.csv

Gerçek neden etiketten okunur (motorN_* -> motor N, batt_* -> batarya); açıklama kuralı etiketi GÖRMEZ.
Her segmentin ilk 5 saniyesi atılır (model_egit.py ile aynı geçiş kırpması).
"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'exporter'))
import pandas as pd
from anomali_model import MahalanobisModeli
from aciklama import Aciklayici

KOK = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
TRIM = 5.0
model = MahalanobisModeli.yukle(os.path.join(KOK, 'model', 'mahalanobis_v1.json'))
acik = Aciklayici(model)
OZ = ['motor1_pwm', 'motor2_pwm', 'motor3_pwm', 'motor4_pwm', 'voltaj_v', 'akim_a']


def gercek(etiket):
    if etiket.startswith('motor'):
        return 'motor', int(etiket[5])
    if etiket.startswith('batt'):
        return 'batarya', None
    return 'normal', None


def calis(ad, yol, mod):
    d = pd.read_csv(yol)
    d['seg'] = (d['etiket'] != d['etiket'].shift()).cumsum()
    d['seg_t'] = d['zaman_sn'] - d.groupby('seg')['zaman_sn'].transform('min')
    d = d[(d.armed == 1) & (d['mod'] == mod) & (d.etiket != 'hazirlik') & (d.seg_t >= TRIM)]
    satir = []
    for _, s in d.iterrows():
        x = [float(s[k]) for k in OZ]
        md, esik, alarm = model.degerlendir(mod, x)
        a = acik.acikla(mod, x, alarm, md / esik)
        gt, gn = gercek(s.etiket)
        satir.append({'etiket': s.etiket, 'gercek': gt, 'gercek_motor': gn, 'alarm': alarm,
                      'tur': a['tur'], 'motor': a['motor']})
    t = pd.DataFrame(satir)
    t['dogru'] = (t.alarm & (t.tur == t.gercek) & ((t.gercek != 'motor') | (t.motor == t.gercek_motor)))
    t['belirsiz'] = t.alarm & (t.tur == 'belirsiz')
    t['yanlis'] = t.alarm & ~t.dogru & ~t.belirsiz
    print(f"\n######## {ad} (mod={mod}), {len(t)} satır")
    n = t[t.gercek == 'normal']
    print(f"normal satırlarda alarm: {int(n.alarm.sum())}/{len(n)}")
    a = t[t.gercek != 'normal']
    print(f"arıza satırlarında alarm: {int(a.alarm.sum())}/{len(a)}")
    ozet = a.groupby('gercek').agg(satir=('alarm', 'size'), alarm=('alarm', 'sum'), dogru=('dogru', 'sum'),
                                   belirsiz=('belirsiz', 'sum'), yanlis=('yanlis', 'sum'))
    print(ozet.to_string())
    tab = a.groupby('etiket').agg(satir=('alarm', 'size'), dogru=('dogru', 'sum'),
                                  belirsiz=('belirsiz', 'sum'), yanlis=('yanlis', 'sum'))
    kotu = tab[(tab.dogru != tab.satir)]
    print("tam doğru olmayan etiketler:" if len(kotu) else "her etiketin her satırında neden doğru bulundu")
    if len(kotu):
        print(kotu.to_string())
    return t


if __name__ == '__main__':
    hover, daire = sys.argv[1:3]
    calis('HOVER', hover, 'GUIDED')
    calis('DAIRE', daire, 'CIRCLE')
