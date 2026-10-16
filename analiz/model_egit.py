"""
Normal ucus verisinden Mahalanobis modelini egitir ve model/mahalanobis_v1.json olarak kaydeder.

Kullanim (repo kokunden):
  venv/bin/python analiz/model_egit.py HOVER.csv DAIRE.csv BAGIMSIZ_NORMAL.csv

Model = her ucus rejimi (hover, daire) icin 3 sey:
  1) ortalama        : normal ucusta 6 sayinin (4 motor, voltaj, akim) tipik degeri
  2) ters kovaryans  : sayilarin birbiriyle nasil birlikte oynadigi (bulutun sekli)
  3) esik            : "bundan uzaktaysa tuhaf" siniri = egitimdeki en uzak normal noktanin 1,5 kati

Egitime SADECE normal satirlar girer. Ucuncu dosya (baska gunun normal ucusu) egitime girmez,
sadece sinav icin: modelin hic gormedigi normal ucusta yanlis alarm veriyor mu?
"""
import os
import sys
import json
import datetime
import numpy as np
import pandas as pd

KOK = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(KOK, 'exporter'))
from anomali_model import MahalanobisModeli

OZELLIKLER = ['motor1_pwm', 'motor2_pwm', 'motor3_pwm', 'motor4_pwm', 'voltaj_v', 'akim_a']
COZUNURLUK = [1.0, 1.0, 1.0, 1.0, 0.01, 0.05]      # sensorun olcebildigi en kucuk adim
REJIMLER = {'hover': ['GUIDED', 'LOITER'], 'daire': ['CIRCLE']}
KIRPMA_SN = 5.0        # her etiket degisiminden sonraki ilk 5 sn alinmaz (gecis anlari)
ESIK_CARPANI = 1.5
CIKTI = os.path.join(KOK, 'model', 'mahalanobis_v1.json')


def segmentle(d):
    d = d.copy()
    d['seg'] = (d['etiket'] != d['etiket'].shift()).cumsum()
    d['seg_t'] = d['zaman_sn'] - d.groupby('seg')['zaman_sn'].transform('min')
    return d


def temiz_satirlar(d, modlar):
    d = d[(d['armed'] == 1) & (d['mod'].isin(modlar)) & (d['etiket'] != 'hazirlik')]
    d = d[d['seg_t'] >= KIRPMA_SN]
    return d.dropna(subset=OZELLIKLER)


def egit(normal_satirlar):
    X = normal_satirlar[OZELLIKLER].values.astype(float)
    ort = X.mean(axis=0)
    C = np.cov(X.T) + np.diag(np.array(COZUNURLUK) ** 2)
    P = np.linalg.inv(C)
    D = X - ort
    mesafeler = np.sqrt(np.einsum('ij,jk,ik->i', D, P, D))
    return ort, P, ESIK_CARPANI * mesafeler.max(), mesafeler.max(), len(X)


def main():
    hover_csv, daire_csv, bagimsiz_csv = sys.argv[1:4]
    hover = segmentle(pd.read_csv(hover_csv))
    daire = segmentle(pd.read_csv(daire_csv))

    model = {'versiyon': 1, 'tarih': datetime.date.today().isoformat(),
             'ozellikler': OZELLIKLER, 'cozunurluk': COZUNURLUK,
             'esik_carpani': ESIK_CARPANI, 'kirpma_sn': KIRPMA_SN, 'rejimler': {}}
    for ad, d in (('hover', hover), ('daire', daire)):
        normal = temiz_satirlar(d[d['etiket'] == 'normal'], REJIMLER[ad])
        ort, P, esik, md_max, n = egit(normal)
        model['rejimler'][ad] = {'modlar': REJIMLER[ad], 'egitim_satiri': n,
                                 'ortalama': ort.tolist(), 'ters_kovaryans': P.tolist(),
                                 'egitimdeki_en_uzak': float(md_max), 'esik': float(esik)}
        print(f"[{ad}] egitim satiri {n} | egitimdeki en uzak mesafe {md_max:.2f} | esik {esik:.2f}")

    os.makedirs(os.path.dirname(CIKTI), exist_ok=True)
    with open(CIKTI, 'w', encoding='utf-8') as f:
        json.dump(model, f, indent=1, ensure_ascii=False)
    print("Model kaydedildi:", CIKTI)

    # ---- SINAV: kaydedilen dosyadan yukleyip, canli kullanimdaki kodla puanla ----
    m = MahalanobisModeli.yukle(CIKTI)

    def puanla(satirlar):
        sonuc = []
        for mod, *x in satirlar[['mod'] + OZELLIKLER].itertuples(index=False, name=None):
            r = m.degerlendir(mod, [float(v) for v in x])
            if r is not None:
                sonuc.append(r)
        return sonuc

    print("\n=== SINAV 1: modelin hic gormedigi normal ucus (15 Ekim) -> yanlis alarm var mi? ===")
    eski = pd.read_csv(bagimsiz_csv)
    eski['grp'] = (eski['mod'] != eski['mod'].shift()).cumsum()
    eski['t'] = eski['zaman_sn'] - eski.groupby('grp')['zaman_sn'].transform('min')
    eski = eski[(eski['armed'] == 1) & (eski['t'] >= 10)].dropna(subset=OZELLIKLER)
    for mod in ('LOITER', 'CIRCLE'):
        r = puanla(eski[eski['mod'] == mod])
        if r:
            print(f"{mod:7} {len(r):4d} satir | yanlis alarm {sum(a for _, _, a in r)} | "
                  f"en uzak mesafe {max(md for md, _, _ in r):.2f} (esik {r[0][1]:.2f})")

    print("\n=== SINAV 2: arizali ucuslar -> kac arizali saniye yakalandi? ===")
    for ad, d in (('hover', hover), ('daire', daire)):
        ar = temiz_satirlar(d[d['etiket'] != 'normal'], REJIMLER[ad])
        zayif = None
        top = yak = 0
        for et, g in ar.groupby('etiket'):
            r = puanla(g)
            a = sum(x for _, _, x in r)
            top += len(r); yak += a
            oran = float(np.median([md / es for md, es, _ in r]))    # esigin kac kati
            if zayif is None or oran < zayif[1]:
                zayif = (et, oran)
        print(f"{ad:6} {yak}/{top} arizali satir yakalandi (%{100 * yak / top:.1f}) | "
              f"en zor ariza: {zayif[0]}, mesafesi esigin {zayif[1]:.1f} kati")


if __name__ == '__main__':
    main()