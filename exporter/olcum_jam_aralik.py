from pymavlink import mavutil
import time

SURE = 120  # kaç saniye ölçülecek

m = mavutil.mavlink_connection('udpin:127.0.0.1:14552')
t0 = time.time()
suphe_zamanlari = []
toplam = 0

while time.time() - t0 < SURE:
    g = m.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
    if g is None:
        continue
    toplam += 1
    if g.fix_type < 3 or g.satellites_visible < 6:
        suphe_zamanlari.append(time.time() - t0)

araliklar = [b - a for a, b in zip(suphe_zamanlari, suphe_zamanlari[1:])]
print("toplam paket:", toplam)
print("şüpheli paket:", len(suphe_zamanlari))
if araliklar:
    araliklar.sort()
    print("ortalama aralık (sn):", round(sum(araliklar) / len(araliklar), 2))
    print("en uzun aralık (sn):", round(araliklar[-1], 2))
    i95 = min(len(araliklar) - 1, int(len(araliklar) * 0.95))
    print("%95 aralık (sn):", round(araliklar[i95], 2)) 