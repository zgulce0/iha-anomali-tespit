from pymavlink import mavutil

# SITL'e bağlan (MAVProxy'nin yönlendirdiği port)
print("SITL'e bağlanılıyor...")
baglanti = mavutil.mavlink_connection('udpin:127.0.0.1:14551')

# İlk heartbeat'i bekle - bağlantının kurulduğunun kanıtı
baglanti.wait_heartbeat()
print(f"Bağlantı kuruldu! Sistem ID: {baglanti.target_system}, Bileşen ID: {baglanti.target_component}")

print("Paketler dinleniyor... (durdurmak için Ctrl+C)")

while True:
    mesaj = baglanti.recv_match(blocking=True)
    if mesaj is None:
        continue
    
    tip = mesaj.get_type()
    
    if tip == 'HEARTBEAT':
        print(f"[HEARTBEAT] sistem çalışıyor, mod kodu: {mesaj.custom_mode}")
    elif tip == 'BATTERY_STATUS':
        voltaj = mesaj.voltages[0] / 1000.0  # mV -> V
        print(f"[BATTERY_STATUS] Voltaj: {voltaj:.2f}V, Kalan: %{mesaj.battery_remaining}")
    elif tip == 'GLOBAL_POSITION_INT':
        irtifa = mesaj.relative_alt / 1000.0  # mm -> m
        print(f"[POSITION] İrtifa: {irtifa:.2f}m")

