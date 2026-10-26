#!/bin/bash
pkill -f exporter.py 2>/dev/null; pkill -f arducopter 2>/dev/null; pkill -f mavproxy 2>/dev/null; sleep 1
PROJE="$HOME/iha-anomali-tespit"
export PATH="$HOME/.local/bin:$HOME/ardupilot/Tools/autotest:$PATH"

echo "[1/3] Prometheus + Grafana başlatılıyor..."
cd "$PROJE/monitoring" && docker compose up -d

echo "[2/3] Exporter başlatılıyor (arka planda)..."
cd "$PROJE/exporter"
# Operatör ekranına gömülen Grafana skor grafiği (panel-1). Başka bir adres vermek için bu değişkeni
# betiği çalıştırmadan önce ayarla.
export GRAFANA_PANEL_URL="${GRAFANA_PANEL_URL:-http://localhost:3000/d-solo/iha-anomali-model/x?orgId=1&panelId=panel-1&from=now-5m&to=now&refresh=5s&theme=dark}"
"$PROJE/venv/bin/python" exporter.py > "$PROJE/exporter.log" 2>&1 &
EXPORTER_PID=$!
trap 'echo "Exporter durduruluyor..."; kill $EXPORTER_PID 2>/dev/null' EXIT

echo "Operatör ekranı: http://localhost:8088  (SITL bağlandıktan birkaç saniye sonra açılır)"
echo "[3/3] SITL başlatılıyor..."
cd "$HOME/ardupilot/ArduCopter"
# -w: önceki oturumdan kalan parametreleri (ör. jamming ayarı) sıfırlar, her açılış temiz başlar
sim_vehicle.py -w --console --map --out=127.0.0.1:14551 --out=127.0.0.1:14552 --out=127.0.0.1:14553