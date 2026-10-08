#!/bin/bash
PROJE="$HOME/iha-anomali-tespit"
export PATH="$HOME/.local/bin:$HOME/ardupilot/Tools/autotest:$PATH"

echo "[1/3] Prometheus + Grafana başlatılıyor..."
cd "$PROJE/monitoring" && docker compose up -d

echo "[2/3] Exporter başlatılıyor (arka planda)..."
cd "$PROJE/exporter"
"$PROJE/venv/bin/python" exporter.py > "$PROJE/exporter.log" 2>&1 &
EXPORTER_PID=$!
trap 'echo "Exporter durduruluyor..."; kill $EXPORTER_PID 2>/dev/null' EXIT

echo "[3/3] SITL başlatılıyor..."
cd "$HOME/ardupilot/ArduCopter"
sim_vehicle.py --console --map --out=127.0.0.1:14551
