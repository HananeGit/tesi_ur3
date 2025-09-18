#!/usr/bin/env bash
set -e
CSV="$HOME/tesi_ur3/results/E4_planner.csv"
mkdir -p "$(dirname "$CSV")"
[ -s "$CSV" ] || echo "planner,ostacolo,planning_time_s,success" > "$CSV"
echo "Inserisci righe (invio su 'planner' per finire). Esempi:"
echo "  planner = RRTConnect | RRTStar"
echo "  ostacolo = si | no"
echo "  planning_time_s = 0.123"
echo "  success = 1|0"
while true; do
  read -p "planner: " P; [ -z "$P" ] && break
  read -p "ostacolo (si/no): " O
  read -p "planning_time_s: " T
  read -p "success (1/0): " S
  echo "$P,$O,$T,$S" >> "$CSV"
  echo "OK: $P,$O,$T,$S"
done
echo "Salvato in: $CSV"
