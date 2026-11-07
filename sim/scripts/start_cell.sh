#!/usr/bin/env bash
# Start whatever part of the KitMesh packing cell is not already running:
# Isaac Sim (GUI on :0), the two observation-only robot agents, and the demo page.
# Safe to run again: running pieces are left alone. Logs go to logs/.
set -uo pipefail
ROOT="${KITMESH_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
LOGS="$ROOT/logs"
export DISPLAY_FOR_SIM=:0
export XAUTH_FOR_SIM=/run/user/1000/gdm/Xauthority
mkdir -p "$LOGS" "$ROOT/data/station-a" "$ROOT/data/station-b"

listening() { ss -ltn 2>/dev/null | grep -q "127.0.0.1:$1 "; }

if ! nvidia-smi >/dev/null 2>&1; then
  echo "nvidia-smi fails. Run: sudo modprobe nvidia   then run this script again."
  exit 1
fi

if listening 9401; then
  echo "Isaac Sim: already running (port 9401)"
else
  echo "Isaac Sim: starting (about 30-60 s)…"
  [[ -f "$LOGS/sim.log" ]] && cp "$LOGS/sim.log" "$ROOT/artifacts/sim.$(date +%Y%m%d-%H%M%S).previous.log"
  nohup bash -c "source ${ISAAC_ACTIVATE:-/home/cal/isaac/activate.sh} && export DISPLAY=$DISPLAY_FOR_SIM XAUTHORITY=$XAUTH_FOR_SIM && cd $ROOT && exec python -m kitmesh.sim_worker --gui --no-snapshots --port 9401" \
    >"$LOGS/sim.log" 2>&1 </dev/null &
  disown
fi

start_agent() {
  local id=$1 http=$2
  if listening "$http"; then
    echo "$id agent: already running (http $http)"
    return
  fi
  echo "$id agent: starting (observes the table only; no link to the other robot)"
  (cd "$ROOT" && PYTHONPATH=$ROOT nohup /usr/bin/python3 -m kitmesh.cell_agent --robot "$id" --http-port "$http" \
    --sim-port 9401 --data "$ROOT/data/cell-$id" >"$LOGS/agent-$id.log" 2>&1 </dev/null &
   disown)
}
start_agent station-a 9201
start_agent station-b 9202

if listening 8080; then
  echo "Demo page: already running on 127.0.0.1:8080"
else
  echo "Demo page: starting on 127.0.0.1:8080"
  # Also serve on this machine's Tailscale address, if it has one (private to your tailnet).
  TS_IP="$(tailscale ip -4 2>/dev/null | head -1)"
  EXTRA=()
  [[ -n "$TS_IP" ]] && EXTRA=(--bind "$TS_IP")
  (cd "$ROOT" && PYTHONPATH=$ROOT nohup /usr/bin/python3 -m kitmesh.demo --port 8080 --bind 127.0.0.1 "${EXTRA[@]}" \
    --sim-log "$LOGS/sim.log" >"$LOGS/demo.log" 2>&1 </dev/null & disown)
fi

for _ in $(seq 1 60); do
  grep -qE 'KITMESH_SIM_READY|KITMESH_SIM_FAILED' "$LOGS/sim.log" 2>/dev/null && break
  sleep 3
done
grep -E '^KITMESH_SIM_(READY|FAILED)' "$LOGS/sim.log" | tail -1
# Keep :0 from blanking while the demo runs (the page's video films that screen).
# Same mechanism a video player uses; stop it with: pkill -f "gnome-session-inhibit --inhibit idle"
if ! pgrep -f "gnome-session-inhibit --inhibit idle" >/dev/null; then
  DISPLAY=:0 XAUTHORITY=$XAUTH_FOR_SIM DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus XDG_RUNTIME_DIR=/run/user/1000 \
    nohup gnome-session-inhibit --inhibit idle --reason "KitMesh demo: keep the Isaac Sim display awake" --inhibit-only \
    >"$LOGS/inhibit.log" 2>&1 </dev/null &
  disown
  echo "Display: idle blanking held off while the demo runs"
fi
echo "Demo page: http://127.0.0.1:8080$([[ -n "$(tailscale ip -4 2>/dev/null)" ]] && echo " or http://$(tailscale ip -4 | head -1):8080 (tailnet)")"
