"""
human_leader.py - OWNER: Ruslan. The human agent's body: the leader arm as a TRACKED tool.

Reads the leader's joint angles ~20x/sec (torque stays OFF, it's moved by hand) and works out
which zone it is in by matching against positions you teach once.

    pip install feetech-servo-sdk pyserial
    python human_leader.py ports
    python human_leader.py teach --port COM6          # walks you through home, 1-5, tray, reject
    python human_leader.py watch --port COM6          # live zone + trigger, check it works

A pick by the human counts when EITHER:
    - the leader squeezes its trigger while in a slot zone (explicit, preferred), OR
    - the leader stays in a slot zone >= dwell_s (config.json) and then leaves it

The interface the agent uses (do not change without telling Aslan):
    LeaderTracker(port, arm_id, zones_path)  - a thread; call .start()
    .zone            -> "home" | "1".."5" | "tray" | "reject" | None (between zones)
    .last_motion_t   -> time.time() of the last movement (used for 'human walked away')
    .pop_click()     -> zone name if the trigger was squeezed since last call, else None
"""
import argparse
import json
import math
import threading
import time
from collections import deque
from pathlib import Path

from servo_bus import ARM_JOINTS, Bus, list_ports

DEFAULT_ZONES = ["home", "1", "2", "3", "4", "5", "tray", "reject"]
THRESHOLD_TICKS = 220      # max distance (over the 5 arm joints) to count as "in" a zone
MOTION_TICKS = 15          # movement smaller than this is noise
TRIGGER_TICKS = 150        # trigger moved this far from rest = squeeze


def dist(a, b):
    return math.sqrt(sum((a[j] - b[j]) ** 2 for j in ARM_JOINTS))


class LeaderTracker(threading.Thread):
    def __init__(self, port, arm_id=None, zones_path="zones.json", threshold=THRESHOLD_TICKS):
        super().__init__(daemon=True)
        self.bus = Bus(port)
        self.bus.torque(False)
        self.zones = json.loads(Path(zones_path).read_text())
        self.threshold = threshold
        self.zone = None
        self.last_motion_t = time.time()
        self.ok = True
        self.pos = None
        self._clicks = deque(maxlen=10)
        self._trigger_rest = None
        self._squeezed = False

    def classify(self, pos):
        best, bd = None, 1e9
        for name, zpos in self.zones.items():
            d = dist(pos, zpos)
            if d < bd:
                best, bd = name, d
        return (best if bd <= self.threshold else None), bd

    def pop_click(self):
        return self._clicks.popleft() if self._clicks else None

    def run(self):
        last = None
        while True:
            try:
                pos = self.bus.read_positions()
                self.ok = True
            except Exception:
                self.ok = False
                time.sleep(0.5)
                continue
            if last and max(abs(pos[j] - last[j]) for j in ARM_JOINTS) > MOTION_TICKS:
                self.last_motion_t = time.time()
            last, self.pos = pos, pos
            self.zone, _ = self.classify(pos)
            # trigger squeeze edge
            if self._trigger_rest is None:
                self._trigger_rest = pos["gripper"]
            squeezed = abs(pos["gripper"] - self._trigger_rest) > TRIGGER_TICKS
            if squeezed and not self._squeezed:
                self._clicks.append(self.zone)
                self.last_motion_t = time.time()
            self._squeezed = squeezed
            time.sleep(0.05)


def teach(port, out, zones):
    bus = Bus(port)
    bus.torque(False)
    data = json.loads(Path(out).read_text()) if Path(out).exists() else {}
    print("Move the leader to each zone and press Enter. Type 'skip' to keep the old one.")
    for z in zones:
        cmd = input(f"  zone '{z}': position the leader, then Enter > ").strip().lower()
        if cmd == "skip" and z in data:
            continue
        data[z] = bus.read_positions()
        print("   saved", {k: int(v) for k, v in data[z].items()})
    Path(out).write_text(json.dumps(data, indent=1))
    print(f"saved {out}. Now run: python human_leader.py watch --port {port}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["ports", "teach", "watch"])
    ap.add_argument("--port")
    ap.add_argument("--zones", default="zones.json")
    ap.add_argument("--only", help="comma list of zones to (re)teach, e.g. 3,tray")
    ap.add_argument("--threshold", type=int, default=THRESHOLD_TICKS)
    a = ap.parse_args()
    if a.cmd == "ports":
        for dev, desc in list_ports():
            print(dev, "-", desc)
    elif a.cmd == "teach":
        teach(a.port, a.zones, a.only.split(",") if a.only else DEFAULT_ZONES)
    else:
        t = LeaderTracker(a.port, None, a.zones, a.threshold)
        t.start()
        while True:
            time.sleep(0.2)
            if t.pos:
                z, d = t.classify(t.pos)
                click = t.pop_click()
                idle = time.time() - t.last_motion_t
                print(f"zone={str(z):7} nearest_dist={d:6.0f}  idle={idle:4.1f}s" + (f"  CLICK in {click}" if click else ""))


if __name__ == "__main__":
    main()
