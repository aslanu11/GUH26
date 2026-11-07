#!/usr/bin/env python3
"""
agent.py - OWNER: Aslan. Neptune North fulfilment cell: one agent process per actor (robot or human).

No central controller. Every agent:
  - gossips its full state (orders + pick registers + status) to its peers over UDP, ~4x/sec
  - merges what it hears (CRDT: done is final, otherwise highest (clock, writer) wins)
  - runs the SAME allocation rule (distance + weight + queue) on its own view, so connected
    agents agree on who does what without negotiating
  - detects peers going silent and decides: online / silent / cut off (still moving) / down
  - serves its own web page: what IT believes, an order box, a "cut comms" button

Stdlib only. Run `python agent.py --help`.
"""
import argparse
import json
import random
import socket
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

FINAL = ("done", "missing")
DEFAULT_REG = {"status": "pending", "owner": None, "clock": 0, "w": ""}
KIND_RANK = {"robot": 0, "human": 1}  # ties go to the robot


def winner(a, b):
    """CRDT merge for one pick register. Final beats non-final; else highest (clock, writer)."""
    fa, fb = a["status"] in FINAL, b["status"] in FINAL
    if fa != fb:
        return a if fa else b
    return a if (a["clock"], a["w"]) >= (b["clock"], b["w"]) else b


# --------------------------------------------------------------------------------------
# Executors: what the agent's body does. Real ones live in arms.py.
# --------------------------------------------------------------------------------------
class FakeRobot:
    """Pretends to pick. Use to develop without the arm. 'Simulate arm failure' on the page."""

    def __init__(self):
        self.arm_ok = True
        self.fail_now = False
        self.miss_slots = set()   # (sim) slots where the gripper will close on nothing

    def execute(self, slot, should_pause, should_abort):
        if slot in self.miss_slots:
            time.sleep(1.0)
            return "missed"
        dur, t = random.uniform(4.0, 6.0), 0.0
        while t < dur:
            if self.fail_now:
                self.fail_now = False
                self.arm_ok = False
                return "failed"
            if should_abort():
                return "aborted"
            if should_pause():
                time.sleep(0.1)
                continue
            time.sleep(0.1)
            t += 0.1
        return "ok"

    def reconnect(self):
        self.arm_ok = True
        return True


class FakeHuman:
    """Pretends to be a human using the leader arm. Does its assigned pick after 3-6 s."""

    def __init__(self):
        self.zone = None
        self.last_motion_t = time.time()
        self.paused = False  # 'walked away' simulation toggled from the page
        self._task = None

    def assign(self, slot):
        self._task = (slot, time.time() + random.uniform(3.0, 6.0)) if slot is not None else None

    def poll(self):
        """Returns a slot number when the fake human finishes touching it, else None."""
        now = time.time()
        if self.paused:
            self.zone = "home"
            return None
        self.last_motion_t = now
        if not self._task:
            self.zone = "home"
            return None
        slot, due = self._task
        self.zone = str(slot)
        if now >= due:
            self._task = None
            self.zone = "home"
            return slot
        return None


# --------------------------------------------------------------------------------------
class Agent:
    def __init__(self, cfg, me, port, peers, executor, watcher=None, watch_peer=None):
        self.cfg = cfg
        self.me = me
        self.kind = cfg["agents"][me]["kind"]
        self.ex = executor
        self.watcher = watcher          # camera.MotionWatcher or None
        self.watch_peer = watch_peer    # which peer the camera watches
        self.lock = threading.RLock()
        self.epoch, self.clock, self.order_counter = 0, 0, 0
        self.orders, self.picks = {}, {}
        self.peers = peers
        self.peer_status = {}           # agent id -> {"t": recv time, "status": {...}}
        self.comms_cut = False
        self.available = True           # manual toggle / auto 'away' / arm failure
        self.away_auto = False
        self.busy_pid = None
        self.target_slot = None
        self.abort_flag = False
        self.log = deque(maxlen=40)
        self.last_view, self.last_assign, self.last_reasons = {}, {}, {}
        self.danger = None              # human agent: {"slot": n} while the hand is in the robot's slot
        self.robot_target = None        # human agent: where the robot is working (from its heartbeat)
        self.view_url = None            # camera stream shown on this agent's page
        # learned teammate model: my own measured time per pick (EMA), shared in my heartbeat
        self.cycle_est = float(cfg["agents"][me]["cycle_s"])
        self.n_picks = 0
        self.claim_t = {}
        self.last_why = {}
        self.my_reason = {}
        self._dwell_zone, self._dwell_since = None, 0.0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))

    # ---------------- helpers ----------------
    def say(self, msg):
        self.log.appendleft(f"{time.strftime('%H:%M:%S')}  {msg}")
        print(f"[{self.me}] {msg}", flush=True)

    def reg(self, pid):
        return self.picks.get(pid, DEFAULT_REG)

    def write(self, pid, status, owner, **extra):
        if self.reg(pid)["status"] in FINAL:
            return
        self.clock += 1
        self.picks[pid] = {"status": status, "owner": owner, "clock": self.clock, "w": self.me, **extra}

    def slot_of(self, pid):
        return int(pid.rsplit(":", 1)[1])

    def weight_of(self, pid):
        """Each order carries its parcels' weights (drawn when the order is created), like a manifest."""
        o = self.orders.get(pid.rsplit(":", 1)[0], {})
        slot = str(self.slot_of(pid))
        return int(o.get("weights", {}).get(slot, self.cfg["slots"][slot]["weight"]))

    def ordered_pids(self):
        out = []
        for o in sorted(self.orders.values(), key=lambda o: (o["seq"], o["id"])):
            out += [f"{o['id']}:{s}" for s in o["slots"]]
        return out

    def status(self):
        return {
            "kind": self.kind,
            "available": self.available,
            "busy": self.busy_pid,
            "target": self.target_slot,
            "zone": getattr(self.ex, "zone", None),
            "cycle": round(self.cycle_est, 2),
            "n": self.n_picks,
        }

    # ---------------- networking ----------------
    def snapshot_msg(self):
        return json.dumps({
            "from": self.me, "epoch": self.epoch, "clock": self.clock,
            "orders": self.orders, "picks": self.picks, "status": self.status(),
        }).encode()

    def send_loop(self):
        while True:
            if not self.comms_cut:
                with self.lock:
                    data = self.snapshot_msg()
                for p in self.peers:
                    try:
                        self.sock.sendto(data, p)
                    except OSError:
                        pass
            time.sleep(self.cfg["heartbeat_s"])

    def recv_loop(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
                msg = json.loads(data)
            except (OSError, ValueError):
                continue
            if self.comms_cut or msg.get("from") == self.me:
                continue
            with self.lock:
                self.merge(msg)

    def merge(self, msg):
        src = msg["from"]
        if src not in self.cfg["agents"]:
            return
        prev = self.peer_status.get(src)
        if prev is None or time.time() - prev["t"] > self.cfg["suspect_s"]:
            self.say(f"hearing {src} again" if prev else f"found {src}")
        self.peer_status[src] = {"t": time.time(), "status": msg["status"]}
        if msg["epoch"] < self.epoch:
            return
        if msg["epoch"] > self.epoch:
            self.say(f"reset by {src} (run {msg['epoch']})")
            self.epoch, self.orders, self.picks = msg["epoch"], {}, {}
            self.abort_flag = self.busy_pid is not None
            self.busy_pid = None
            if self.kind == "human":
                self._assign_human(None)
        self.clock = max(self.clock, msg["clock"])
        for oid, o in msg["orders"].items():
            self.orders.setdefault(oid, o)
        for pid, r in msg["picks"].items():
            mine = self.picks.get(pid)
            self.picks[pid] = r if mine is None else winner(mine, r)

    # ---------------- who is alive, from MY point of view ----------------
    def view(self):
        now, v, c = time.time(), {}, self.cfg
        for a in c["agents"]:
            if a == self.me:
                v[a] = {"alive": True, "fresh": True, "available": self.available, "state": "me"}
                continue
            ps = self.peer_status.get(a)
            if ps is None:
                v[a] = {"alive": False, "fresh": False, "available": False, "state": "never seen"}
                continue
            age = now - ps["t"]
            avail = ps["status"].get("available", True)
            if age < c["suspect_s"]:
                v[a] = {"alive": True, "fresh": True, "available": avail, "state": "online"}
                continue
            moving = self.watcher.moving() if (self.watcher and a == self.watch_peer) else None
            if moving is True:
                st, alive = "cut off (camera: still moving)", True
            elif moving is False and age > c["suspect_s"] + 3:
                st, alive = "down (camera: not moving)", False
            elif moving is None and age > c["down_s"]:
                st, alive = f"down (silent {age:.0f}s)", False
            else:
                st, alive = f"silent {age:.1f}s", True
            v[a] = {"alive": alive, "fresh": False, "available": avail, "state": st}
        return v

    # ---------------- the shared rule: distance + weight + queue ----------------
    def cycle_of(self, a):
        """Learned seconds-per-pick for agent a: mine is measured here, theirs comes from their heartbeat."""
        if a == self.me:
            return self.cycle_est
        ps = self.peer_status.get(a)
        return float(ps["status"].get("cycle", self.cfg["agents"][a]["cycle_s"])) if ps else float(self.cfg["agents"][a]["cycle_s"])

    def learn(self, pid):
        """A pick I claimed is done: update my seconds-per-pick estimate (exponential moving average)."""
        t0 = self.claim_t.pop(pid, None)
        if t0 is None:
            return
        dt = min(60.0, max(1.0, time.time() - t0))
        a = self.cfg.get("learn_alpha", 0.4)
        old = self.cycle_est
        self.cycle_est = (1 - a) * self.cycle_est + a * dt
        self.n_picks += 1
        self.say(f"learned: that pick took {dt:.1f}s -> my pick time {old:.1f}s -> {self.cycle_est:.1f}s")

    def cost(self, a, slot, queue, weight):
        ag, s = self.cfg["agents"][a], self.cfg["slots"][str(slot)]
        c = s["dist"][a] / ag["speed"] + queue[a] * self.cycle_of(a)
        if ag["kind"] == "human":
            c += self.cfg["weight_cost_k"] * weight
        return c

    def allocate(self, v):
        c = self.cfg
        assign, reasons, why = {}, {}, {}
        queue = {a: 0 for a in c["agents"]}
        for pid in self.ordered_pids():
            r = self.reg(pid)
            if r["status"] == "claimed" and r["owner"] in v and v[r["owner"]]["alive"]:
                queue[r["owner"]] += 1
        orders = {o["id"]: o for o in self.orders.values()}
        seq = {o["id"]: o["seq"] for o in self.orders.values()}
        # decide the heaviest parcels first (within each order), so heavy ones are matched before queues fill
        heavy_first = sorted(self.ordered_pids(), key=lambda p: (seq[p.rsplit(":", 1)[0]], -self.weight_of(p), self.slot_of(p)))
        for pid in heavy_first:
            r = self.reg(pid)
            if r["status"] in FINAL:
                continue
            if r["status"] == "claimed" and r["owner"] in v and v[r["owner"]]["alive"]:
                assign[pid] = r["owner"]
                why[pid] = self.my_reason.get(pid, f"claimed by {r['owner']}") if r["owner"] == self.me else f"claimed by {r['owner']}"
                continue
            slot = self.slot_of(pid)
            weight = self.weight_of(pid)
            among = orders[pid.rsplit(":", 1)[0]]["among"]
            cands, excluded = [], []
            skip = r.get("skip", [])
            for a, ag in c["agents"].items():
                x = v[a]
                if a in skip:
                    excluded.append(f"{a} missed it twice")
                    continue
                if not x["alive"]:
                    excluded.append(f"{a} down")
                    continue
                if not x["available"]:
                    excluded.append(f"{a} unavailable")
                    continue
                if not (x["fresh"] or a in among):
                    excluded.append(f"{a} cut off")
                    continue
                if slot not in ag["reach"]:
                    excluded.append(f"{a} can't reach")
                    continue
                if ag["kind"] == "human" and weight > c["human_max_weight"]:
                    excluded.append(f"{a}: weight {weight} > limit {c['human_max_weight']}")
                    continue
                cands.append(a)
            if not cands:
                assign[pid] = None
                heavy = weight > c["human_max_weight"]
                reasons[pid] = (("blocked: robot missed it twice: reposition it, then press Retry robot" if skip else
                                 f"blocked: weight {weight} needs a robot") if heavy else "blocked: no agent available")
                continue
            costs = {a: self.cost(a, slot, queue, weight) for a in cands}
            best = min(cands, key=lambda a: (costs[a], KIND_RANK[c["agents"][a]["kind"]], a))
            assign[pid] = best
            parts = [f"{a} {costs[a]:.1f}" for a in sorted(cands, key=lambda a: costs[a])]
            why[pid] = f"{best}: score " + " vs ".join(parts) + ("; " + ", ".join(excluded) if excluded else "")
            queue[best] += 1
        return assign, reasons, why

    # ---------------- decision loop ----------------
    def tick_loop(self):
        while True:
            with self.lock:
                self.tick()
            time.sleep(0.1)

    def tick(self):
        v = self.view()
        self.last_view = v
        self.last_assign, self.last_reasons, self.last_why = self.allocate(v)

        # my current pick got done by someone else, or my claim was overridden -> drop it
        if self.busy_pid and not self.abort_flag:
            r = self.reg(self.busy_pid)
            if r["status"] in FINAL or r["owner"] != self.me:
                who = r["w"] or "someone"
                self.say(f"slot {self.slot_of(self.busy_pid)} was taken by {who} - replanning")
                self.abort_flag = True
                if self.kind == "human":
                    self.busy_pid = None
                    self._assign_human(None)

        if self.kind == "human":
            self._human_tick()
            self._safety_tick()

        if self.busy_pid is None and self.available and getattr(self.ex, "arm_ok", True):
            sign = 1 if self.kind == "human" else -1   # human: lightest first, robot: heaviest first
            now = time.time()
            mine = sorted(self.ordered_pids(), key=lambda p: (not self.reg(p).get("skip"), sign * self.weight_of(p)))
            for pid in mine:
                if self.orders[pid.rsplit(":", 1)[0]].get("start_at", 0) > now:
                    continue   # countdown: let the human read their picks first
                if self.last_assign.get(pid) == self.me:
                    self.my_reason[pid] = self.last_why.get(pid, "")
                    self.claim_t[pid] = time.time()
                    skip = self.reg(pid).get("skip")
                    self.write(pid, "claimed", self.me, **({"skip": skip} if skip else {}))
                    self.busy_pid = pid
                    self.target_slot = self.slot_of(pid)
                    self.abort_flag = False
                    self.say(f"claimed slot {self.target_slot} ({self.my_reason[pid]})")
                    if self.kind == "robot":
                        threading.Thread(target=self._robot_run, args=(pid,), daemon=True).start()
                    else:
                        self._assign_human(self.target_slot)
                    break

    # ---------------- robot body ----------------
    def should_pause(self):
        """Speed-and-separation: hold still while the human is in the slot I'm working."""
        with self.lock:
            for a, ps in self.peer_status.items():
                if self.cfg["agents"][a]["kind"] != "human":
                    continue
                if time.time() - ps["t"] < self.cfg["suspect_s"]:
                    if self.target_slot is not None and ps["status"].get("zone") == str(self.target_slot):
                        return True
        return False

    def should_abort(self):
        return self.abort_flag

    def _robot_run(self, pid):
        slot = self.slot_of(pid)
        paused_logged = [False]

        def pause_cb():
            p = self.should_pause()
            if p and not paused_logged[0]:
                self.say(f"human in slot {slot} - pausing")
            if not p and paused_logged[0]:
                self.say("human clear - resuming")
            paused_logged[0] = p
            return p

        res = "failed"
        for attempt in (1, 2):
            try:
                res = self.ex.execute(slot, pause_cb, self.should_abort)
            except Exception as e:  # serial errors, power pulled, etc.
                res = "failed"
                self.say(f"arm error: {e}")
            if res != "missed":
                break
            if attempt == 1:
                self.say(f"gripper closed on nothing at slot {slot} - trying again")
            if self.abort_flag:
                break
        with self.lock:
            if res == "ok":
                self.write(pid, "done", self.me)
                self.say(f"done slot {slot}")
                self.learn(pid)
            elif res == "empty":
                self.write(pid, "missing", self.me)
                self.say(f"slot {slot} was empty")
            elif res == "missed":
                self.write(pid, "pending", None, skip=[self.me])
                self.say(f"missed slot {slot} twice - back home, handing it to the human")
            elif res == "failed":
                self.available = False
                if self.reg(pid)["owner"] == self.me:
                    self.write(pid, "pending", None)
                self.say("ARM FAILED - released my pick, marked myself unavailable")
            else:
                self.say(f"aborted slot {slot}")
            if self.busy_pid == pid:
                self.busy_pid = None
                self.target_slot = None

    # ---------------- human body ----------------
    def _assign_human(self, slot):
        if hasattr(self.ex, "assign"):
            self.ex.assign(slot)
        if slot is None:
            self.target_slot = None

    def human_touched(self, slot):
        """Human arm dwelt in a slot then left = they took that item (assigned to them or not)."""
        now = time.time()
        if getattr(self, "_last_touch", (None, 0))[0] == slot and now - self._last_touch[1] < 3.0:
            return  # same touch reported twice (trigger + dwell)
        self._last_touch = (slot, now)
        cands = [p for p in self.ordered_pids() if self.slot_of(p) == slot and self.reg(p)["status"] not in FINAL]
        if not cands:
            self.say(f"human touched slot {slot} (no open pick)")
            return
        pid = self.busy_pid if self.busy_pid in cands else cands[0]
        prev_owner = self.reg(pid)["owner"]
        self.write(pid, "done", self.me)
        note = "" if prev_owner in (None, self.me) else f" (was {prev_owner}'s)"
        self.say(f"human took slot {slot}{note}")
        if pid == self.busy_pid:
            self.learn(pid)
            self.busy_pid = None
            self._assign_human(None)

    def _safety_tick(self):
        """Warn the human when their hand is in the slot the robot is working in."""
        target = None
        for a, ps in self.peer_status.items():
            if self.cfg["agents"][a]["kind"] == "robot" and time.time() - ps["t"] < self.cfg["suspect_s"]:
                if ps["status"].get("busy") and ps["status"].get("target") is not None:
                    target = ps["status"]["target"]
        self.robot_target = target
        zone = getattr(self.ex, "zone", None)
        danger = target is not None and zone == str(target)
        if danger and not self.danger:
            self.say(f"WARNING: hand in slot {target} where the robot is working")
        self.danger = {"slot": target} if danger else None
        if hasattr(self.ex, "robot_target"):
            self.ex.robot_target, self.ex.danger = target, danger

    def _human_tick(self):
        now, c = time.time(), self.cfg
        # completion: dwell in a slot zone, then leave it
        if hasattr(self.ex, "poll"):  # fake human
            s = self.ex.poll()
            if s is not None:
                self.human_touched(s)
        else:
            click = self.ex.pop_click() if hasattr(self.ex, "pop_click") else None
            if click and click.isdigit():
                self.human_touched(int(click))
            z = self.ex.zone
            if z != self._dwell_zone:
                if self._dwell_zone and self._dwell_zone.isdigit() and now - self._dwell_since >= c["dwell_s"]:
                    self.human_touched(int(self._dwell_zone))
                self._dwell_zone, self._dwell_since = z, now
        # walked away: no leader movement while holding a pick
        idle = now - self.ex.last_motion_t
        if self.available and self.busy_pid and idle > c["away_s"]:
            self.available, self.away_auto = False, True
            self.write(self.busy_pid, "pending", None)
            self.say(f"human idle {idle:.0f}s - away, released slot {self.slot_of(self.busy_pid)}")
            self.busy_pid = None
            self._assign_human(None)
        elif self.away_auto and idle < 1.0:
            self.available, self.away_auto = True, False
            self.say("human back")

    # ---------------- page actions ----------------
    def create_order(self, slots):
        slots = sorted({int(s) for s in slots})
        bad = [s for s in slots if str(s) not in self.cfg["slots"]]
        if bad or not slots:
            return f"unknown slots {bad}" if bad else "no slots"
        used = {self.slot_of(p) for p in self.ordered_pids()}
        clash = [s for s in slots if s in used]
        if clash:
            return f"slots {clash} already used this run - press Reset to restock"
        v = self.view()
        among = sorted(a for a, x in v.items() if x["alive"] and x["fresh"])
        self.clock += 1
        self.order_counter += 1
        oid = f"{self.me}{self.epoch}-{self.order_counter}"
        order = {"id": oid, "slots": slots, "seq": self.clock, "among": among, "by": self.me,
                 "start_at": time.time() + float(self.cfg.get("start_delay_s", 0))}
        if self.cfg.get("random_weights"):
            lo, hi = self.cfg.get("weight_range", [1, 10])
            order["weights"] = {str(s): random.randint(lo, hi) for s in slots}
        self.orders[oid] = order
        w = order.get("weights")
        self.say(f"order {oid}: slots {slots}" + (f" weights {w}" if w else "") + f" (shared with {among})")
        return "ok"

    def reset(self):
        self.epoch += 1
        self.orders, self.picks = {}, {}
        self.abort_flag = self.busy_pid is not None
        self.busy_pid = None
        if self.kind == "human":
            self._assign_human(None)
        self.say(f"RESET -> run {self.epoch}")

    def state_json(self):
        v = self.last_view
        rows = []
        for pid in self.ordered_pids():
            r, slot = self.reg(pid), self.slot_of(pid)
            rows.append({
                "pid": pid, "slot": slot, "weight": self.weight_of(pid),
                "status": r["status"], "owner": r["owner"], "by": r["w"],
                "assigned": self.last_assign.get(pid), "reason": self.last_reasons.get(pid),
                "why": self.last_why.get(pid), "skip": r.get("skip", []),
            })
        my_next = None
        if self.busy_pid:
            s = self.slot_of(self.busy_pid)
            my_next = {"slot": s, "weight": self.weight_of(self.busy_pid)}
        return {
            "me": self.me, "kind": self.kind, "epoch": self.epoch, "comms_cut": self.comms_cut,
            "available": self.available, "arm_ok": getattr(self.ex, "arm_ok", True),
            "zone": getattr(self.ex, "zone", None), "paused": self.kind == "robot" and self.busy_pid is not None and self.should_pause(),
            "my_next": my_next, "view": v, "rows": rows, "log": list(self.log),
            "peer_zone": {a: ps["status"].get("zone") for a, ps in self.peer_status.items()},
            "human_max_weight": self.cfg["human_max_weight"],
            "slots": sorted(int(s) for s in self.cfg["slots"]),
            "danger": self.danger, "robot_target": self.robot_target,
            "countdown": max([0.0] + [o.get("start_at", 0) - time.time() for o in self.orders.values()]),
            "my_picks": [{"slot": self.slot_of(p), "weight": self.weight_of(p)} for p in
                         sorted(self.ordered_pids(), key=lambda p: self.weight_of(p) * (1 if self.kind == "human" else -1))
                         if self.last_assign.get(p) == self.me and self.reg(p)["status"] not in FINAL],
            "handoff": [self.slot_of(p) for p in self.ordered_pids()
                        if self.reg(p).get("skip") and self.reg(p)["status"] not in FINAL],
            "learned": {a: {"cycle": round(self.cycle_of(a), 1),
                            "n": self.n_picks if a == self.me else (self.peer_status.get(a, {}).get("status", {}).get("n", 0))}
                        for a in self.cfg["agents"]},
            "view_url": self.view_url or ("/video" if hasattr(self.ex, "jpeg") else None),
            "fake": type(self.ex).__name__.startswith("Fake"),
            "walked_away": getattr(self.ex, "paused", False),
        }

    def handle_post(self, path, body):
        with self.lock:
            if path == "/order":
                raw = body.get("slots", "")
                parts = raw if isinstance(raw, list) else str(raw).replace(" ", ",").split(",")
                try:
                    return self.create_order([p for p in parts if str(p).strip()])
                except ValueError:
                    return "slots must be numbers, e.g. 1,3,4"
            if path == "/comms":
                self.comms_cut = bool(body.get("cut"))
                self.say("COMMS CUT" if self.comms_cut else "comms restored")
                return "ok"
            if path == "/available":
                self.available = bool(body.get("value"))
                self.away_auto = False
                if not self.available and self.busy_pid:
                    if self.kind == "robot":
                        self.abort_flag = True
                    self.write(self.busy_pid, "pending", None)
                    self.busy_pid = None
                    self._assign_human(None)
                self.say("available" if self.available else "marked unavailable")
                return "ok"
            if path == "/walkaway":  # fake human: stop moving, auto-away kicks in after away_s
                if hasattr(self.ex, "paused"):
                    self.ex.paused = bool(body.get("value"))
                    self.say("(sim) human walked away" if self.ex.paused else "(sim) human returns")
                return "ok"
            if path == "/touch":  # manual 'I took slot N' button for the human
                self.human_touched(int(body["slot"]))
                return "ok"
            if path == "/retry":  # clear 'robot missed it' so the robot tries again
                pid = body.get("pid")
                if pid in self.picks and self.reg(pid)["status"] not in FINAL:
                    self.write(pid, "pending", None)
                    self.say(f"retry: slot {self.slot_of(pid)} open to the robot again")
                return "ok"
            if path == "/miss":   # fake robot: simulate empty grasps on a slot
                if hasattr(self.ex, "miss_slots"):
                    self.ex.miss_slots.add(int(body["slot"]))
                return "ok"
            if path == "/fail":   # fake robot: simulate pulling power
                if hasattr(self.ex, "fail_now"):
                    self.ex.fail_now = True
                return "ok"
            if path == "/armreset":
                ok = self.ex.reconnect() if hasattr(self.ex, "reconnect") else True
                if ok:
                    self.available = True
                self.say("arm reconnected" if ok else "arm reconnect FAILED")
                return "ok" if ok else "reconnect failed"
            if path == "/reset":
                self.reset()
                return "ok"
        return "unknown"


# --------------------------------------------------------------------------------------
PAGE = Path(__file__).with_name("page.html")


def make_handler(agent):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path.startswith("/video"):
                return self._mjpeg()
            if self.path.startswith("/state"):
                with agent.lock:
                    data = json.dumps(agent.state_json()).encode()
                self._send(200, data, "application/json")
            else:
                self._send(200, PAGE.read_bytes(), "text/html; charset=utf-8")

        def _mjpeg(self):
            if not hasattr(agent.ex, "jpeg"):
                return self._send(404, b"no camera on this agent", "text/plain")
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                while True:
                    jpg = agent.ex.jpeg
                    if jpg:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                    time.sleep(0.08)
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                body = {}
            res = agent.handle_post(self.path, body)
            self._send(200, json.dumps({"result": res}).encode(), "application/json")

    return H


def parse_addr(s):
    host, port = s.rsplit(":", 1)
    return (host, int(port))


def main():
    ap = argparse.ArgumentParser(description="Fulfilment cell agent")
    ap.add_argument("--id", required=True, help="agent id from config.json, e.g. robot or human")
    ap.add_argument("--config", default=str(Path(__file__).with_name("config.json")))
    ap.add_argument("--port", type=int, default=9100, help="UDP port this agent listens on")
    ap.add_argument("--peer", action="append", default=[], help="peer address host:port (repeatable)")
    ap.add_argument("--web", type=int, default=8000, help="web page port")
    ap.add_argument("--arm", default="fake", choices=["fake", "so101", "hand", "leader"],
                    help="robot: fake|so101   human: fake|hand (camera tracks a real hand)|leader (fallback)")
    ap.add_argument("--arm-port", help="serial port of the arm, e.g. /dev/ttyACM0 or COM5")
    ap.add_argument("--arm-id", help="unused for now (kept for a LeRobot backend)")
    ap.add_argument("--motions", default="motions", help="folder of taught robot motions")
    ap.add_argument("--zones", default="zones.json", help="taught leader zones file (leader fallback)")
    ap.add_argument("--regions", default="regions.json", help="camera zone boxes for --arm hand")
    ap.add_argument("--cam", type=int, help="camera index: hand tracking (--arm hand) or Tier 2 motion watcher")
    ap.add_argument("--roi", help="camera region x,y,w,h covering the human arm")
    ap.add_argument("--watch", default="human", help="which peer the camera watches")
    ap.add_argument("--view", help="camera stream to show on this page (display only), e.g. http://127.0.0.1:8001/video")
    args = ap.parse_args()

    cfg = json.loads(Path(args.config).read_text())
    kind = cfg["agents"][args.id]["kind"]
    if args.arm == "fake":
        ex = FakeRobot() if kind == "robot" else FakeHuman()
    elif args.arm == "so101":
        from robot_arm import FollowerReplay
        ex = FollowerReplay(args.arm_port, args.arm_id, args.motions)
    elif args.arm == "hand":
        from human_hand import HandTracker
        ex = HandTracker(args.cam if args.cam is not None else 0, args.regions)
        ex.start()
    else:
        from human_leader import LeaderTracker
        ex = LeaderTracker(args.arm_port, args.arm_id, args.zones)
        ex.start()

    watcher = None
    if args.cam is not None and args.arm != "hand":
        from vision import MotionWatcher
        roi = tuple(int(x) for x in args.roi.split(",")) if args.roi else None
        watcher = MotionWatcher(args.cam, roi)
        watcher.start()

    agent = Agent(cfg, args.id, args.port, [parse_addr(p) for p in args.peer], ex, watcher, args.watch)
    agent.view_url = args.view
    for f in (agent.send_loop, agent.recv_loop, agent.tick_loop):
        threading.Thread(target=f, daemon=True).start()
    agent.say(f"{kind} agent up. UDP :{args.port} peers {args.peer}  page http://localhost:{args.web}")
    ThreadingHTTPServer(("0.0.0.0", args.web), make_handler(agent)).serve_forever()


if __name__ == "__main__":
    main()
