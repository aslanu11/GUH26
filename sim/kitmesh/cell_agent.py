"""One robot's agent in the two-robot packing cell.

Each robot runs its own copy of this process. The two never exchange
anything: no sockets between them, no shared files, no heartbeats. An agent
knows only:

* the fixed shift plan in cell_layout (its lane, the other lane, its carton);
* what it can observe on the table through the simulator's ``observe``
  command, which returns parcel poses and the other arm's link positions,
  the same things a camera over the cell would see;
* the replies to its own pick requests.

It infers that the other robot has stopped when the other lane still has
parcels on their marks and that count has not dropped for STALL_SECONDS.
Then, after finishing its own lane, it takes over the other lane, working
from the end the other robot would have reached last. It never claims the
other robot is broken, only that it appears stopped.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from kitmesh.cell_layout import (
    ARM_CLEARANCE_M,
    PARCELS,
    ROBOT_INFO,
    STALL_SECONDS,
    other,
    where,
)


class Brain:
    """Decision logic, fed only with observations and this robot's own pick results."""

    def __init__(self, me: str, now: float, stall_seconds: float = STALL_SECONDS):
        self.me = me
        self.peer = other(me)
        self.own_lane = list(ROBOT_INFO[me]["lane"])
        self.other_lane = list(ROBOT_INFO[self.peer]["lane"])
        self.stall_seconds = stall_seconds
        self.started = now
        self.other_count: int | None = None
        self.last_change = now
        self.other_stopped = False
        self.outcome: dict[str, str] = {}  # parcel -> packed | gone | review | blocked
        self.detail: dict[str, str] = {}
        self.events: list[dict] = []
        self.logged = 0
        self.active: str | None = None  # the parcel this robot is picking right now
        self.blocked: dict[str, str] = {}
        self.now = now
        self.seen: dict[str, str] = {}

    def _event(self, now: float, kind: str, note: str) -> None:
        self.events.append({"t": round(now - self.started, 1), "kind": kind, "note": note})

    def observe(self, parcels: dict, now: float) -> None:
        """parcels: {id: [x, y, z]} as observed on the table."""
        self.now = now
        self.seen = {pid: where(pid, pos) for pid, pos in parcels.items() if pid in PARCELS}
        # Parcels I took from the other lane myself are not a sign of life from
        # the other robot, so they still count as "left on its lane".
        count = sum(
            1
            for pid in self.other_lane
            if self.seen.get(pid) == "mark" or pid == self.active or self.outcome.get(pid) == "packed"
        )
        peer = ROBOT_INFO[self.peer]["label"]
        if self.other_count is None:
            self.other_count = count
            self.last_change = now
        elif count < self.other_count:
            self.other_count = count
            self.last_change = now
            if self.other_stopped:
                self.other_stopped = False
                self._event(now, "peer_moving", f"{peer}'s lane changed again ({count} left). Takeover paused.")
        elif count > self.other_count:
            # Parcels came back (scene reset). Start watching afresh.
            self.other_count = count
            self.last_change = now
        if (
            not self.other_stopped
            and count > 0
            and now - self.last_change >= self.stall_seconds
        ):
            self.other_stopped = True
            self._event(
                now,
                "peer_stopped",
                f"{peer} appears stopped: {count} parcel(s) still on its lane, none moved for "
                f"{now - self.last_change:.0f} s. Taking over after my own lane.",
            )

    def watch(self, now: float) -> dict:
        return {
            "robot": self.peer,
            "on_marks": self.other_count,
            "since_change_s": round(now - self.last_change, 1),
            "stall_after_s": self.stall_seconds,
            "appears_stopped": self.other_stopped,
        }

    def next_pick(self, other_arm_links=()) -> str | None:
        for pid in self.own_lane:
            if self.seen.get(pid) == "mark" and pid not in self.outcome:
                return pid
        if not self.other_stopped:
            return None
        for pid in reversed(self.other_lane):
            if self.seen.get(pid) != "mark" or self.outcome.get(pid) in ("packed", "gone", "review"):
                continue
            x, y = PARCELS[pid]["xy"]
            near = min((math.hypot(lx - x, ly - y) for lx, ly, _ in other_arm_links), default=9.0)
            if near < ARM_CLEARANCE_M:
                # Not final: checked again every cycle, taken as soon as the arm is clear.
                if pid not in self.blocked:
                    self._event(
                        self.now,
                        "blocked",
                        f"{pid} is {near:.2f} m from {ROBOT_INFO[self.peer]['label']}'s stopped arm. "
                        "Not reaching in; waiting until it is clear or a person moves it.",
                    )
                self.blocked[pid] = f"{ROBOT_INFO[self.peer]['label']}'s stopped arm is {near:.2f} m away"
                continue
            self.blocked.pop(pid, None)
            return pid
        return None

    def record(self, pid: str, result: dict, now: float) -> str:
        reason = str(result.get("reason", ""))
        mine = pid in self.own_lane
        if result.get("verified"):
            outcome = "packed"
            note = f"Packed {pid} into my carton" + ("" if mine else " (taken over)")
        elif "is not at its mark" in reason:
            outcome = "gone"
            note = f"{pid} was no longer on its mark when I reached for it"
        elif "faulted" in reason:
            outcome = "fault"
            note = f"My arm reported a fault during {pid}: {reason}"
        else:
            outcome = "review"
            note = f"{pid} needs review: {reason}"
        if outcome != "fault":
            self.outcome[pid] = outcome
            self.detail[pid] = reason
        self._event(now, outcome, note)
        return outcome

    def done(self) -> bool:
        own_left = any(self.seen.get(p) == "mark" and p not in self.outcome for p in self.own_lane)
        # A parcel blocked by the stopped arm keeps the job open: it may clear.
        other_left = any(self.seen.get(p) == "mark" and self.outcome.get(p) != "review" for p in self.other_lane)
        return not own_left and not other_left

    def parcel_view(self) -> list[dict]:
        rows = []
        for pid in self.own_lane + self.other_lane:
            rows.append(
                {
                    "parcel": pid,
                    "lane": "mine" if pid in self.own_lane else "other",
                    "seen": self.seen.get(pid, "unknown"),
                    "outcome": self.outcome.get(pid) or ("blocked" if pid in self.blocked else None),
                    "detail": self.detail.get(pid) or self.blocked.get(pid, ""),
                }
            )
        return rows


def sim_request(port: int, payload: dict, timeout: float) -> dict:
    with socket.create_connection(("127.0.0.1", port), timeout=timeout) as conn:
        conn.settimeout(timeout)
        conn.sendall((json.dumps(payload) + "\n").encode())
        buffer = b""
        while b"\n" not in buffer:
            chunk = conn.recv(65536)
            if not chunk:
                raise ConnectionError("simulator closed the connection")
            buffer += chunk
    return json.loads(buffer.split(b"\n", 1)[0].decode())


class Agent:
    """Two threads: one keeps watching the table, one does this robot's picks.

    Time is the simulator's own clock (``sim_time`` in every observation), so
    a slow-running simulation cannot make a healthy robot look stopped.
    """

    def __init__(self, me: str, sim_port: int, data: Path):
        self.me = me
        self.sim_port = sim_port
        self.data = data
        data.mkdir(parents=True, exist_ok=True)
        self.log_path = data / "events.jsonl"
        self.lock = threading.Lock()
        self.state = "idle"
        self.current: str | None = None
        self.sim_now = 0.0
        self.other_links: list = []
        self.brain = Brain(me, 0.0)
        self.run_id = 0
        self.picks = 0
        threading.Thread(target=self._watch, daemon=True).start()
        threading.Thread(target=self._work, daemon=True).start()

    def _log(self) -> None:
        with self.lock:
            brain = self.brain
            fresh = brain.events[brain.logged:]
            brain.logged = len(brain.events)
            run = self.run_id
        if fresh:
            with self.log_path.open("a") as handle:
                for event in fresh:
                    handle.write(json.dumps({"robot": self.me, "run": run, **event}) + "\n")

    def start(self) -> dict:
        with self.lock:
            if self.state == "running":
                return {"ok": False, "error": "already running"}
            self.run_id += 1
            self.brain = Brain(self.me, self.sim_now)
            self.brain._event(self.sim_now, "start", "Job started. Watching the other lane by observation only.")
            self.state = "running"
        return {"ok": True}

    def resume(self) -> dict:
        with self.lock:
            if self.state not in ("fault", "paused", "done"):
                return {"ok": False, "error": f"cannot resume from {self.state}"}
            self.state = "running"
            self.brain._event(self.sim_now, "resume", "Resumed by the operator.")
        return {"ok": True}

    def pause(self) -> dict:
        with self.lock:
            if self.state == "running":
                self.state = "paused"
                self.brain._event(self.sim_now, "pause", "Paused by the operator.")
        return {"ok": True}

    def status(self) -> dict:
        with self.lock:
            brain = self.brain
            return {
                "robot": self.me,
                "label": ROBOT_INFO[self.me]["label"],
                "carton": ROBOT_INFO[self.me]["carton"],
                "state": self.state,
                "current": self.current,
                "picks": self.picks,
                "sim_time": self.sim_now,
                "watch": brain.watch(self.sim_now),
                "parcels": brain.parcel_view(),
                "events": brain.events[-40:],
                "communication": "none: observes the table only",
            }

    def _watch(self) -> None:
        while True:
            time.sleep(0.4)
            try:
                seen = sim_request(self.sim_port, {"cmd": "observe", "robot": self.me}, timeout=2)
            except (OSError, ConnectionError, json.JSONDecodeError):
                continue
            with self.lock:
                self.sim_now = float(seen.get("sim_time", self.sim_now))
                self.other_links = seen.get("other_arm_links", [])
                self.brain.observe(seen.get("parcels", {}), self.sim_now)
            self._log()

    def _work(self) -> None:
        while True:
            time.sleep(0.3)
            with self.lock:
                if self.state != "running" or not self.brain.seen:
                    continue
                brain = self.brain
                pid = brain.next_pick(self.other_links)
                if pid is None:
                    if brain.done():
                        self.state = "done"
                        brain._event(self.sim_now, "done", "No parcels left on any mark that I can take. Job finished.")
                    continue
                self.current = pid
                brain.active = pid
                self.picks += 1
                run = self.run_id
            try:
                result = sim_request(
                    self.sim_port,
                    {
                        "cmd": "pick",
                        "arm": self.me,
                        "stock": pid,
                        "carton": ROBOT_INFO[self.me]["carton"],
                        "task_id": f"{self.me}-run{run}-{pid}",
                        "cube_id": f"parcel-{pid}",
                    },
                    timeout=600,
                )
            except (OSError, ConnectionError, json.JSONDecodeError) as exc:
                result = {"verified": False, "reason": f"simulator connection lost ({exc}); not retried"}
            try:
                # Re-read the clock: after a long pause (or a frozen process) sim_now is stale.
                clock = float(sim_request(self.sim_port, {"cmd": "observe", "robot": self.me}, timeout=2)["sim_time"])
            except (OSError, ConnectionError, json.JSONDecodeError, KeyError, ValueError):
                clock = None
            with self.lock:
                if clock is not None:
                    self.sim_now = max(self.sim_now, clock)
                self.current = None
                brain.active = None
                if brain is self.brain and brain.record(pid, result, self.sim_now) == "fault":
                    self.state = "fault"
            self._log()


def serve(agent: Agent, port: int) -> None:
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format: str, *args) -> None:  # noqa: A002
            return

        def _send(self, payload: dict, code: int = 200) -> None:
            data = json.dumps(payload).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self) -> None:  # noqa: N802
            if self.path.split("?", 1)[0] == "/status":
                self._send(agent.status())
            else:
                self._send({"ok": False, "error": "not found"}, 404)

        def do_POST(self) -> None:  # noqa: N802
            routes = {"/start": agent.start, "/resume": agent.resume, "/pause": agent.pause}
            handler = routes.get(self.path.split("?", 1)[0])
            if handler is None:
                self._send({"ok": False, "error": "not found"}, 404)
            else:
                self._send(handler())

    httpd = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    print(f"KITMESH_AGENT_READY {agent.me} http=127.0.0.1:{port} (observation only, no peer link)", flush=True)
    httpd.serve_forever()


def main() -> None:
    parser = argparse.ArgumentParser(description="KitMesh robot agent (observation only)")
    parser.add_argument("--robot", choices=sorted(ROBOT_INFO), required=True)
    parser.add_argument("--http-port", type=int, required=True)
    parser.add_argument("--sim-port", type=int, default=9401)
    parser.add_argument("--data", required=True)
    args = parser.parse_args()
    serve(Agent(args.robot, args.sim_port, Path(args.data)), args.http_port)


if __name__ == "__main__":
    main()
