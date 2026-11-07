"""KitMesh demo page for the two-robot packing cell.

This process is the operator's console. It never relays anything between
the two robot agents: "Start job" sends a start to each agent separately,
and "Fault robot" goes only to the simulator, so the other robot is not
told. It finds out, if at all, by watching the table.

It also streams the Isaac Sim window as MJPEG (ffmpeg x11grab) and forwards
camera moves to the simulator.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
import socket
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.error import URLError
from urllib.request import Request, urlopen

from kitmesh.cell_layout import (
    ARM_CLEARANCE_M,
    CARTONS,
    PARCEL_SIZE,
    PARCELS,
    ROBOT_INFO,
    SLOT_OFFSETS,
    STALL_SECONDS,
)

PAGE = Path(__file__).with_name("demo.html")


def agent_pid(robot: str) -> int | None:
    found = subprocess.run(
        ["pgrep", "-f", f"^/usr/bin/python3 -m kitmesh.cell_agent --robot {robot} "], capture_output=True, text=True
    ).stdout.split()
    return int(found[0]) if found else None


def http_json(url: str, payload: dict | None = None, timeout: float = 3.0) -> dict:
    data = None if payload is None else json.dumps(payload).encode()
    request = Request(url, data=data, headers={"Content-Type": "application/json"})
    try:
        with urlopen(request, timeout=timeout) as response:
            return json.loads(response.read().decode())
    except (URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
        return {"ok": False, "offline": True, "error": str(exc)}


def sim_request(port: int, payload: dict, timeout: float = 3.0) -> dict:
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=timeout) as conn:
            conn.settimeout(timeout)
            conn.sendall((json.dumps(payload) + "\n").encode())
            buffer = b""
            while b"\n" not in buffer:
                chunk = conn.recv(65536)
                if not chunk:
                    break
                buffer += chunk
        return json.loads(buffer.split(b"\n", 1)[0].decode())
    except (OSError, json.JSONDecodeError) as exc:
        return {"ok": False, "offline": True, "error": str(exc)}


def sim_log_lines(log_path: Path, limit: int = 14) -> list[dict]:
    try:
        lines = log_path.read_text(errors="replace").splitlines()
    except OSError:
        return []
    out = []
    for line in lines:
        if line.startswith("KITMESH_PICK_RESULT "):
            try:
                r = json.loads(line.split(" ", 1)[1])
            except json.JSONDecodeError:
                continue
            out.append({"kind": "result", "verified": r.get("verified"), "reason": r.get("reason"),
                        "arm": r.get("evidence", {}).get("arm")})
        elif line.startswith(("KITMESH_FAULT", "KITMESH_RESTORE", "KITMESH_SCENE_RESET")):
            out.append({"kind": line.split(" ", 1)[0].removeprefix("KITMESH_").lower(), "reason": line.split(" ", 1)[-1]})
    return out[-limit:]


class WindowStream:
    """Latest JPEG of the Isaac Sim window, captured with ffmpeg x11grab."""

    def __init__(self, display: str, xauthority: str, title: str, fps: int = 8):
        self.env = dict(os.environ, DISPLAY=display, XAUTHORITY=xauthority)
        self.display = display
        self.title = title
        self.fps = fps
        self.frame: bytes | None = None
        self.frame_id = 0
        self.status = "starting"
        self.cond = threading.Condition()
        threading.Thread(target=self._run, name="window-stream", daemon=True).start()

    def _geometry(self):
        try:
            tree = subprocess.run(["xwininfo", "-root", "-tree"], env=self.env, capture_output=True, text=True, timeout=5).stdout
        except (OSError, subprocess.TimeoutExpired):
            return None
        pattern = re.compile(r'^\s*(0x[0-9a-f]+) "' + re.escape(self.title) + r'": \("' + re.escape(self.title))
        for line in tree.splitlines():
            match = pattern.match(line)
            if not match:
                continue
            info = subprocess.run(["xwininfo", "-id", match.group(1)], env=self.env, capture_output=True, text=True, timeout=5).stdout
            values = {}
            for key in ("Absolute upper-left X", "Absolute upper-left Y", "Width", "Height", "Map State"):
                found = re.search(re.escape(key) + r":\s+(\S+)", info)
                values[key] = found.group(1) if found else None
            if values["Map State"] != "IsViewable":
                return None
            x, y = max(0, int(values["Absolute upper-left X"])), max(0, int(values["Absolute upper-left Y"]))
            return x, y, int(values["Width"]) // 2 * 2, int(values["Height"]) // 2 * 2
        return None

    def _run(self) -> None:
        while True:
            geometry = self._geometry()
            if geometry is None:
                self.status = f'No visible "{self.title}" window on {self.display}'
                time.sleep(2)
                continue
            x, y, w, h = geometry
            command = [
                "ffmpeg", "-loglevel", "error", "-f", "x11grab", "-draw_mouse", "0",
                "-framerate", str(self.fps), "-video_size", f"{w}x{h}", "-i", f"{self.display}+{x},{y}",
                "-vf", "scale='min(1280,iw)':-2", "-q:v", "6", "-f", "image2pipe", "-vcodec", "mjpeg", "-",
            ]
            try:
                proc = subprocess.Popen(command, env=self.env, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
            except OSError as exc:
                self.status = f"ffmpeg failed: {exc}"
                time.sleep(5)
                continue
            self.status = f"Streaming {w}x{h} from {self.display}"
            buffer = b""
            checked = time.monotonic()
            stdout = proc.stdout
            while stdout is not None:
                chunk = stdout.read(65536)
                if not chunk:
                    break
                buffer += chunk
                while True:
                    start = buffer.find(b"\xff\xd8")
                    end = buffer.find(b"\xff\xd9", start + 2)
                    if start < 0 or end < 0:
                        break
                    with self.cond:
                        self.frame = buffer[start : end + 2]
                        self.frame_id += 1
                        self.cond.notify_all()
                    buffer = buffer[end + 2 :]
                if time.monotonic() - checked > 3:
                    checked = time.monotonic()
                    if self._geometry() != geometry:
                        break
            proc.kill()
            proc.wait()
            time.sleep(0.5)

    def wait_frame(self, last_id: int, timeout: float = 2.0):
        with self.cond:
            self.cond.wait_for(lambda: self.frame_id != last_id, timeout=timeout)
            return self.frame_id, self.frame


def serve(args) -> None:
    agents = {"station-a": args.agent_a.rstrip("/"), "station-b": args.agent_b.rstrip("/")}
    stream = WindowStream(args.display, args.xauthority, args.window_title)
    sim_log = Path(args.sim_log)
    frozen: set[str] = set()
    layout = {
        "robots": {name: {k: info[k] for k in ("label", "base", "yaw_deg", "carton", "colour", "lane")} for name, info in ROBOT_INFO.items()},
        "parcels": PARCELS,
        "cartons": CARTONS,
        "slots": SLOT_OFFSETS,
        "parcel_size": PARCEL_SIZE,
        "stall_seconds": STALL_SECONDS,
        "clearance": ARM_CLEARANCE_M,
    }

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, format: str, *args) -> None:  # noqa: A002
            return

        def do_GET(self) -> None:  # noqa: N802
            path = self.path.split("?", 1)[0]
            if path == "/":
                self._send_bytes(200, PAGE.read_bytes(), "text/html; charset=utf-8")
            elif path == "/api/layout":
                self._send(200, layout)
            elif path == "/api/state":
                self._send(200, {
                    "agents": {
                        name: ({"offline": True, "frozen": True} if name in frozen else http_json(url + "/status", timeout=1.5))
                        for name, url in agents.items()
                    },
                    "frozen": sorted(frozen),
                    "sim": sim_request(args.sim_port, {"cmd": "status"}),
                    "log": sim_log_lines(sim_log),
                    "stream": stream.status,
                })
            elif path == "/stream.mjpg":
                self._mjpeg()
            elif path == "/frame.jpg":
                _, frame = stream.wait_frame(-1, timeout=0.1)
                if frame is None:
                    self._send(503, {"ok": False, "error": stream.status})
                else:
                    self._send_bytes(200, frame, "image/jpeg")
            else:
                self._send(404, {"ok": False, "error": "not found"})

        def do_POST(self) -> None:  # noqa: N802
            path = self.path.split("?", 1)[0]
            length = int(self.headers.get("Content-Length", "0") or 0)
            try:
                body = json.loads(self.rfile.read(length).decode() or "{}")
            except json.JSONDecodeError:
                self._send(400, {"ok": False, "error": "expected JSON"})
                return
            robot = body.get("robot")
            if path == "/api/job/start":
                self._send(200, {name: http_json(url + "/start", {}) for name, url in agents.items()})
            elif path in ("/api/fault", "/api/crash", "/api/recover"):
                if robot not in agents:
                    self._send(400, {"ok": False, "error": "unknown robot"})
                    return
                if path == "/api/fault":
                    # Arm fault: the arm stops where it is and suction turns off.
                    self._send(200, sim_request(args.sim_port, {"cmd": "fault", "arm": robot}, timeout=10))
                elif path == "/api/crash":
                    # Software crash: freeze the robot's agent process. The arm finishes the
                    # motion it was already given, then nothing more is asked of it.
                    pid = agent_pid(robot)
                    if pid is None:
                        self._send(200, {"ok": False, "error": f"{robot} agent process not found"})
                        return
                    os.kill(pid, signal.SIGSTOP)
                    frozen.add(robot)
                    print(f"[demo] froze {robot} agent pid {pid} (operator-injected software crash)", flush=True)
                    self._send(200, {"ok": True, "frozen_pid": pid})
                else:
                    reply = {"ok": True}
                    pid = agent_pid(robot)
                    if robot in frozen and pid is not None:
                        os.kill(pid, signal.SIGCONT)
                        frozen.discard(robot)
                        time.sleep(0.5)
                    reply["arm"] = sim_request(args.sim_port, {"cmd": "restore", "arm": robot}, timeout=10)
                    reply["agent"] = http_json(agents[robot] + "/resume", {})
                    self._send(200, reply)
            elif path == "/api/reset":
                paused = {name: http_json(url + "/pause", {}) for name, url in agents.items()}
                reply = sim_request(args.sim_port, {"cmd": "reset"}, timeout=70)
                reply["paused"] = paused
                self._send(200, reply)
            elif path == "/api/camera":
                eye, target = body.get("eye"), body.get("target")
                if not (isinstance(eye, list) and isinstance(target, list) and len(eye) == 3 and len(target) == 3):
                    self._send(400, {"ok": False, "error": "eye and target must be [x, y, z]"})
                    return
                self._send(200, sim_request(args.sim_port, {"cmd": "camera", "eye": eye, "target": target}, timeout=1.0))
            else:
                self._send(404, {"ok": False, "error": "not found"})

        def _mjpeg(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Connection", "close")
            self.end_headers()
            last = -1
            try:
                while True:
                    last, frame = stream.wait_frame(last)
                    if frame is None:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n")
            except (BrokenPipeError, ConnectionResetError, OSError):
                return

        def _send(self, code: int, payload: dict) -> None:
            self._send_bytes(code, json.dumps(payload).encode(), "application/json")

        def _send_bytes(self, code: int, data: bytes, kind: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

    for host in args.bind:
        httpd = ThreadingHTTPServer((host, args.port), Handler)
        httpd.daemon_threads = True
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        print(f"KITMESH_DEMO_READY http://{host}:{args.port}", flush=True)
    while True:
        time.sleep(3600)


def main() -> None:
    parser = argparse.ArgumentParser(description="KitMesh two-robot demo page")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--bind", action="append", help="address to listen on (repeatable); default 127.0.0.1")
    parser.add_argument("--agent-a", default="http://127.0.0.1:9201")
    parser.add_argument("--agent-b", default="http://127.0.0.1:9202")
    parser.add_argument("--sim-port", type=int, default=9401)
    parser.add_argument("--sim-log", default="/home/cal/kitmesh/logs/sim.log")
    parser.add_argument("--display", default=":0")
    parser.add_argument("--xauthority", default="/run/user/1000/gdm/Xauthority")
    parser.add_argument("--window-title", default="Isaac Sim 5.1.0")
    args = parser.parse_args()
    args.bind = args.bind or ["127.0.0.1"]
    serve(args)


if __name__ == "__main__":
    main()
