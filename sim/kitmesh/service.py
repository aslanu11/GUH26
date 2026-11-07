"""Station process: peer link, local HTTP, and the robot executor.

The dashboard calls this HTTP API. It can submit an order, partition the link,
stop execution, or pass on an explicit handover. It does not choose owners.
"""

from __future__ import annotations

import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from kitmesh.layout import STATION_INFO
from kitmesh.station import Station

SIM_NAME_ISAAC = "isaac-sim"


class SimClient:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port

    def request(self, payload: dict, timeout: float) -> dict:
        try:
            with socket.create_connection((self.host, self.port), timeout=min(timeout, 5)) as sock:
                sock.settimeout(timeout)
                sock.sendall((json.dumps(payload) + "\n").encode())
                return _read_json_line(sock, timeout)
        except OSError as exc:
            raise ConnectionError(str(exc)) from exc


def _read_json_line(sock: socket.socket, timeout: float) -> dict:
    sock.settimeout(timeout)
    buffer = b""
    while b"\n" not in buffer:
        chunk = sock.recv(65536)
        if not chunk:
            raise ConnectionError("simulation connection closed")
        buffer += chunk
        if len(buffer) > 2_000_000:
            raise ConnectionError("simulation response is too large")
    line, _, _ = buffer.partition(b"\n")
    return json.loads(line.decode())


class StationService:
    def __init__(self, station: Station, *, listen_port: int, peer_host: str, peer_port: int, sim: SimClient | None):
        self.station = station
        self.listen_port = listen_port
        self.peer_host = peer_host
        self.peer_port = peer_port
        self.sim = sim
        self.lock = threading.RLock()
        self.stop_flag = threading.Event()
        self.peer_connected = False
        self._peer_sock: socket.socket | None = None
        self._sim_status = {"name": None, "connected": False, "ready": False, "detail": "no simulator configured"}
        self.peer_name = next(name for name in STATION_INFO if name != station.station_id)
        self._is_dialer = station.station_id < self.peer_name

    def status(self) -> dict:
        with self.lock:
            payload = self.station.status(simulator=dict(self._sim_status))
            payload["label"] = STATION_INFO[self.station.station_id]["label"]
            payload["peer_connected"] = self.peer_connected and not self.station.partitioned
            payload["peer"] = self.peer_name
            return payload

    def start(self) -> None:
        self._threads = [
            threading.Thread(target=self._listen_peers, name="peer-listen", daemon=True),
            threading.Thread(target=self._dial_peer, name="peer-dial", daemon=True),
            threading.Thread(target=self._heartbeat, name="heartbeat", daemon=True),
            threading.Thread(target=self._robot_loop, name="robot", daemon=True),
            threading.Thread(target=self._sim_poll, name="sim-poll", daemon=True),
        ]
        for thread in self._threads:
            thread.start()

    def close(self) -> None:
        self.stop_flag.set()
        self._drop_peer()
        self.station.close()

    def _listen_peers(self) -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("127.0.0.1", self.listen_port))
        server.listen(4)
        server.settimeout(0.5)
        while not self.stop_flag.is_set():
            try:
                conn, _ = server.accept()
            except socket.timeout:
                continue
            if self.station.partitioned or self.peer_connected:
                conn.close()
                continue
            threading.Thread(target=self._handle_peer, args=(conn,), daemon=True).start()
        server.close()

    def _dial_peer(self) -> None:
        if not self._is_dialer:
            return
        while not self.stop_flag.is_set():
            if self.station.partitioned or self.peer_connected:
                time.sleep(0.2)
                continue
            try:
                conn = socket.create_connection((self.peer_host, self.peer_port), timeout=1)
            except OSError:
                time.sleep(0.4)
                continue
            self._handle_peer(conn)

    def _handle_peer(self, conn: socket.socket) -> None:
        with self.lock:
            if self.station.partitioned or self.peer_connected:
                conn.close()
                return
            self._peer_sock = conn
            self.peer_connected = True
            try:
                _send_line(conn, {"type": "events", "events": self.station.events()})
            except OSError:
                self._drop_peer()
                return
        try:
            while not self.stop_flag.is_set() and not self.station.partitioned:
                message = _read_json_line(conn, 30)
                kind = message.get("type")
                if kind == "heartbeat":
                    with self.lock:
                        # Liveness only. Ownership stays on the event log.
                        self.station.apply_heartbeat(time.time())
                elif kind == "events":
                    with self.lock:
                        added = self.station.merge_events(message.get("events") or [])
                        if added:
                            _send_line(conn, {"type": "events", "events": self.station.events()})
                else:
                    continue
        except (OSError, json.JSONDecodeError, ConnectionError, ValueError):
            pass
        finally:
            with self.lock:
                if self._peer_sock is conn:
                    self._peer_sock = None
                    self.peer_connected = False
            try:
                conn.close()
            except OSError:
                pass

    def _heartbeat(self) -> None:
        while not self.stop_flag.is_set():
            time.sleep(0.4)
            with self.lock:
                conn = self._peer_sock
                if conn is None or self.station.partitioned:
                    continue
                try:
                    _send_line(conn, {"type": "heartbeat", "station": self.station.station_id, "ts": time.time()})
                except OSError:
                    self._drop_peer()

    def _drop_peer(self) -> None:
        with self.lock:
            conn = self._peer_sock
            self._peer_sock = None
            self.peer_connected = False
        if conn is not None:
            try:
                conn.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                conn.close()
            except OSError:
                pass

    def _robot_loop(self) -> None:
        while not self.stop_flag.is_set():
            if self.station.role != "robot" or self.sim is None:
                time.sleep(0.3)
                continue
            with self.lock:
                task = self.station.peek_robot_task()
            if task is None:
                time.sleep(0.2)
                continue
            try:
                sim_status = self.sim.request({"cmd": "status"}, timeout=2)
            except ConnectionError:
                time.sleep(0.5)
                continue
            # A stub or a simulator that has not finished loading must not consume the queue.
            if sim_status.get("name") != "isaac-sim" or not sim_status.get("ready"):
                time.sleep(0.5)
                continue
            with self.lock:
                claimed = self.station.claim_robot_task(task["task_id"])
                if claimed is not None:
                    self._push_events()
            if claimed is None:
                continue
            try:
                result = self.sim.request(
                    {
                        "cmd": "pick",
                        "station": self.station.station_id,
                        "task_id": claimed["task_id"],
                        "cube_id": claimed["cube_id"],
                        "stock_id": claimed["stock_id"],
                        "zone_id": claimed["zone_id"],
                    },
                    timeout=180,
                )
                result["started"] = True
            except (ConnectionError, json.JSONDecodeError, TimeoutError) as exc:
                result = {
                    "verified": False,
                    "reason": f"Simulator connection lost during the pick ({exc}). Not retried.",
                }
            with self.lock:
                self.station.finish_robot_task(claimed["task_id"], result)
                self._push_events()

    def _push_events(self) -> None:
        conn = self._peer_sock
        if conn is None or self.station.partitioned:
            return
        try:
            _send_line(conn, {"type": "events", "events": self.station.events()})
        except OSError:
            self._drop_peer()

    def _sim_poll(self) -> None:
        while not self.stop_flag.is_set():
            if self.sim is None or self.station.role != "robot":
                time.sleep(1)
                continue
            try:
                status = self.sim.request({"cmd": "status"}, timeout=2)
                status["connected"] = True
            except (ConnectionError, json.JSONDecodeError, TimeoutError) as exc:
                status = {"name": None, "connected": False, "ready": False, "detail": str(exc)}
            with self.lock:
                self._sim_status = status
            time.sleep(1)

    def abort_sim(self) -> None:
        if self.sim is None:
            return
        try:
            self.sim.request({"cmd": "abort"}, timeout=2)
        except (ConnectionError, json.JSONDecodeError, TimeoutError):
            return


def _send_line(conn: socket.socket, payload: dict) -> None:
    conn.sendall((json.dumps(payload) + "\n").encode())


def serve(station: Station, *, listen_port: int, peer_host: str, peer_port: int, http_port: int, sim_host: str | None, sim_port: int | None) -> None:
    sim = SimClient(sim_host, sim_port) if sim_host and sim_port and station.role == "robot" else None
    service = StationService(station, listen_port=listen_port, peer_host=peer_host, peer_port=peer_port, sim=sim)
    service.start()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args) -> None:
            print(f"[{station.station_id}] {self.address_string()} {fmt % args}", flush=True)

        def do_GET(self) -> None:  # noqa: N802
            if self.path.split("?", 1)[0] != "/status":
                self._send(404, {"ok": False, "error": "not found"})
                return
            self._send(200, service.status())

        def do_POST(self) -> None:  # noqa: N802
            path = self.path.split("?", 1)[0]
            try:
                length = int(self.headers.get("Content-Length", "0"))
                raw = self.rfile.read(length) if length else b"{}"
                body = json.loads(raw.decode() or "{}")
                result = self._dispatch(path, body)
            except ValueError as exc:
                self._send(400, {"ok": False, "error": str(exc)})
                return
            except json.JSONDecodeError:
                self._send(400, {"ok": False, "error": "expected JSON"})
                return
            self._send(200, result)

        def _dispatch(self, path: str, body: dict) -> dict:
            with service.lock:
                if path == "/order":
                    order_id = station.submit_order(body.get("lines") or [])
                    service._push_events()
                    return {"ok": True, "order_id": order_id}
                if path == "/partition":
                    station.set_partition(bool(body.get("enabled")))
                    if station.partitioned:
                        service._drop_peer()
                    return {"ok": True, "partitioned": station.partitioned}
                if path == "/hold":
                    station.set_hold_next(bool(body.get("enabled")))
                    return {"ok": True, "hold_next": station.hold_next}
                if path == "/execution":
                    stopping = not bool(body.get("enabled"))
                    if stopping:
                        station.stop_execution()
                    else:
                        station.resume_execution()
                    service._push_events()
                    result = {"ok": True, "execution_enabled": station.execution_enabled}
                else:
                    stopping = False
                    result = None
            if stopping:
                service.abort_sim()
            if result is not None:
                return result
            with service.lock:
                if path == "/handover/offer":
                    offer_id = station.offer_handover(body["task_id"], body["to"])
                    service._push_events()
                    return {"ok": True, "offer_id": offer_id}
                if path == "/handover/accept":
                    station.accept_handover(body["task_id"])
                    service._push_events()
                    return {"ok": True}
                if path == "/operator/begin":
                    station.begin_operator_task(body["task_id"])
                    service._push_events()
                    return {"ok": True}
                if path == "/operator/confirm":
                    station.confirm_operator_task(body["task_id"])
                    service._push_events()
                    return {"ok": True}
                if path == "/operator/uncertain":
                    station.uncertain_operator_task(body["task_id"])
                    service._push_events()
                    return {"ok": True}
            raise ValueError("unknown action")

        def _send(self, code: int, payload: dict) -> None:
            data = json.dumps(payload).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

    httpd = ThreadingHTTPServer(("127.0.0.1", http_port), Handler)
    print(
        f"KITMESH_STATION_READY {station.station_id} http=127.0.0.1:{http_port} peer={listen_port}",
        flush=True,
    )
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        service.close()


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="Run one KitMesh station agent")
    parser.add_argument("--station", required=True, choices=sorted(STATION_INFO))
    parser.add_argument("--role", required=True, choices=("robot", "operator"))
    parser.add_argument("--data", required=True)
    parser.add_argument("--listen-port", type=int, required=True)
    parser.add_argument("--peer-host", default="127.0.0.1")
    parser.add_argument("--peer-port", type=int, required=True)
    parser.add_argument("--http-port", type=int, required=True)
    parser.add_argument("--sim-host", default="")
    parser.add_argument("--sim-port", type=int, default=0)
    parser.add_argument("--isolated", action="store_true")
    args = parser.parse_args()
    station = Station(args.station, args.data, args.role)
    if args.isolated:
        station.set_isolated(True)
    serve(
        station,
        listen_port=args.listen_port,
        peer_host=args.peer_host,
        peer_port=args.peer_port,
        http_port=args.http_port,
        sim_host=args.sim_host or None,
        sim_port=args.sim_port or None,
    )


if __name__ == "__main__":
    main()
