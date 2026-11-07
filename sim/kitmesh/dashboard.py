"""Local dashboard. It displays station records and forwards operator actions.

Task owners are computed by the stations. Nothing in this process assigns work.
"""

from __future__ import annotations

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.error import URLError
from urllib.request import Request, urlopen

PAGE = (Path(__file__).with_name("dashboard.html")).read_text()


def fetch(url: str, payload: dict | None = None, timeout: float = 3) -> dict:
    data = None if payload is None else json.dumps(payload).encode()
    request = Request(url, data=data, headers={"Content-Type": "application/json"})
    with urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode())


def safe_fetch(url: str, payload: dict | None = None) -> dict:
    try:
        return fetch(url, payload)
    except (URLError, TimeoutError, json.JSONDecodeError, OSError) as exc:
        return {"ok": False, "error": str(exc), "offline": True}


def snapshot(stations: dict[str, str]) -> dict:
    records = {name: safe_fetch(url + "/status") for name, url in stations.items()}
    task_ids = set()
    for record in records.values():
        for task in record.get("tasks") or []:
            task_ids.add(task["task_id"])
    differ = []
    for task_id in sorted(task_ids):
        views = []
        for record in records.values():
            match = next((task for task in record.get("tasks") or [] if task["task_id"] == task_id), None)
            if match is None:
                views.append(None)
            else:
                views.append((match["state"], match["owner"], match["verification"], match["established"]))
        if views[0] != views[1]:
            differ.append(task_id)
    return {"stations": records, "differ": differ}


def serve(port: int, stations: dict[str, str]) -> None:
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args) -> None:
            print(f"[dashboard] {fmt % args}", flush=True)

        def do_GET(self) -> None:  # noqa: N802
            path = self.path.split("?", 1)[0]
            if path == "/":
                data = PAGE.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return
            if path == "/api/snapshot":
                self._send(200, snapshot(stations))
                return
            self._send(404, {"ok": False, "error": "not found"})

        def do_POST(self) -> None:  # noqa: N802
            path = self.path.split("?", 1)[0]
            length = int(self.headers.get("Content-Length", "0"))
            try:
                body = json.loads(self.rfile.read(length).decode() or "{}")
            except json.JSONDecodeError:
                self._send(400, {"ok": False, "error": "expected JSON"})
                return
            station = body.get("station")
            if path == "/api/partition":
                enabled = bool(body.get("enabled"))
                results = {
                    name: safe_fetch(url + "/partition", {"enabled": enabled})
                    for name, url in stations.items()
                }
                self._send(200, {"ok": True, "results": results})
                return
            if station not in stations:
                self._send(400, {"ok": False, "error": "unknown station"})
                return
            base = stations[station]
            routes = {
                "/api/order": "/order",
                "/api/hold": "/hold",
                "/api/execution": "/execution",
                "/api/handover/offer": "/handover/offer",
                "/api/handover/accept": "/handover/accept",
                "/api/operator/begin": "/operator/begin",
                "/api/operator/confirm": "/operator/confirm",
                "/api/operator/uncertain": "/operator/uncertain",
            }
            if path not in routes:
                self._send(404, {"ok": False, "error": "not found"})
                return
            forwarded = {key: value for key, value in body.items() if key != "station"}
            self._send(200, safe_fetch(base + routes[path], forwarded))

        def _send(self, code: int, payload: dict) -> None:
            data = json.dumps(payload).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

    httpd = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    print(f"KITMESH_DASHBOARD_READY http://127.0.0.1:{port}", flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        httpd.shutdown()


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="KitMesh station dashboard")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--station-a", default="http://127.0.0.1:9201")
    parser.add_argument("--station-b", default="http://127.0.0.1:9202")
    args = parser.parse_args()
    serve(args.port, {"station-a": args.station_a.rstrip("/"), "station-b": args.station_b.rstrip("/")})


if __name__ == "__main__":
    main()
