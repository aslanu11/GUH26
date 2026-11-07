"""A stand-in socket used only to rehearse station communication.

It never reports an Isaac Sim verification. Robot tasks that reach it are
rejected so they cannot show up as simulation-verified grasps.
"""

from __future__ import annotations

import json
import socket
import threading

from kitmesh.layout import STOCKS, ZONES


def serve(port: int) -> socket.socket:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", port))
    server.listen(8)

    def client(conn: socket.socket) -> None:
        try:
            buffer = b""
            while b"\n" not in buffer:
                chunk = conn.recv(65536)
                if not chunk:
                    return
                buffer += chunk
            payload = json.loads(buffer.split(b"\n", 1)[0].decode())
            command = payload.get("cmd")
            if command == "status":
                reply = {
                    "name": "stub-sim",
                    "ready": False,
                    "connected": True,
                    "arm": None,
                    "detail": "Coordination stub only. No physics and no grasp.",
                    "cubes": {
                        stock_id: {"position": list(spec["xyz"])}
                        for stock_id, spec in STOCKS.items()
                    },
                    "zones": {zone_id: {"xy": list(spec["xy"])} for zone_id, spec in ZONES.items()},
                }
            elif command == "abort":
                reply = {"ok": True}
            elif command == "pick":
                reply = {
                    "ok": False,
                    "verified": False,
                    "reason": "Stub simulator cannot verify a grasp",
                    "evidence": {"simulator": "stub-sim"},
                }
            else:
                reply = {"ok": False, "verified": False, "reason": "unknown command"}
            conn.sendall((json.dumps(reply) + "\n").encode())
        except (OSError, json.JSONDecodeError):
            return
        finally:
            conn.close()

    def loop() -> None:
        while True:
            try:
                conn, _ = server.accept()
            except OSError:
                return
            threading.Thread(target=client, args=(conn,), daemon=True).start()

    threading.Thread(target=loop, daemon=True).start()
    print(f"KITMESH_STUB_READY port={port}", flush=True)
    return server


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="Run the KitMesh coordination stub")
    parser.add_argument("--port", type=int, default=9401)
    args = parser.parse_args()
    serve(args.port)
    threading.Event().wait()


if __name__ == "__main__":
    main()
