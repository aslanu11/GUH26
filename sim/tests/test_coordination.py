"""Ownership, partition, handover, dedup, and restart checks. No simulator required."""

from __future__ import annotations

import json
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from kitmesh.layout import DEMO_ORDER, STATION_A, STATION_B, WAITING_ORDER, default_owner
from kitmesh.station import Station


def post(url: str, payload: dict) -> dict:
    request = Request(url, data=json.dumps(payload).encode(), headers={"Content-Type": "application/json"})
    with urlopen(request, timeout=3) as response:
        return json.loads(response.read().decode())


def get(url: str) -> dict:
    with urlopen(url, timeout=3) as response:
        return json.loads(response.read().decode())


def wait_until(predicate, timeout: float = 5) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False


class CoordinationTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        root = Path(self.tmp.name)
        self.alpha = Station(STATION_A, root / "a", "robot")
        self.beta = Station(STATION_B, root / "b", "operator")
        self.addCleanup(self.alpha.close)
        self.addCleanup(self.beta.close)

    def share(self, lines=DEMO_ORDER) -> str:
        order_id = self.alpha.submit_order(list(lines))
        self.beta.merge_events(self.alpha.events())
        self.alpha.merge_events(self.beta.events())
        self.assertTrue(self.alpha.order_established(order_id))
        self.assertTrue(self.beta.order_established(order_id))
        return order_id

    def task(self, station: Station, stock_id: str) -> dict:
        match = [item for item in station.folded()["tasks"].values() if item["stock_id"] == stock_id]
        self.assertEqual(len(match), 1)
        return match[0]

    def test_same_owner_and_reach_limit(self) -> None:
        self.share()
        for stock_id, zone_id in (("S1", "ZA"), ("S2", "ZA"), ("S4", "ZB")):
            self.assertEqual(self.task(self.alpha, stock_id)["owner"], default_owner(stock_id, zone_id))
            self.assertEqual(self.task(self.beta, stock_id)["owner"], default_owner(stock_id, zone_id))
        self.assertEqual(self.task(self.alpha, "S1")["owner"], STATION_A)
        self.assertEqual(self.task(self.alpha, "S4")["owner"], STATION_B)
        with self.assertRaises(ValueError):
            self.alpha.offer_handover(self.task(self.alpha, "S1")["task_id"], STATION_B)

    def test_partition_keeps_established_work_and_holds_new_orders(self) -> None:
        self.share()
        self.alpha.set_partition(True)
        self.beta.set_partition(True)
        waiting = self.alpha.submit_order(list(WAITING_ORDER))
        self.assertFalse(self.alpha.order_established(waiting))
        self.assertEqual(self.alpha.peek_robot_task()["stock_id"], "S1")
        self.assertNotEqual(self.alpha.peek_robot_task()["order_id"], waiting)
        self.assertNotIn(waiting, self.beta.folded()["orders"])
        before = [task["owner"] for task in self.alpha.folded()["tasks"].values()]
        events_before = len(self.alpha.events())
        self.alpha.apply_heartbeat(time.time() - 100)
        self.beta.apply_heartbeat(None)
        self.assertEqual(before, [task["owner"] for task in self.alpha.folded()["tasks"].values()])
        self.assertEqual(events_before, len(self.alpha.events()))
        with self.assertRaises(ValueError):
            self.alpha.offer_handover(self.task(self.alpha, "S2")["task_id"], STATION_B)

    def test_reconnect_deduplicates_and_converges(self) -> None:
        self.share()
        self.alpha.set_partition(True)
        self.beta.set_partition(True)
        self.beta.begin_operator_task(self.task(self.beta, "S4")["task_id"])
        self.beta.confirm_operator_task(self.task(self.beta, "S4")["task_id"])
        self.alpha.submit_order(list(WAITING_ORDER))
        self.assertNotEqual(
            self.task(self.alpha, "S4")["verification"],
            self.task(self.beta, "S4")["verification"],
        )
        self.alpha.set_partition(False)
        self.beta.set_partition(False)
        self.alpha.merge_events(self.beta.events())
        self.beta.merge_events(self.alpha.events())
        self.alpha.merge_events(self.beta.events())
        self.assertEqual(
            {event["event_id"] for event in self.alpha.events()},
            {event["event_id"] for event in self.beta.events()},
        )
        self.assertEqual(self.task(self.alpha, "S4")["verification"], "operator")
        self.assertEqual(self.task(self.beta, "S4")["state"], "Verified complete")
        self.assertTrue(self.beta.order_established(self.task(self.beta, "S3")["order_id"]))
        again = self.alpha.merge_events(self.beta.events())
        self.assertEqual(again, 0)

    def test_handover_has_no_overlap_and_stop_transfers_only_eligible_work(self) -> None:
        self.share(WAITING_ORDER)
        s3 = self.task(self.alpha, "S3")["task_id"]
        self.alpha.offer_handover(s3, STATION_B)
        self.beta.merge_events(self.alpha.events())
        self.assertIsNone(self.alpha.peek_robot_task())
        with self.assertRaises(ValueError):
            self.beta.begin_operator_task(s3)
        self.beta.accept_handover(s3)
        self.alpha.merge_events(self.beta.events())
        self.assertEqual(self.task(self.alpha, "S3")["owner"], STATION_B)
        self.assertIsNone(self.alpha.peek_robot_task())
        self.beta.begin_operator_task(s3)
        self.assertEqual(self.task(self.beta, "S3")["state"], "In progress")
        self.alpha.merge_events(self.beta.events())
        self.assertIsNone(self.alpha.claim_robot_task(s3))

    def test_stop_and_review_do_not_retry(self) -> None:
        self.share()
        s1 = self.task(self.alpha, "S1")["task_id"]
        s2 = self.task(self.alpha, "S2")["task_id"]
        self.alpha.claim_robot_task(s1)
        self.alpha.stop_execution()
        self.assertEqual(self.task(self.alpha, "S1")["state"], "In progress")
        self.assertEqual(self.task(self.alpha, "S2")["state"], "Blocked")
        self.alpha.finish_robot_task(s1, {"verified": False, "reason": "cube slipped"})
        self.assertEqual(self.task(self.alpha, "S1")["state"], "Needs review")
        self.alpha.resume_execution()
        self.assertEqual(self.task(self.alpha, "S1")["state"], "Needs review")
        self.assertEqual(self.alpha.peek_robot_task()["task_id"], s2)
        self.alpha.offer_handover(s2, STATION_B)
        self.beta.merge_events(self.alpha.events())
        self.beta.accept_handover(s2)
        self.alpha.merge_events(self.beta.events())
        self.assertEqual(self.task(self.alpha, "S2")["owner"], STATION_B)
        self.assertIsNone(self.alpha.peek_robot_task())

    def test_restart_marks_interrupted_pick_once(self) -> None:
        self.share()
        s1 = self.task(self.alpha, "S1")["task_id"]
        self.alpha.claim_robot_task(s1)
        directory = self.alpha.data_dir
        self.alpha.close()
        revived = Station(STATION_A, directory, "robot")
        self.addCleanup(revived.close)
        self.assertEqual(self.task(revived, "S1")["state"], "Needs review")
        self.assertIn("Not retried", revived.events()[-1]["note"])
        revived.close()
        again = Station(STATION_A, directory, "robot")
        self.addCleanup(again.close)
        reviews = [event for event in again.events() if event["kind"] == "task_needs_review"]
        self.assertEqual(len(reviews), 1)

    def test_only_isaac_evidence_verifies(self) -> None:
        self.share()
        s1 = self.task(self.alpha, "S1")["task_id"]
        self.alpha.claim_robot_task(s1)
        self.alpha.finish_robot_task(
            s1,
            {"verified": True, "evidence": {"simulator": "stub-sim", "position": [0, 0, 0]}},
        )
        self.assertEqual(self.task(self.alpha, "S1")["state"], "Needs review")
        self.assertNotEqual(self.alpha.peek_robot_task()["task_id"], s1)
        s2 = self.task(self.alpha, "S2")["task_id"]
        self.alpha.claim_robot_task(s2)
        self.alpha.finish_robot_task(
            s2,
            {
                "verified": True,
                "evidence": {
                    "simulator": "isaac-sim",
                    "position": [0.1, -0.16, 0.01],
                    "inside_zone": True,
                    "released": True,
                },
            },
        )
        done = self.task(self.alpha, "S2")
        self.assertEqual(done["state"], "Verified complete")
        self.assertEqual(done["verification"], "simulation")
        self.beta.begin_operator_task(self.task(self.beta, "S4")["task_id"])
        self.beta.confirm_operator_task(self.task(self.beta, "S4")["task_id"])
        human = self.task(self.beta, "S4")
        self.assertEqual(human["verification"], "operator")
        self.assertNotEqual(human["verification"], done["verification"])


class ProcessTest(unittest.TestCase):
    def test_live_peers_partition_and_reconnect(self) -> None:
        root = Path(tempfile.mkdtemp(prefix="kitmesh-"))
        self.addCleanup(lambda: shutil.rmtree(root, ignore_errors=True))
        env = {**os.environ, "PYTHONPATH": str(ROOT)}
        python = sys.executable
        commands = [
            [python, "-m", "kitmesh.stub_sim", "--port", "19401"],
            [python, "-m", "kitmesh.service", "--station", "station-a", "--role", "robot",
             "--data", str(root / "a"), "--listen-port", "19101", "--peer-port", "19102",
             "--http-port", "19201", "--sim-host", "127.0.0.1", "--sim-port", "19401"],
            [python, "-m", "kitmesh.service", "--station", "station-b", "--role", "operator",
             "--data", str(root / "b"), "--listen-port", "19102", "--peer-port", "19101",
             "--http-port", "19202"],
            [python, "-m", "kitmesh.dashboard", "--port", "18080",
             "--station-a", "http://127.0.0.1:19201", "--station-b", "http://127.0.0.1:19202"],
        ]
        processes = []
        logs = []
        try:
            for index, command in enumerate(commands):
                log = (root / f"proc-{index}.log").open("w")
                logs.append(log)
                processes.append(subprocess.Popen(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, text=True))
            self.assertTrue(wait_until(lambda: _port_open(19201) and _port_open(19202) and _port_open(18080), 8))
            post("http://127.0.0.1:19201/hold", {"enabled": True})
            post("http://127.0.0.1:19201/order", {"lines": list(DEMO_ORDER)})
            def agreed() -> bool:
                left = get("http://127.0.0.1:19201/status")
                right = get("http://127.0.0.1:19202/status")
                if not left["orders"] or not right["orders"]:
                    return False
                return left["orders"][0]["established"] and right["orders"][0]["established"] and _owners(left) == _owners(right)
            self.assertTrue(wait_until(agreed, 6), "stations did not share the order")
            post("http://127.0.0.1:19201/hold", {"enabled": False})
            time.sleep(1.2)
            held_open = get("http://127.0.0.1:19201/status")
            s1_state = next(task["state"] for task in held_open["tasks"] if task["stock_id"] == "S1")
            self.assertEqual(s1_state, "Pending", "stub simulator must not start a robot pick")
            post("http://127.0.0.1:18080/api/partition", {"enabled": True})
            post("http://127.0.0.1:18080/api/order", {"station": "station-a", "lines": list(WAITING_ORDER)})
            time.sleep(0.4)
            left = get("http://127.0.0.1:19201/status")
            right = get("http://127.0.0.1:19202/status")
            self.assertEqual(len(left["orders"]), 2)
            self.assertEqual(len(right["orders"]), 1)
            s4 = next(task for task in right["tasks"] if task["stock_id"] == "S4")
            post("http://127.0.0.1:19202/operator/begin", {"task_id": s4["task_id"]})
            post("http://127.0.0.1:19202/operator/confirm", {"task_id": s4["task_id"]})
            stale = get("http://127.0.0.1:19201/status")
            stale_s4 = next(task for task in stale["tasks"] if task["stock_id"] == "S4")
            self.assertNotEqual(stale_s4["verification"], "operator")
            post("http://127.0.0.1:18080/api/partition", {"enabled": False})
            def converged() -> bool:
                left = get("http://127.0.0.1:19201/status")
                right = get("http://127.0.0.1:19202/status")
                if len(left["orders"]) != 2 or len(right["orders"]) != 2:
                    return False
                left_s4 = next(task for task in left["tasks"] if task["stock_id"] == "S4")
                right_s4 = next(task for task in right["tasks"] if task["stock_id"] == "S4")
                return left_s4["verification"] == "operator" and right_s4["verification"] == "operator" and _owners(left) == _owners(right)
            self.assertTrue(wait_until(converged, 6), "records did not converge after reconnect")
            page = urlopen("http://127.0.0.1:18080/", timeout=3).read().decode()
            self.assertIn("does not assign tasks", page)
            self.assertIn("Operator confirmed", page)
        finally:
            for process in processes:
                process.terminate()
            for process in processes:
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
            for log in logs:
                log.close()
            if any(process.returncode not in (0, None, -15) for process in processes):
                for path in sorted(root.glob("proc-*.log")):
                    print(path.read_text()[-2000:])


def _owners(record: dict) -> dict:
    return {task["stock_id"]: task["owner"] for task in record["tasks"]}


def _all_events(record: dict) -> list:
    return record["events"]


def _port_open(port: int) -> bool:
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=0.2):
            return True
    except OSError:
        return False


if __name__ == "__main__":
    unittest.main()
