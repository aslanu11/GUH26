"""One station's queue, event history, and ownership rules.

A station is single-threaded. The process wrapper serializes calls. Event
history is the only source of task truth; heartbeats are not events.
"""

from __future__ import annotations

import fcntl
import json
import os
import uuid
from datetime import datetime, timezone
from pathlib import Path

from kitmesh.layout import STATIONS, can_perform, default_owner

STATES = ("Pending", "In progress", "Verified complete", "Blocked", "Needs review")
ROLES = ("robot", "operator")


def new_id(prefix: str) -> str:
    return f"{prefix}_{uuid.uuid4().hex[:12]}"


def now_ts() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


def ack_event_id(order_id: str, station_id: str) -> str:
    return f"evt_ack_{order_id}_{station_id}"


class EventLog:
    def __init__(self, path: Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.events: list[dict] = []
        self._ids: set[str] = set()
        if self.path.exists():
            for line in self.path.read_text().splitlines():
                if not line.strip():
                    continue
                try:
                    event = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if not _valid_event(event) or event["event_id"] in self._ids:
                    continue
                self._ids.add(event["event_id"])
                self.events.append(event)

    def append(self, event: dict) -> bool:
        if not _valid_event(event):
            raise ValueError("Event is missing event_id, ts, station, or kind")
        if event["event_id"] in self._ids:
            return False
        line = json.dumps(event, sort_keys=True, separators=(",", ":"))
        with self.path.open("a") as handle:
            handle.write(line + "\n")
            handle.flush()
            os.fsync(handle.fileno())
        self._ids.add(event["event_id"])
        self.events.append(event)
        return True

    def merge(self, events: list[dict]) -> int:
        added = 0
        ordered = sorted((event for event in events if _valid_event(event)), key=_event_key)
        for event in ordered:
            if self.append(event):
                added += 1
        return added


def _valid_event(event: dict) -> bool:
    return (
        isinstance(event, dict)
        and isinstance(event.get("event_id"), str)
        and isinstance(event.get("ts"), str)
        and isinstance(event.get("station"), str)
        and isinstance(event.get("kind"), str)
    )


def _event_key(event: dict) -> tuple:
    rank = {
        "order_created": 0,
        "ownership_ack": 1,
        "handover_offered": 2,
        "handover_accepted": 3,
        "task_started": 4,
        "task_blocked": 4,
        "task_unblocked": 5,
        "task_sim_verified": 6,
        "task_operator_confirmed": 6,
        "task_needs_review": 7,
    }.get(event.get("kind"), 5)
    return (event.get("ts") or "", rank, event.get("event_id") or "")


def fold(events: list[dict]) -> dict:
    """Replay events. A verification event sticks; a later review note cannot erase it."""
    orders: dict[str, dict] = {}
    tasks: dict[str, dict] = {}
    for event in sorted(events, key=_event_key):
        kind = event["kind"]
        payload = event.get("payload") or {}
        if kind == "order_created":
            order_id = event["order_id"]
            if order_id in orders:
                continue
            orders[order_id] = {
                "order_id": order_id,
                "origin": payload.get("origin"),
                "acks": set(),
                "task_ids": [],
            }
            for line in payload.get("lines") or []:
                task_id = line["task_id"]
                tasks[task_id] = {
                    "task_id": task_id,
                    "order_id": order_id,
                    "cube_id": line["cube_id"],
                    "stock_id": line["stock_id"],
                    "zone_id": line["zone_id"],
                    "state": "Pending",
                    "owner": None,
                    "verification": None,
                    "reason": "",
                    "evidence": None,
                    "offer": None,
                    "started_by": None,
                }
                orders[order_id]["task_ids"].append(task_id)
        elif kind == "ownership_ack":
            order = orders.get(event.get("order_id"))
            if order is None:
                continue
            order["acks"].add(payload.get("station"))
            if set(STATIONS).issubset(order["acks"]):
                for task_id in order["task_ids"]:
                    task = tasks[task_id]
                    if task["owner"] is None and task["state"] == "Pending":
                        task["owner"] = default_owner(task["stock_id"], task["zone_id"])
        elif kind == "task_started":
            task = tasks.get(event.get("task_id"))
            if task is None or task["verification"]:
                continue
            task["state"] = "In progress"
            task["started_by"] = event.get("station")
            task["reason"] = ""
        elif kind in ("task_sim_verified", "task_operator_confirmed"):
            task = tasks.get(event.get("task_id"))
            if task is None or task["verification"]:
                continue
            task["state"] = "Verified complete"
            task["verification"] = "simulation" if kind == "task_sim_verified" else "operator"
            task["reason"] = ""
            task["evidence"] = payload.get("evidence")
            task["offer"] = None
        elif kind == "task_needs_review":
            task = tasks.get(event.get("task_id"))
            if task is None or task["verification"]:
                continue
            task["state"] = "Needs review"
            task["reason"] = payload.get("reason") or "Needs review"
            task["offer"] = None
        elif kind == "task_blocked":
            task = tasks.get(event.get("task_id"))
            if task is None or task["state"] != "Pending" or task["verification"]:
                continue
            task["state"] = "Blocked"
            task["reason"] = payload.get("reason") or "Blocked"
        elif kind == "task_unblocked":
            task = tasks.get(event.get("task_id"))
            if task is None or task["state"] != "Blocked":
                continue
            if task["reason"] != "execution stopped":
                continue
            task["state"] = "Pending"
            task["reason"] = ""
        elif kind == "handover_offered":
            task = tasks.get(event.get("task_id"))
            if task is None or task["state"] not in ("Pending", "Blocked") or task["verification"]:
                continue
            task["offer"] = {
                "offer_id": payload["offer_id"],
                "to": payload["to"],
                "from": event.get("station"),
            }
        elif kind == "handover_accepted":
            task = tasks.get(event.get("task_id"))
            if task is None or task["verification"] or task["state"] not in ("Pending", "Blocked"):
                continue
            offer = task.get("offer")
            if not offer or offer["offer_id"] != payload.get("offer_id"):
                continue
            task["owner"] = payload.get("to")
            task["offer"] = None
            task["state"] = "Pending"
            task["reason"] = ""
    return {"orders": orders, "tasks": tasks}


class Station:
    def __init__(self, station_id: str, data_dir: str | Path, role: str):
        if station_id not in STATIONS:
            raise ValueError(f"Unknown station {station_id}")
        if role not in ROLES:
            raise ValueError(f"Unknown role {role}")
        self.station_id = station_id
        self.role = role
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self._lock_file = (self.data_dir / "station.lock").open("a+")
        try:
            fcntl.flock(self._lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(f"{station_id} already has a process using {self.data_dir}") from exc
        self.log = EventLog(self.data_dir / "events.jsonl")
        self.execution_enabled = True
        self.hold_next = False
        self.partitioned = False
        self.isolated = False
        self.peer_heartbeat_at: float | None = None
        self._load_runtime()
        self._recover_interrupted()

    def close(self) -> None:
        if self._lock_file is not None:
            fcntl.flock(self._lock_file.fileno(), fcntl.LOCK_UN)
            self._lock_file.close()
            self._lock_file = None

    def events(self) -> list[dict]:
        return list(self.log.events)

    def folded(self) -> dict:
        return fold(self.log.events)

    def apply_heartbeat(self, ts: float | None) -> None:
        """Record peer liveness. A heartbeat never grants or moves ownership."""
        self.peer_heartbeat_at = ts

    def set_partition(self, enabled: bool) -> None:
        self.partitioned = bool(enabled)
        self._save_runtime()

    def set_isolated(self, enabled: bool) -> None:
        """Run from the local record only. No peer link and no shared acks."""
        self.isolated = bool(enabled)
        if self.isolated:
            self.partitioned = True
        self._save_runtime()

    def set_hold_next(self, enabled: bool) -> None:
        self.hold_next = bool(enabled)
        self._save_runtime()

    def submit_order(self, lines: list[dict]) -> str:
        cleaned = _validate_lines(lines)
        order_id = new_id("ord")
        stamped = []
        for index, line in enumerate(cleaned, start=1):
            stamped.append(
                {
                    "task_id": f"task_{order_id}_{index:02d}_{line['stock_id']}",
                    "cube_id": line["cube_id"],
                    "stock_id": line["stock_id"],
                    "zone_id": line["zone_id"],
                }
            )
        self._emit(
            "order_created",
            order_id=order_id,
            note=f"Order {order_id} created with {len(stamped)} tasks",
            payload={"origin": self.station_id, "lines": stamped},
            event_id=f"evt_order_{order_id}",
        )
        self._ack(order_id)
        return order_id

    def merge_events(self, events: list[dict]) -> int:
        added = self.log.merge(events)
        added += self.ensure_acks()
        return added

    def ensure_acks(self) -> int:
        created = 0
        folded = self.folded()
        for order_id, order in folded["orders"].items():
            if self.station_id not in order["acks"]:
                self._ack(order_id)
                created += 1
        return created

    def order_established(self, order_id: str) -> bool:
        order = self.folded()["orders"].get(order_id)
        if not order:
            return False
        if self.isolated:
            return self.station_id in order["acks"]
        return set(STATIONS).issubset(order["acks"])

    def peek_robot_task(self) -> dict | None:
        if self.role != "robot" or not self.execution_enabled or self.hold_next:
            return None
        folded = self.folded()
        if any(
            task["state"] == "In progress" and self._responsible(task, folded) == self.station_id
            for task in folded["tasks"].values()
        ):
            return None
        return self._next_runnable(folded)

    def claim_robot_task(self, task_id: str) -> dict | None:
        peeked = self.peek_robot_task()
        if peeked is None or peeked["task_id"] != task_id:
            return None
        self._emit(
            "task_started",
            order_id=peeked["order_id"],
            task_id=task_id,
            note=f"{self.station_id} started {peeked['cube_id']} from {peeked['stock_id']}",
        )
        return self._task(task_id)

    def finish_robot_task(self, task_id: str, result: dict) -> None:
        task = self._task(task_id)
        if task is None or task["state"] != "In progress":
            return
        evidence = result.get("evidence") if isinstance(result.get("evidence"), dict) else {}
        # A local stub, a dashboard click, or a lost connection cannot count as Isaac verification.
        if result.get("verified") and evidence.get("simulator") == "isaac-sim":
            self._emit(
                "task_sim_verified",
                order_id=task["order_id"],
                task_id=task_id,
                note=f"Simulation verified {task['cube_id']} released inside {task['zone_id']}",
                payload={"evidence": evidence},
            )
            return
        reason = result.get("reason") or "Pick was not verified in Isaac Sim"
        self._emit(
            "task_needs_review",
            order_id=task["order_id"],
            task_id=task_id,
            note=f"Needs review: {reason} Not retried.",
            payload={"reason": reason},
        )

    def begin_operator_task(self, task_id: str) -> None:
        if self.role != "operator":
            raise ValueError("Only the human station begins an operator pick")
        task = self._require_runnable(task_id)
        self._emit(
            "task_started",
            order_id=task["order_id"],
            task_id=task_id,
            note=f"Operator began {task['cube_id']} (not a robot action)",
        )

    def confirm_operator_task(self, task_id: str) -> None:
        if self.role != "operator":
            raise ValueError("Only the human station can record an operator confirmation")
        task = self._task(task_id)
        if task is None or task["owner"] != self.station_id or task["state"] != "In progress":
            raise ValueError("Operator confirmation requires an in-progress task owned here")
        self._emit(
            "task_operator_confirmed",
            order_id=task["order_id"],
            task_id=task_id,
            note=f"Operator confirmed {task['cube_id']} (not verified in simulation)",
            payload={"evidence": {"source": "operator"}},
        )

    def uncertain_operator_task(self, task_id: str) -> None:
        if self.role != "operator":
            raise ValueError("Only the human station can mark an operator pick uncertain")
        task = self._task(task_id)
        if task is None or task["owner"] != self.station_id or task["state"] != "In progress":
            raise ValueError("Only an in-progress operator task can be marked uncertain")
        self._emit(
            "task_needs_review",
            order_id=task["order_id"],
            task_id=task_id,
            note="Needs review: operator marked the pick uncertain. Not retried.",
            payload={"reason": "Operator marked the pick uncertain"},
        )

    def stop_execution(self) -> None:
        self.execution_enabled = False
        self._save_runtime()
        for task in self._owned_tasks():
            if task["state"] == "Pending" and not task["offer"]:
                self._emit(
                    "task_blocked",
                    order_id=task["order_id"],
                    task_id=task["task_id"],
                    note=f"Blocked {task['cube_id']} because execution stopped",
                    payload={"reason": "execution stopped"},
                )

    def resume_execution(self) -> None:
        self.execution_enabled = True
        self._save_runtime()
        for task in self._owned_tasks():
            if task["state"] == "Blocked" and task["reason"] == "execution stopped" and not task["offer"]:
                self._emit(
                    "task_unblocked",
                    order_id=task["order_id"],
                    task_id=task["task_id"],
                    note=f"Returned {task['cube_id']} to Pending",
                )

    def offer_handover(self, task_id: str, to_station: str) -> str:
        if self.partitioned:
            raise ValueError(
                "Handover needs a live peer connection. A missed heartbeat does not transfer ownership."
            )
        if to_station not in STATIONS or to_station == self.station_id:
            raise ValueError("Handover target must be the other station")
        task = self._task(task_id)
        if task is None or task["owner"] != self.station_id:
            raise ValueError("Only the current owner can offer a handover")
        if task["state"] == "Pending":
            pass
        elif task["state"] == "Blocked" and task["reason"] == "execution stopped":
            pass
        else:
            raise ValueError("Only an unstarted task can be handed over")
        if task["offer"]:
            raise ValueError("A handover offer is already open")
        if not can_perform(to_station, task["stock_id"], task["zone_id"]):
            raise ValueError(f"{to_station} cannot reach {task['stock_id']} and {task['zone_id']}")
        offer_id = new_id("offer")
        self._emit(
            "handover_offered",
            order_id=task["order_id"],
            task_id=task_id,
            note=f"Offered {task['cube_id']} to {to_station}",
            payload={"offer_id": offer_id, "to": to_station},
            event_id=f"evt_offer_{offer_id}",
        )
        return offer_id

    def accept_handover(self, task_id: str) -> None:
        if self.partitioned:
            raise ValueError(
                "Accepting a handover needs a live peer connection. A missed heartbeat does not transfer ownership."
            )
        task = self._task(task_id)
        offer = task.get("offer") if task else None
        if task is None or not offer or offer["to"] != self.station_id:
            raise ValueError("There is no handover offer for this station")
        if not can_perform(self.station_id, task["stock_id"], task["zone_id"]):
            raise ValueError("This station cannot perform that task")
        self._emit(
            "handover_accepted",
            order_id=task["order_id"],
            task_id=task_id,
            note=f"Accepted {task['cube_id']} from {offer['from']}",
            payload={"offer_id": offer["offer_id"], "to": self.station_id},
            event_id=f"evt_accept_{offer['offer_id']}",
        )

    def status(self, *, now: float | None = None, simulator: dict | None = None) -> dict:
        import time

        folded = self.folded()
        moment = time.time() if now is None else now
        age = None if self.peer_heartbeat_at is None else max(0.0, moment - self.peer_heartbeat_at)
        tasks = []
        for task in self._sorted_tasks(folded):
            other = next(station for station in STATIONS if station != self.station_id)
            eligible = other if can_perform(other, task["stock_id"], task["zone_id"]) else None
            established = self.order_established(task["order_id"])
            shown = dict(task)
            if shown["owner"] is None:
                shown["owner"] = self._responsible(task, folded)
            tasks.append({**shown, "offer": task["offer"], "established": established, "eligible_handover_to": eligible})
        orders = []
        for order in folded["orders"].values():
            orders.append(
                {
                    "order_id": order["order_id"],
                    "origin": order["origin"],
                    "established": self.order_established(order["order_id"]),
                    "acks": sorted(station for station in order["acks"] if isinstance(station, str)),
                    "task_ids": list(order["task_ids"]),
                }
            )
        return {
            "station": self.station_id,
            "role": self.role,
            "execution_enabled": self.execution_enabled,
            "hold_next": self.hold_next,
            "partitioned": self.partitioned,
            "isolated": self.isolated,
            "peer_heartbeat_age_s": age,
            "orders": orders,
            "tasks": tasks,
            "events": list(reversed(self.log.events[-40:])),
            "simulator": simulator or {"name": None, "connected": False, "ready": False},
        }

    def _recover_interrupted(self) -> None:
        for task in self._owned_tasks():
            if task["state"] == "In progress":
                self._emit(
                    "task_needs_review",
                    order_id=task["order_id"],
                    task_id=task["task_id"],
                    note=f"Needs review: {task['cube_id']} was in progress at restart. Not retried.",
                    payload={"reason": "Interrupted pick was not verified at restart"},
                )
        if not self.execution_enabled:
            for task in self._owned_tasks():
                if task["state"] == "Pending" and not task["offer"]:
                    self._emit(
                        "task_blocked",
                        order_id=task["order_id"],
                        task_id=task["task_id"],
                        note=f"Blocked {task['cube_id']} because execution is stopped",
                        payload={"reason": "execution stopped"},
                    )

    def _ack(self, order_id: str) -> None:
        self._emit(
            "ownership_ack",
            order_id=order_id,
            note=f"{self.station_id} acknowledged {order_id}",
            payload={"station": self.station_id},
            event_id=ack_event_id(order_id, self.station_id),
        )

    def _emit(
        self,
        kind: str,
        *,
        order_id: str | None = None,
        task_id: str | None = None,
        note: str = "",
        payload: dict | None = None,
        event_id: str | None = None,
    ) -> dict:
        event = {
            "event_id": event_id or new_id("evt"),
            "ts": now_ts(),
            "station": self.station_id,
            "kind": kind,
            "order_id": order_id,
            "task_id": task_id,
            "note": note,
            "payload": payload or {},
        }
        self.log.append(event)
        return event

    def _task(self, task_id: str) -> dict | None:
        return self.folded()["tasks"].get(task_id)

    def _sorted_tasks(self, folded: dict | None = None) -> list[dict]:
        folded = self.folded() if folded is None else folded
        return [folded["tasks"][task_id] for task_id in sorted(folded["tasks"])]

    def _owned_tasks(self) -> list[dict]:
        folded = self.folded()
        return [task for task in self._sorted_tasks(folded) if self._responsible(task, folded) == self.station_id]

    def _responsible(self, task: dict, folded: dict) -> str | None:
        if task["owner"]:
            return task["owner"]
        if not self.isolated:
            return None
        order = folded["orders"].get(task["order_id"])
        if not order or self.station_id not in order["acks"]:
            return None
        return default_owner(task["stock_id"], task["zone_id"])

    def _next_runnable(self, folded: dict) -> dict | None:
        busy_stocks = {
            task["stock_id"]
            for task in folded["tasks"].values()
            if task["state"] == "In progress"
        }
        for task in self._sorted_tasks(folded):
            if self._responsible(task, folded) != self.station_id or task["state"] != "Pending" or task["offer"]:
                continue
            if not self.order_established(task["order_id"]):
                continue
            if task["stock_id"] in busy_stocks:
                continue
            return task
        return None

    def _require_runnable(self, task_id: str) -> dict:
        if not self.execution_enabled:
            raise ValueError("Execution is stopped at this station")
        task = self._task(task_id)
        if task is None:
            raise ValueError("Unknown task")
        if task["owner"] != self.station_id:
            raise ValueError("This station does not own that task")
        if task["state"] != "Pending" or task["offer"]:
            raise ValueError("The task is not waiting to start")
        if not self.order_established(task["order_id"]):
            raise ValueError("The order has not been shared with the peer yet")
        return task

    def _load_runtime(self) -> None:
        path = self.data_dir / "runtime.json"
        if not path.exists():
            self._save_runtime()
            return
        try:
            data = json.loads(path.read_text())
        except json.JSONDecodeError:
            return
        self.execution_enabled = bool(data.get("execution_enabled", True))
        self.hold_next = bool(data.get("hold_next", False))
        self.partitioned = bool(data.get("partitioned", False))
        self.isolated = bool(data.get("isolated", False))

    def _save_runtime(self) -> None:
        path = self.data_dir / "runtime.json"
        payload = {
            "execution_enabled": self.execution_enabled,
            "hold_next": self.hold_next,
            "partitioned": self.partitioned,
            "isolated": self.isolated,
            "role": self.role,
            "station": self.station_id,
        }
        temporary = path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(payload, indent=2, sort_keys=True))
        temporary.replace(path)


def _validate_lines(lines: list[dict]) -> list[dict]:
    if not isinstance(lines, list) or not lines:
        raise ValueError("An order needs at least one line")
    cleaned = []
    seen = set()
    for line in lines:
        stock_id = line.get("stock_id")
        zone_id = line.get("zone_id")
        cube_id = line.get("cube_id")
        if not isinstance(cube_id, str) or not cube_id:
            raise ValueError("Each line needs a cube id")
        owner = default_owner(stock_id, zone_id)
        if stock_id in seen:
            raise ValueError(f"Stock {stock_id} is repeated in one order")
        seen.add(stock_id)
        cleaned.append({"cube_id": cube_id, "stock_id": stock_id, "zone_id": zone_id, "default_owner": owner})
    return cleaned
