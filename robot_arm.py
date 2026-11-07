"""
robot_arm.py - OWNER: Kazy. The robot agent's body: teach picks by hand, replay them.

No inverse kinematics, no ML. For each slot you move the follower by hand (torque off) through
the pick, saving waypoints. The agent replays: HOME -> slot waypoints -> HOME.

Commands (run on the laptop the FOLLOWER is plugged into, with MakerMods Lab closed):
    python robot_arm.py ports                         # find the follower's port
    python robot_arm.py teach --port COM5 --name home # teach the HOME pose (1 waypoint)
    python robot_arm.py teach --port COM5 --name 3    # teach the pick for slot 3
    python robot_arm.py play  --port COM5 --name 3    # replay slot 3 (home -> pick -> home)

Teach keys (type then Enter):
    <Enter>  save waypoint          g  save GRASP waypoint (squeezes the gripper a bit harder)
    u        undo last              p  preview (plays it back, then torque off again)
    s        save and quit          q  quit without saving

Recipe for a slot: start above the object -> lower -> close the gripper by hand on the object,
press g -> lift -> move over YOUR end of the tray -> open the gripper by hand, Enter -> lift, Enter.
The interface the agent uses (do not change without telling Aslan):
    FollowerReplay(port, arm_id, motions_dir).execute(slot, should_pause, should_abort)
        -> "ok" | "failed" | "empty" | "aborted"
    .arm_ok (bool), .reconnect() -> bool
"""
import argparse
import json
import time
from pathlib import Path

from servo_bus import ARM_JOINTS, JOINTS, Bus, list_ports

SPEED_TICKS_S = 600     # joint speed during replay (4096 ticks = 360 degrees)
GRIPPER_TICKS_S = 900
MIN_SEGMENT_S = 0.4
GRASP_SQUEEZE = 100     # extra ticks past contact so the grasp actually holds
GRASP_DWELL_S = 0.4
RELEASE_TICKS = 300   # opening the gripper more than this after a grasp = release
EMPTY_GRASP_TICKS = 35  # gripper got within this of its squeeze target = it closed on nothing (tune!)
HZ = 50


def load_motion(motions_dir, name):
    p = Path(motions_dir) / f"{name}.json"
    if not p.exists():
        raise FileNotFoundError(f"no taught motion {p} - run: python robot_arm.py teach --name {name}")
    return json.loads(p.read_text())


def full_path(motions_dir, slot):
    """HOME -> slot waypoints -> HOME (home is optional but strongly recommended)."""
    wps = load_motion(motions_dir, str(slot))
    try:
        home = load_motion(motions_dir, "home")
        return home + wps + home
    except FileNotFoundError:
        return wps


def hold_grip(waypoints):
    """After a GRASP waypoint, keep the squeezed gripper value until the gripper is clearly opened.
    (When teaching, your hand lets go of the jaws after 'g', so later waypoints record a looser grip.)"""
    out, hold = [], None
    for wp in waypoints:
        wp = {"pos": dict(wp["pos"]), "grip": wp.get("grip", False)}
        if wp["grip"]:
            hold = wp["pos"]["gripper"]
        elif hold is not None:
            if abs(wp["pos"]["gripper"] - hold) < RELEASE_TICKS:
                wp["pos"]["gripper"] = hold      # still carrying the object: keep squeezing
            else:
                hold = None                      # gripper opened = released
        out.append(wp)
    return out


def play(bus, waypoints, should_pause=lambda: False, should_abort=lambda: False, check_grasp=False):
    """Interpolate through waypoints. Abort is honoured until the first grasp, then we retrace home.
    check_grasp: after the grasp, if the gripper closed (almost) fully there's nothing in it -> open,
    retrace back the way we came, return "missed"."""
    waypoints = hold_grip(waypoints)
    cur = bus.read_positions()
    prev = {"pos": cur, "grip": False}
    visited, grasped = [prev], False
    for wp in waypoints:
        a, b = prev["pos"], wp["pos"]
        d_arm = max(abs(b[j] - a[j]) for j in ARM_JOINTS)
        d_grip = abs(b["gripper"] - a["gripper"])
        dur = max(MIN_SEGMENT_S, d_arm / SPEED_TICKS_S, d_grip / GRIPPER_TICKS_S)
        t = 0.0
        while t < dur:
            if not grasped and should_abort():
                for back in reversed(visited[:-1]):  # retrace the way we came
                    _move(bus, visited[-1]["pos"], back["pos"])
                    visited.append(back)
                return "aborted"
            if should_pause():
                time.sleep(0.05)
                continue
            t = min(dur, t + 1.0 / HZ)
            s = t / dur
            s = s * s * (3 - 2 * s)  # smoothstep
            bus.write_goals({j: a[j] + (b[j] - a[j]) * s for j in JOINTS})
            time.sleep(1.0 / HZ)
        if wp.get("grip"):
            grasped = True
            time.sleep(GRASP_DWELL_S)
            if check_grasp:
                got = bus.read_positions()["gripper"]
                err = abs(got - wp["pos"]["gripper"])
                print(f"[robot_arm] grasp check: gripper {err} ticks short of target (empty below {EMPTY_GRASP_TICKS})")
                if err < EMPTY_GRASP_TICKS:
                    cur = wp["pos"]
                    for back in reversed(visited[1:]):   # open at the pre-grasp pose, then retrace home
                        _move(bus, cur, back["pos"])
                        cur = back["pos"]
                    return "missed"
        visited.append(wp)
        prev = wp
    return "ok"


def _move(bus, a, b):
    d = max(abs(b[j] - a[j]) for j in JOINTS)
    dur = max(MIN_SEGMENT_S, d / SPEED_TICKS_S)
    steps = int(dur * HZ)
    for i in range(1, steps + 1):
        s = i / steps
        s = s * s * (3 - 2 * s)
        bus.write_goals({j: a[j] + (b[j] - a[j]) * s for j in JOINTS})
        time.sleep(1.0 / HZ)


class FollowerReplay:
    def __init__(self, port, arm_id=None, motions_dir="motions"):
        self.port, self.motions = port, motions_dir
        self.bus = None
        self.arm_ok = False
        self.zone = None
        self.reconnect()

    def reconnect(self):
        try:
            if self.bus:
                self.bus.close()
            self.bus = Bus(self.port)
            self.bus.read_positions()
            self.bus.set_acceleration(40)
            self.bus.set_torque_limit("gripper", 500)  # 50%: avoid burning the gripper servo
            self.bus.torque(True)
            self.arm_ok = True
        except Exception as e:
            print(f"[robot_arm] connect failed: {e}")
            self.arm_ok = False
        return self.arm_ok

    def execute(self, slot, should_pause, should_abort):
        # Tier 2 hook (Tanish/Kazy): check the wrist camera here and return "empty" if the slot is empty.
        try:
            self.zone = str(slot)
            return play(self.bus, full_path(self.motions, slot), should_pause, should_abort, check_grasp=True)
        except FileNotFoundError as e:
            print(f"[robot_arm] {e}")
            return "failed"
        except Exception as e:
            print(f"[robot_arm] arm error: {e}")
            self.arm_ok = False
            return "failed"
        finally:
            self.zone = "home"


# ------------------------------------------------------------------ CLI
def teach(port, name, motions_dir):
    bus = Bus(port)
    bus.torque(False)
    print(f"Torque OFF - move the arm by hand. Teaching '{name}'. Keys: Enter, g, u, p, s, q")
    wps = []
    while True:
        cmd = input(f"[{len(wps)} waypoints] > ").strip().lower()
        if cmd in ("", "g"):
            try:
                pos = bus.read_positions()
            except IOError as e:
                print(f"  !! {e}\n  Check the arm's power and the servo cables, then try again. Your {len(wps)} waypoints are kept.")
                continue
            wp = {"pos": pos, "grip": cmd == "g"}
            if cmd == "g" and wps:  # squeeze further in the closing direction
                direction = 1 if pos["gripper"] >= wps[-1]["pos"]["gripper"] else -1
                wp["pos"] = dict(pos, gripper=pos["gripper"] + direction * GRASP_SQUEEZE)
            wps.append(wp)
            print("  saved", {k: int(v) for k, v in wp["pos"].items()}, "(GRASP)" if wp["grip"] else "")
        elif cmd == "u" and wps:
            wps.pop()
            print("  removed last")
        elif cmd == "p" and wps:
            bus.torque(True)
            try:
                home = load_motion(motions_dir, "home") if name != "home" else []
            except FileNotFoundError:
                home = []
            print("  previewing..."), play(bus, home + wps + home)
            input("  done. Hold the arm, then press Enter to turn torque OFF")
            bus.torque(False)
        elif cmd == "w" and wps:  # emergency save without preview
            Path(motions_dir).mkdir(exist_ok=True)
            (Path(motions_dir) / f"{name}.json").write_text(json.dumps(wps, indent=1))
            print("  saved (no preview)")
        elif cmd == "s" and wps:
            Path(motions_dir).mkdir(exist_ok=True)
            p = Path(motions_dir) / f"{name}.json"
            p.write_text(json.dumps(wps, indent=1))
            print(f"saved {p}")
            break
        elif cmd == "q":
            break
    input("Hold the arm (torque stays OFF), press Enter to exit")
    bus.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["ports", "teach", "play", "read"])
    ap.add_argument("--port")
    ap.add_argument("--name", help="'home' or a slot number")
    ap.add_argument("--motions", default="motions")
    a = ap.parse_args()
    if a.cmd == "ports":
        for dev, desc in list_ports():
            print(dev, "-", desc)
        print("Unplug the arm's USB, run again: the port that disappears is the arm.")
    elif a.cmd == "read":
        bus = Bus(a.port)
        bus.torque(False)
        while True:
            print({k: int(v) for k, v in bus.read_positions().items()})
            time.sleep(0.3)
    elif a.cmd == "teach":
        teach(a.port, a.name, a.motions)
    elif a.cmd == "play":
        arm = FollowerReplay(a.port, None, a.motions)
        path = full_path(a.motions, a.name) if a.name != "home" else load_motion(a.motions, "home")
        print(play(arm.bus, path))


if __name__ == "__main__":
    main()
