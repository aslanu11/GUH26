"""
test_scenarios.py - OWNER: Ayman. Runs a fake robot + fake human on this laptop and checks every
demo moment works. No hardware needed. Run:  python test_scenarios.py
Add a scenario here whenever you find a bug, so it never comes back.
"""
import json
import subprocess
import sys
import tempfile
import time
import urllib.request
from pathlib import Path

HERE = Path(__file__).parent
R, H = "http://127.0.0.1:8100", "http://127.0.0.1:8101"


def post(base, path, body=None):
    req = urllib.request.Request(base + path, data=json.dumps(body or {}).encode(), method="POST")
    return json.loads(urllib.request.urlopen(req, timeout=3).read())["result"]


def state(base):
    return json.loads(urllib.request.urlopen(base + "/state", timeout=3).read())


def wait(cond, timeout, what):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            print(f"  PASS  {what} ({time.time() - t0:.1f}s)")
            return True
        time.sleep(0.3)
    print(f"  FAIL  {what}")
    return False


def statuses(base):
    return {r["slot"]: r for r in state(base)["rows"]}


def main():
    cfg = json.loads((HERE / "config.json").read_text())
    cfg.update(away_s=4.0, down_s=4.0)  # faster for testing
    tmp = Path(tempfile.mkdtemp()) / "config.json"
    tmp.write_text(json.dumps(cfg))
    procs = [
        subprocess.Popen([sys.executable, str(HERE / "agent.py"), "--id", "robot", "--port", "9100",
                          "--peer", "127.0.0.1:9101", "--web", "8100", "--config", str(tmp)], stdout=subprocess.DEVNULL),
        subprocess.Popen([sys.executable, str(HERE / "agent.py"), "--id", "human", "--port", "9101",
                          "--peer", "127.0.0.1:9100", "--web", "8101", "--config", str(tmp)], stdout=subprocess.DEVNULL),
    ]
    ok = True
    try:
        time.sleep(1.5)
        print("1. Normal run: order on robot, both see it, heavy items go to robot, all get done")
        assert post(R, "/order", {"slots": "1,2,3,4,5"}) == "ok"
        ok &= wait(lambda: len(state(H)["rows"]) == 5, 3, "human sees the robot's order")
        ok &= wait(lambda: all(r["status"] == "done" for r in state(R)["rows"]), 60, "all 5 picks done")
        st = statuses(R)
        ok &= wait(lambda: st[4]["owner"] == "robot" and st[5]["owner"] == "robot", 1, "weights 4-5 done by robot")
        ok &= wait(lambda: any(r["owner"] == "human" for r in st.values()), 1, "human did at least one pick")

        print("2. Robot arm fails: light picks go to human, heavy ones blocked")
        post(R, "/reset")
        time.sleep(1)
        post(H, "/order", {"slots": "1,2,3,4,5"})
        ok &= wait(lambda: state(R)["my_next"] is not None, 5, "robot started a pick")
        post(R, "/fail")
        ok &= wait(lambda: all(statuses(H)[s]["status"] == "done" for s in (1, 2, 3)), 40, "human finished 1-3")
        ok &= wait(lambda: all("needs a robot" in (statuses(H)[s]["reason"] or "") for s in (4, 5)), 5, "4-5 blocked: needs a robot")
        post(R, "/armreset")
        ok &= wait(lambda: all(r["status"] == "done" for r in state(R)["rows"]), 40, "robot back -> finishes 4-5")

        print("3. Human takes the robot's pick: robot drops it and replans")
        post(R, "/reset")
        time.sleep(1)
        post(R, "/order", {"slots": "1,4"})
        ok &= wait(lambda: (state(R)["my_next"] or {}).get("slot") is not None, 5, "robot picking")
        target = state(R)["my_next"]["slot"]
        post(H, "/touch", {"slot": target})
        ok &= wait(lambda: any("taken by human" in l for l in state(R)["log"]), 3, f"robot saw human take slot {target}")
        ok &= wait(lambda: all(r["status"] == "done" for r in state(R)["rows"]), 30, "order still completes")

        print("4. Human walks away: robot takes over their light picks")
        post(R, "/reset")
        time.sleep(1)
        post(H, "/walkaway", {"value": True})
        post(H, "/order", {"slots": "1,2,3"})
        ok &= wait(lambda: not state(H)["available"], 10, "human agent marks itself away")
        ok &= wait(lambda: all(r["status"] == "done" for r in state(R)["rows"]), 40, "robot did all 3")
        post(H, "/walkaway", {"value": False})
        ok &= wait(lambda: state(H)["available"], 5, "human back")

        print("5. Comms cut: both keep working, then merge")
        post(R, "/reset")
        time.sleep(1)
        post(R, "/order", {"slots": "1,2,3,4,5"})
        time.sleep(1)
        post(R, "/comms", {"cut": True})
        ok &= wait(lambda: "silent" in state(H)["view"]["robot"]["state"] or "down" in state(H)["view"]["robot"]["state"], 4, "human notices robot silent")
        time.sleep(2)
        post(R, "/comms", {"cut": False})
        ok &= wait(lambda: state(H)["view"]["robot"]["state"] == "online", 4, "reconnected")
        ok &= wait(lambda: all(r["status"] == "done" for r in state(H)["rows"]), 60, "all done after merge")
        same = lambda: [r["status"] for r in state(R)["rows"]] == [r["status"] for r in state(H)["rows"]]
        ok &= wait(same, 3, "both agents agree on the final board")
    finally:
        for p in procs:
            p.terminate()
    print("\nALL PASS" if ok else "\nSOME FAILED")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
