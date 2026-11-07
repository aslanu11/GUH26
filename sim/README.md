# Dum-E & U in Isaac Sim

OWNER: Kazy

The simulated half of our Neptune North demo. Two UFactory UF850 arms, **Dum-E** and **U**, pack parcels at the same time in Isaac Sim 5.1. **They share nothing while they run**: no messages, no shared records, no heartbeat. If one of them dies, the other notices that its lane stopped emptying, and finishes its parcels.

The physical SO-101 demo in the repo root uses a different approach: agents that exchange state over UDP and keep working when the link drops. This folder takes it to the extreme, with zero communication from the start.

## How it works

- **The plan, fixed before the job** (`kitmesh/cell_layout.py`): Dum-E owns parcels P1–P3 and carton CA, and U owns P4–P6 and carton CB. Both arms can reach every parcel.
- **One agent process per robot** (`kitmesh/cell_agent.py`). It learns only two things from the simulator:
  - `observe`: parcel positions, the other arm's link positions and the sim clock, what a camera over the cell would see;
  - the replies to its own `pick` requests.
- **The failure rule:** the other lane still has parcels on their marks, and that count hasn't dropped for **25 s of sim time** (about two pick cycles). The other robot is then presumed stopped.
- **Takeover:** finish your own lane first, then take the other lane from the end it would have reached last. Never reach within 0.22 m of the other arm.
- **The sim** (`kitmesh/sim_worker.py`) runs both arms at once: each control step advances both arms, then physics steps once. Suction is Isaac Sim's surface gripper, a joint that holds the parcel only while suction is on. A pick counts only if physics shows that the parcel left its mark, sits inside the carton walls, has stopped moving, and suction is off (`KITMESH_PICK_RESULT ... "verified": true` in the sim log).
- **The demo page** (`kitmesh/demo.py` + `demo.html`, port 8080) shows:
  - a live view of the Isaac window, with drag to orbit and scroll to zoom;
  - a top-down map of the table;
  - buttons: Start job, Crash software / Fault arm per robot, Recover, Reset scene;
  - each robot's own log.

## Run it

You need Isaac Sim 5.1 + Isaac Lab 2.3.2 (`ISAAC_ACTIVATE`, default `/home/cal/isaac/activate.sh`), an NVIDIA GPU with `nvidia-smi` working, and an X display on `:0`. Then:

```bash
sim/scripts/start_cell.sh
```

That starts whatever isn't running: the sim, both agents, the demo page, and a hold that keeps the display awake. The page is at <http://127.0.0.1:8080>, and also on this machine's Tailscale address if it has one.

Make Isaac fill the screen, so the page's video is large:

```bash
python3 sim/scripts/arrange_windows.py --isaac-full
```

The 90-second demo script is in `JUDGE.md`.

## Tests (no simulator needed)

```bash
cd sim && PYTHONPATH=. python3 -m unittest discover -s tests -v
```

There are 18 tests. `test_cell_brain.py` covers failure inference, takeover order, ignoring your own takeover picks, the arm-clearance rule and when a job is done. `test_coordination.py` covers the earlier peer-coordination layer (`station.py`, `service.py`, `dashboard.py`, `layout.py`, `stub_sim.py`), which the two-robot cell no longer uses.

## Results from one recorded run

Dum-E's software was frozen 3 s into a job, while its arm was already moving P1, so P1 was still packed. At 28.8 s of sim time, U worked out from the table alone that Dum-E had stopped. U finished P6, P5 and P4, then took over P3 and P2. All 6 parcels were verified, with 0 messages between the robots.

## What it does not claim

- **Slow but alive:** a robot that is slow but still working could, in principle, reach for the same parcel as the other. Reverse order plus a "still on its mark?" check make that unlikely, not impossible. A parcel that has already gone is logged as such, not as a failure.
- **Speed:** the sim runs at about 0.7× real time, with CPU physics because the surface gripper requires it.
- **Arm gravity:** off on the arm links, for tight tracking. Parcels have normal gravity.
