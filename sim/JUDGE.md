# KitMesh — two robots, no communication, one fails

Open **<http://100.124.242.87:8080>** in a browser on the tailnet (Tailscale on). Everything below is done from that page.

Rule: a pick counts only if `logs/sim.log` has `KITMESH_PICK_RESULT` with `"verified": true`. The page's Simulator log shows those lines.

## What the cell is

- **Robots:** two simulated UFactory UF850 arms with suction cups, facing each other across one table in Isaac Sim 5.1.
- **Parcels:** a row of six 1 kg parcels in the middle. Before the shift, the plan gives Robot A lane P1–P3 and carton CA, and Robot B lane P4–P6 and carton CB.
- **Both robots work at the same time.** Each control step moves both arms, then physics steps once for the whole scene.
- **The robots share nothing while they run.** No messages, no shared records, no heartbeats. Each robot's agent (`kitmesh/cell_agent.py`) gets only two things from the simulator:
  - **observe:** parcel positions, the other arm's position, and the sim clock. That's what a camera over the cell would see.
  - **its own pick replies.**
- **How a robot decides the other has stopped:** the other lane still has parcels on their marks, but that count hasn't dropped for **25 s of simulated time** (about two pick cycles). The clock is the simulator's, so a slow machine can't cause a false alarm.
- **Takeover:** the survivor finishes its own lane first, then takes the other lane from the end the stopped robot would have reached last, packing into its *own* carton.
- **Safety:** a robot never reaches within 0.22 m of the other arm. Parcels that close wait, and get re-checked every cycle.
- **What it doesn't guarantee:** if a robot is slow but alive, both could reach for the same parcel. The reverse order and the "still on its mark?" check before every pick make that unlikely, but without communication it can't be ruled out. A parcel that is already gone is logged as "already gone", not as a failure.
- **Suction** is modelled by the Isaac Sim surface gripper: a joint holds the parcel only while suction is on. Nothing is teleported during a pick. **Reset scene** puts parcels back between runs, and the log says so.
- **Simplifications:** physics runs on the CPU, which the surface gripper requires, at about 0.7× real time. Arm links have gravity off.

## Failure buttons

- **Crash software** freezes that robot's agent process (SIGSTOP). Its arm finishes the move it was already given, then gets no more work. **Use this for the clean takeover demo.**
- **Fault arm** stops the arm where it is and turns its suction off. If it stops over the parcel row, the other robot won't reach in next to it. That's the safety rule; it waits.
- **Recover** undoes either one.

## 60–90 second script

| Time | Do | Say |
| --- | --- | --- |
| Before | **Reset scene** | — |
| 0–10 s | Point at the two arms and the top-view map | "Two UF850 robots, one table. Each has its own lane and its own carton. They share no messages and no data. Each one only sees the table." |
| 10–20 s | **Start job** | "Both start at the same time." |
| ~15 s | **Robot A → Crash software** | "Robot A's computer just died. Robot B isn't told anything." |
| 15–45 s | Robot B's card: the "Watching Robot A's lane" meter fills up | "B keeps packing its own lane. It notices A's lane has stopped emptying. After 25 seconds of no change, it decides A has stopped." |
| 45–90 s | B takes P3, then P2 | "B finishes its own parcels, then takes over A's, starting from the end A would have reached last. Every place is checked by Isaac Sim: verified." |
| End | Orbit the camera (drag on the video) | "All simulation, no physical robot, no cloud policy." |

A whole job takes about 1.5 minutes on this machine.

## Bringing it back up

On the Ubuntu box, this starts only the pieces that aren't running (sim, both agents, page):

```bash
/home/cal/kitmesh/scripts/start_cell.sh
```

Then this makes Isaac fill the screen, so the page's video is as large as possible:

```bash
python3 /home/cal/kitmesh/scripts/arrange_windows.py --isaac-full
```

For full desktop control, NoMachine also works: host `100.124.242.87`, port `4000`. Choose the display mode "Scale the remote desktop to fit into the window".

## Tests

```bash
PYTHONPATH=/home/cal/kitmesh /usr/bin/python3 -m unittest discover -s /home/cal/kitmesh/tests -v
```

There are 18 tests. Ten are new: they cover failure inference, takeover order, ignoring your own takeover picks, the arm-clearance rule, and when a job counts as done. The earlier peer-coordination tests (`station.py`, `service.py`) still pass, but that code isn't used by this cell any more.

## History

The first arm was the SO-101 with finger grasps, which wasn't reproducible. Then came single-robot UF850 picks, each station on its own. Old logs and records are in `artifacts/`.
