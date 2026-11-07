# Neptune North Challenge: Human + Robot Fulfilment Cell

**GreatUniHack 2026 · Team: Aslan, Kazy, Ruslan, Tanish, Ayman**

A mini Amazon pick station run by **two independent agents with no central controller**:

- **Robot agent** (laptop 1, Kazy's): the SO-101 follower picks objects with its gripper. It has the overhead camera.
- **Human agent** (laptop 2): a person uses the SO-101 **leader arm as a tracked tool** to push objects into the tray. The leader's encoders tell the system exactly where the human's arm is. **Teleop is OFF: the leader never drives the follower.**

Each object sits in a numbered slot and carries a **post-it weight label from 1 (light) to 5 (heavy)**. Both agents run the **same rule** (distance + weight + queue) on their own copy of the order board, so they agree who picks what without negotiating. **Labels above 3 are robot-only**, for human safety.

## Demo moments (what the judges see)
| Event | What happens |
|---|---|
| Normal order | Picks split between robot and human; heavy ones always go to the robot |
| Human takes the robot's item (e.g. spots a damaged package and bins it) | Robot sees it in the shared state, drops that pick and replans |
| Human's arm enters the robot's slot | Robot **pauses** and resumes when the arm leaves (speed-and-separation monitoring, ISO/TS 15066) |
| Human walks away | Leader stops moving → human agent marks itself away → robot takes the light picks |
| Pull the robot's power | Robot agent releases its pick → human gets the light ones; heavy ones show **"blocked: needs a robot"** and are never given to the human |
| Cut comms (button or wifi off) | Each agent keeps working from its own view. Tier 2: the robot's overhead camera tells "human cut off but working" from "human gone" |
| Reconnect | Boards merge (CRDT), nothing gets done twice |

## How it works (one paragraph)
Each agent sends its full state (orders, pick records, status) over UDP to its peer 4 times a second. That doubles as the heartbeat. Merging is conflict-free: **done is final; otherwise the newest write wins**, ordered by a Lamport clock and then agent id. Every agent runs `allocate()` on its own view: candidates must be alive, available and able to reach the slot, and a human is excluded if the weight is above the limit. Cost = distance/speed + queue × cycle time, plus k × weight for humans; a tie goes to the robot. When an agent is idle, it claims the first pick it wins. A peer is **online** under 1.5 s since its last message; after that it's **silent**, and after 6 s it's **down**, unless the camera says it's still moving.

## Files and owners: only edit your own files
| File | Owner | What |
|---|---|---|
| `agent.py`, `page.html`, `servo_bus.py` | **Aslan** | Agent core, web page, raw servo driver (shared) |
| `config.json` | **Aslan** (Tanish supplies the numbers) | Slot weights, distances, timings |
| `robot_arm.py`, `motions/` | **Kazy** | Teach and replay the follower's picks |
| `human_leader.py`, `zones.json` | **Ruslan** | Leader-arm tracking (zones, trigger, idle) |
| `vision.py` | **Tanish** | Overhead camera motion detection (Tier 2) |
| `test_scenarios.py`, `docs/` | **Ayman** | Automated scenario tests, demo script, pitch |
| `team/<name>.md` | each person | Your task list. Your AI reads this first |

The **interfaces between files** are written at the top of each file. Don't change an interface without telling its owner in Discord.

## Setup (every laptop). Windows PowerShell: run ONE line at a time, no `&&`
```
git clone https://github.com/aslanu11/GUH26.git
cd GUH26
pip install -r requirements.txt
python test_scenarios.py
```
`test_scenarios.py` runs a fake robot + fake human and checks every demo moment, no hardware. It must print `ALL PASS`.

## Run for real
Both laptops on the **same phone hotspot**. Find each laptop's IP (`ipconfig` / `ifconfig`). **Close MakerMods Lab first**: only one program can use an arm's port.

**Laptop 1, robot (Kazy):**
```
python robot_arm.py ports                                   # find the follower's port
python robot_arm.py teach --port COM5 --name home           # once
python robot_arm.py teach --port COM5 --name 1              # ...one per slot, 1-5
python agent.py --id robot --port 9100 --peer <LAPTOP2_IP>:9101 --arm so101 --arm-port COM5
#   Tier 2: add  --cam 1 --roi x,y,w,h   (from python vision.py roi --cam 1)
```
**Laptop 2, human (Ruslan):**
```
python human_leader.py teach --port COM6                    # home, 1-5, tray, reject
python agent.py --id human --port 9101 --peer <LAPTOP1_IP>:9100 --arm leader --arm-port COM6
```
Open **http://localhost:8000** on each laptop and put them side by side. Each page shows what *that* agent believes. Type orders like `1,2,3,4,5` on either one. Use **Reset run** after restocking the slots.

**Fake mode** (no hardware): leave out `--arm`. The robot pretends to pick and the human pretends to push. You can also run both agents on one laptop: give them different `--port` and `--web` values and use `127.0.0.1` as the peer.

## Rules (don't break these, the judges will check)
- **Nothing central**: no single laptop running both arms, no MQTT broker, no cloud AI making decisions
- **Each agent has its own sensors**: overhead camera → robot laptop; leader encoders → human laptop
- **Never move the taped layout after teaching**: the taught picks assume exact slot positions
- **Foam under the follower before pulling its power**: it drops when power is cut

## Timeline
| Time | Milestone |
|---|---|
| 1:45 | Layout taped down (Tanish) |
| 2:15 | Follower replays one pick (Kazy) · both agents talking over the hotspot (Aslan) |
| 2:30 | Leader zones detected live (Ruslan) |
| 3:30 | Full run on real hardware |
| 4:00 | Tier 2 (camera) only if everything above works |
| **4:45** | **Freeze.** Three full runs, film the best one (Ayman) |
| 5:15–6:00 | Rehearse the pitch twice |
