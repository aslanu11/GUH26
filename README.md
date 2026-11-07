# Neptune North Challenge: Human + Robot Fulfilment Cell

**GreatUniHack 2026 · Team: Aslan, Kazy, Ruslan, Tanish, Ayman**

A mini Amazon pick station run by **two independent agents with no central controller**:

- **Robot agent**: the SO-101 follower picks objects with its gripper, fully autonomously (taught picks replayed when the agent decides). It has its wrist camera.
- **Human agent**: a real person (Ruslan in the demo) picks with **their own hand**. The **overhead camera** (plugged into Aslan's laptop) tracks the hand with MediaPipe and knows which slot it's in. A hand resting in a slot, then leaving, counts as a pick. The leader arm isn't used (`human_leader.py` stays as a fallback).

Each slot holds one parcel (a paper ball). **Every new order draws a random weight from 1 to 10 for each parcel** (stored in the order, like a manifest, so every agent sees the same weights). The robot takes the heaviest, heaviest first; the human gets the lightest, lightest first. Both agents run the **same rule** (distance + weight + queue) on their own copy of the order board, so they agree who picks what without negotiating. **Weights above 6 are robot-only** (`human_max_weight` in config.json), for human safety.

## Demo moments (what the judges see)
| Event | What happens |
|---|---|
| Normal order | Picks split between robot and human; heavy ones always go to the robot |
| Human takes the robot's item (e.g. spots a damaged package and bins it) | Robot sees it in the shared state, drops that pick and replans |
| Human's hand enters the robot's slot | Robot **pauses** and resumes when the arm leaves (speed-and-separation monitoring, ISO/TS 15066) |
| Human walks away | No hand seen for 12 s → human agent marks itself away → robot takes the light picks |
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
| `human_hand.py`, `regions.json` | **Ruslan + Tanish** | Real-hand tracking with the overhead camera (MediaPipe) |
| `human_leader.py` | Ruslan | Fallback: leader arm as a tracked tool (not used by default) |
| `vision.py` | **Tanish** | Tier 2 motion watcher (crashed vs cut off) |
| `test_scenarios.py`, `docs/` | **Ayman** | Automated scenario tests, demo script, pitch |
| `team/<name>.md` | each person | Your task list. Your AI reads this first |

The **interfaces between files** are written at the top of each file. Don't change an interface without telling its owner in Discord.

## Which laptop does what (ONE laptop for the demo)
**Aslan's laptop (Windows) runs both agents** as two separate processes in two terminals. Mac users (Ruslan, Ayman, Kazy): use `python3`/`pip3`, ports look like `/dev/tty.usbmodemXXXX`.
The agents still share nothing but UDP messages on `127.0.0.1`: no shared memory, no shared files, no broker. Each one decides for itself.

| Laptop | Use |
|---|---|
| **Aslan's** (Windows) | Robot agent + human agent, follower arm + overhead camera |
| Tanish's (Windows) | Backup demo laptop, fake-mode testing |
| Kazy's (MacBook) | Teaching help, fake-mode tests |
| Ruslan's, Ayman's (MacBooks) | Editing, fake-mode tests, pitch |

**Pitch framing:** "Each agent is an independent process with its own sensors and state. For the demo they share a laptop; in a warehouse each runs on its own device. The code is identical: only the peer IP changes."

## Setup (every laptop). Windows PowerShell: run ONE line at a time, no `&&`
```
git clone https://github.com/aslanu11/GUH26.git
cd GUH26
pip install -r requirements.txt
python test_scenarios.py
```
`test_scenarios.py` runs a fake robot + fake human and checks every demo moment, no hardware. It must print `ALL PASS`.

## Run for real (Aslan's laptop, two terminals)
**Close MakerMods Lab first**: only one program can use the arm's port. Run `pip install -r requirements.txt` once (adds mediapipe).

One-time setup:
```
python robot_arm.py ports                                   # follower port, e.g. COM5
python robot_arm.py teach --port COM5 --name home           # once
python robot_arm.py teach --port COM5 --name 1              # ...one per slot, 1-5
python human_hand.py cams                                   # overhead camera index on THIS laptop
python human_hand.py regions --cam 1                        # zone boxes (or git pull Aslan's regions.json if the camera hasn't moved)
python human_hand.py watch --cam 1                          # check: red dot on fingertip, box turns green
```
**Terminal 1, robot agent:**
```
python agent.py --id robot --port 9100 --peer 127.0.0.1:9101 --web 8000 --arm so101 --arm-port COM7 --view http://127.0.0.1:8001/video
```
**Terminal 2, human agent:**
```
python agent.py --id human --port 9101 --peer 127.0.0.1:9100 --web 8001 --arm hand --cam 1
```
Both pages show the overhead camera (zone boxes, fingertip, the robot's current slot in orange). If the human's hand is in the robot's slot, the human page shows a red **MOVE YOUR HAND** banner and beeps (click the page once to enable sound), and the robot pauses.
Robot page: **http://localhost:8000** · human page: **http://localhost:8001**. Put them side by side. Each shows what *that* agent believes.
Type orders like `1,2,3,4,5` on either one. Use **Reset run** after restocking the slots.

**LLM brains (optional):** set your Claude key, then add `--llm` to the robot AND/OR the human command (two Claude agents that coordinate by message):
```
$env:ANTHROPIC_API_KEY = "sk-ant-..."
python agent.py --id robot ... --llm
```
Claude decides the robot's next action among the options the shared rule allows (it can't break the weight limit or the hand pause), explains why on the robot page, and can talk to the human (blue banner on the human page). If Claude is slow or unreachable, the robot instantly falls back to its reflex rule: the LLM link is just another comms link. Never commit the key.

**Kill an agent in the demo:** close Terminal 1 (the robot's brain dies). Within about 6 s the human page shows the robot **down** and takes the light picks; heavy ones show "blocked: needs a robot". Re-run the Terminal 1 command and it rejoins and merges.

**Fake mode** (no hardware): leave out `--arm`. Leave out `--arm so101 --arm-port` for a pretend robot, or `--arm hand --cam` for a pretend human.
**Two-laptop mode** (optional): same commands, with the other laptop's IP as `--peer` instead of `127.0.0.1`.

## Rules (don't break these, the judges will check)
- **Nothing central**: agents only talk via UDP messages: no shared memory or files, no MQTT broker, no cloud AI (no Dream-1 or LLM APIs) making decisions
- **Each agent has its own sensors**: overhead camera → human agent only (tracks the hand); arm encoders + wrist camera → robot agent only
- **Never move the taped layout after teaching**: the taught picks assume exact slot positions
- **Foam under the follower before pulling its power**: it drops when power is cut

## Timeline
| Time | Milestone |
|---|---|
| 1:45 | Layout taped down (Tanish) |
| 2:15 | Follower replays one pick (Kazy) · both agents talking on Aslan's laptop |
| 2:30 | Hand zones working on Aslan's laptop (done) |
| 3:30 | Full run on real hardware |
| 4:00 | Tier 2 (camera) only if everything above works |
| **4:45** | **Freeze.** Three full runs, film the best one (Ayman) |
| 5:15–6:00 | Rehearse the pitch twice |
