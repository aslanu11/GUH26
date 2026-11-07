# Demo script (draft, Ayman owns this, edit freely)

**Setup:** two laptops side by side showing each agent's page. Ruslan at the table (his hand is tracked by the overhead camera), Kazy next to the follower (hand near its power plug, foam underneath).

1. **Intro (20 s):** "Two independent agents, one robot and one human, run a pick station. No central controller: each one decides for itself from its own view."
2. **Normal order:** type `1,2,3,4,5`. Point out heavy items (4-5) going to the robot and the light/near ones to the human.
3. **Human steals a pick / damaged package:** Ruslan bins the robot's item → the robot's page logs "taken by human, replanning".
4. **Safety pause:** Ruslan puts his hand in the robot's slot → the robot pauses → hand leaves → it resumes. "Speed-and-separation monitoring, ISO/TS 15066."
5. **Pull the robot's power:** human gets the light picks; heavy ones show "blocked: needs a robot". "Even when something fails, the system never hands a heavy item to a person."
6. **Cut comms:** press Cut comms. Both keep working their own picks. Restore → the boards merge, nothing gets done twice.

## Likely judge questions
- *Where's the central server?* There isn't one. Each laptop runs its own agent; state merges peer-to-peer.
- *What if both pick the same item while cut off?* The weight and distance rule splits picks ahead of time; on reconnect the merge keeps one owner; an item can physically only be picked once.
- *Crashed vs cut off?* Heartbeat timeout plus (Tier 2) the overhead camera: a moving arm is alive.
- *Why a human?* Robots take heavy and repetitive picks; humans take exceptions (damage, odd shapes) without expensive AI vision.
- *Does it scale?* The same agent code runs N robots and M humans: add them to config.json.
