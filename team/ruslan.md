# Ruslan: human agent (leader arm)
You own: `human_leader.py`, `zones.json`. You're also **the human in the demo**.

**The leader is plugged into ASLAN's laptop** (yours is a Mac). Write and edit `human_leader.py` on your Mac with your AI, push, then run the hardware steps below on Aslan's laptop (`git pull` first). On a Mac, use `python3`.

## Tasks, in order
1. [ ] Plug the leader into Aslan's laptop (it needs its own power supply). `pip install -r requirements.txt`
2. [ ] `python human_leader.py ports` → find the port (unplug/replug to see which one disappears)
3. [ ] **After the layout is taped**: `python human_leader.py teach --port <PORT>`. For each zone (home, 1-5, tray, reject bin), put the leader's handle where you'd use it, then Enter
4. [ ] `python human_leader.py watch --port <PORT>`. Move between zones and check the right name shows. Squeeze the trigger in a slot: you should see `CLICK in 3`. ✅ **2:30**
5. [ ] If zones get confused, raise or lower `THRESHOLD_TICKS`, or re-teach one: `--only 3,tray`
6. [ ] With Aslan: run the agent with `--arm leader --arm-port <PORT>`
7. [ ] Practise the demo: do your assigned pick (push the object into the tray with the leader, squeeze the trigger), "steal" the robot's pick, put the leader into the robot's slot (the robot should pause), walk away (leader still for 12 s)

## How the agent counts a human pick
Trigger squeeze while in a slot zone **or** staying in a slot zone ≥ 0.6 s and then leaving. There's also a manual "I took slot N" button on the page as a backup.
