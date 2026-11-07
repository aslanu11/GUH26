# Kazy: robot arm (laptop 1)
You own: `robot_arm.py`, `motions/`. Your laptop runs the ROBOT agent (follower + its wrist camera). The overhead camera goes to Aslan's laptop.

## Tasks, in order
1. [ ] Note the follower's USB port, then **close MakerMods Lab** (it holds the port). `pip install -r requirements.txt`
2. [ ] `python robot_arm.py ports`, then `python robot_arm.py read --port <PORT>`. Move the arm by hand and check the numbers change
3. [ ] **Wait for Tanish to tape the layout down.** Then teach HOME: `python robot_arm.py teach --port <PORT> --name home` (arm folded, clear of everything, one waypoint, `s`)
4. [ ] Teach slot 1: `--name 1`. Recipe: above the object (Enter) → lower (Enter) → close the gripper by hand on the object (`g`) → lift (Enter) → over the ROBOT's end of the tray (Enter) → open the gripper by hand (Enter) → lift (Enter) → `p` to preview → `s`
5. [ ] `python robot_arm.py play --port <PORT> --name 1`. It should pick and place reliably 3 times in a row. ✅ **2:15**
6. [ ] Teach slots 2-5 the same way ✅ **3:30**
7. [ ] With Aslan: run the agent with `--arm so101 --arm-port <PORT>`

## Tuning knobs (top of robot_arm.py)
`SPEED_TICKS_S` (slower = safer), `GRASP_SQUEEZE` (raise it if objects slip, lower it if the gripper strains), `GRASP_DWELL_S`.

## If things go wrong
- "cannot open serial port" → MakerMods Lab or another script still has the port open
- Arm shakes → lower `SPEED_TICKS_S`. If the objects move between runs, re-teach that slot only
- Tier 2 (only if ahead): in `FollowerReplay.execute`, use the wrist camera to return `"empty"` if the slot is empty
