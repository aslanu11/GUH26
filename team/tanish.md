# Tanish: layout, objects, camera
You own: `vision.py`, and `human_hand.py` together with Ruslan. You supply the numbers for `config.json` (send them to Aslan).

## Tasks, in order
1. [ ] Tape down **5 slots and one long tray** within reach of the follower (see the layout picture in Discord). Add a **reject bin** for "damaged" items. Tape an outline around each object so it goes back in the same place
2. [ ] Put a **post-it weight label (1-5)** on top of each object (slot number = label). Objects must fit the follower's gripper
3. [ ] ✅ **1:45: layout taped. Nobody moves it after this.** Kazy teaches the robot's picks against it
4. [ ] Measure in cm: follower base → each slot, and where the human stands → each slot. Send to Aslan for `config.json`
5. [ ] Mount the overhead camera **into Kazy's laptop**, looking straight down so all 5 slots, the tray and the reject bin are in view, with the human's hand visible as it reaches in
6. [ ] With Ruslan: `python human_hand.py regions --cam N` then `watch` (see team/ruslan.md). Tune `FINGERTIP` / box sizes until zones are reliable
7. [ ] Help Ayman with the pitch once the hand tracking works
