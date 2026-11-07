# Tanish: layout, objects, camera
You own: `vision.py`. You supply the numbers for `config.json` (send them to Aslan).

## Tasks, in order
1. [ ] Tape down **5 slots and one long tray** within reach of BOTH arms. The robot drops at its end of the tray and the human at theirs. Add a **reject bin** for "damaged" items
2. [ ] Put a **post-it weight label (1-5)** on each object. Objects must fit the follower's gripper. Make the slot number match the label (slot 4 holds the weight-4 object) unless Aslan changes config
3. [ ] ✅ **1:45: layout taped. Nobody moves it after this.** Kazy and Ruslan teach positions against it
4. [ ] Measure in cm: follower base → each slot, and leader base → each slot. Send them to Aslan for `config.json` (`"dist": {"robot": .., "human": ..}`)
5. [ ] Mount the overhead camera (into Kazy's laptop) so it sees the slots, the tray and the leader arm
6. [ ] Tier 2: `python vision.py list` → `python vision.py roi --cam N` (draw a box around the leader arm's area) → `python vision.py watch --cam N --roi ...` → tune `THRESH` so moving the leader shows `moving=True` and still shows `False`
7. [ ] Help Ayman with the pitch and slides once the camera works
