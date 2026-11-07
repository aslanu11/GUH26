# Ruslan: human agent (real hand)
You own (with Tanish): `human_hand.py`, `regions.json`. You're also **the human in the demo**: you pick with your own hand.

The **overhead camera is plugged into ASLAN's laptop**. Edit `human_hand.py` on your Mac with your AI and push;
run the camera steps on Aslan's laptop (`git pull` first). On a Mac use `python3` / `pip3`.

## Tasks, in order
1. [ ] On Aslan's laptop: `pip install -r requirements.txt` (adds mediapipe + opencv)
2. [ ] `python human_hand.py cams` → find the overhead camera index (N)
3. [ ] **After the layout is taped and the camera mounted**: `python human_hand.py regions --cam N`. Drag one box per zone: 1, 2, 3, 4, 5, tray, reject. Make each slot box cover where your FINGERTIP goes when you grab that object
4. [ ] `python human_hand.py watch --cam N` → a red dot on your index fingertip, and the zone box goes green when you're in it ✅ **2:30**
5. [ ] If zones get missed: redraw the boxes bigger, improve the lighting, keep sleeves out of shot
6. [ ] With Aslan: run the agent with `--arm hand --cam N`
7. [ ] Practise the demo: rest your hand in your assigned slot ~1 s and lift the item into the tray; "steal" the robot's item; put your hand in the robot's slot (robot pauses); walk away (no hand for 12 s)

## How a human pick counts
Fingertip inside a slot box for ≥ 0.6 s (`dwell_s` in config.json) and then leaving. The page also has "I took slot N" buttons as a backup.
Passing over a slot quickly doesn't count. Resting your hand in a slot without taking anything DOES count, so don't do that.
