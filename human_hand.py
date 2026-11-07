"""
human_hand.py - OWNERS: Ruslan + Tanish. The human agent's body: a REAL human hand, tracked by the
overhead camera (plugged into Aslan's laptop).

MediaPipe finds the hand; we check which taught box (slot 1-5, tray, reject) the hand is in.
A hand resting in a slot box >= dwell_s (config.json) and then leaving = the human took that item.

    pip install mediapipe opencv-python
    python human_hand.py cams                    # find the overhead camera index
    python human_hand.py regions --cam 1         # drag a box over each zone in order (see prompt), Enter after each
    python human_hand.py watch --cam 1           # live window: hand dot + which zone it's in. Check it works!

The model file (hand_landmarker.task, ~8 MB) downloads automatically the first time.

The interface the agent uses (same as human_leader.py, do not change without telling Aslan):
    HandTracker(cam_index, regions_path).start()
    .zone            -> "1".."5" | "tray" | "reject" | None (no hand / hand not in a box)
    .last_motion_t   -> time.time() when a hand was last seen moving ('walked away' = no hand for away_s)
"""
import argparse
import json
import threading
import time
import urllib.request
from pathlib import Path

MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
             "hand_landmarker/float16/1/hand_landmarker.task")
MODEL_PATH = Path(__file__).with_name("hand_landmarker.task")
ZONES = ["1", "2", "3", "4", "5", "tray", "reject"]
FINGERTIP = 8        # index fingertip landmark: where the hand is "pointing/reaching"
MOTION_PX = 6        # fingertip movement below this (pixels) is jitter


def ensure_model():
    if not MODEL_PATH.exists():
        print(f"downloading hand model to {MODEL_PATH} ...")
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
    return str(MODEL_PATH)


def make_landmarker():
    import mediapipe as mp
    from mediapipe.tasks.python.core.base_options import BaseOptions
    from mediapipe.tasks.python.vision.core.vision_task_running_mode import VisionTaskRunningMode
    from mediapipe.tasks.python.vision.hand_landmarker import HandLandmarker, HandLandmarkerOptions
    opts = HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=ensure_model()),
        running_mode=VisionTaskRunningMode.VIDEO,
        num_hands=2,
        min_hand_detection_confidence=0.5,
    )
    return mp, HandLandmarker.create_from_options(opts)


def zone_of(px, py, regions):
    for name in ZONES:
        if name in regions:
            x, y, w, h = regions[name]
            if x <= px <= x + w and y <= py <= y + h:
                return name
    return None


class HandTracker(threading.Thread):
    def __init__(self, cam_index, regions_path="regions.json", show=False):
        super().__init__(daemon=True)
        self.cam_index = cam_index
        self.regions = json.loads(Path(regions_path).read_text())
        self.show = show
        self.zone = None
        self.hands = []            # list of (x, y) fingertip pixels
        self.last_motion_t = time.time()
        self.ok = False
        self._last_tip = None
        self.jpeg = None           # latest annotated frame, served by the agent at /video
        self.robot_target = None   # set by the agent: the slot the robot is working in
        self.danger = False        # set by the agent: hand is in the robot's slot

    def run(self):
        import cv2
        mp, lm = make_landmarker()
        cap = cv2.VideoCapture(self.cam_index)
        t0 = time.time()
        while True:
            ok, frame = cap.read()
            if not ok:
                self.ok = False
                time.sleep(0.2)
                continue
            self.ok = True
            h, w = frame.shape[:2]
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            res = lm.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), int((time.time() - t0) * 1000))
            tips = [(int(hand[FINGERTIP].x * w), int(hand[FINGERTIP].y * h)) for hand in (res.hand_landmarks or [])]
            self.hands = tips
            # the zone = first hand found inside a box
            z = None
            for tx, ty in tips:
                z = zone_of(tx, ty, self.regions)
                if z:
                    break
            self.zone = z
            if tips:
                tip = tips[0]
                if self._last_tip is None or abs(tip[0] - self._last_tip[0]) + abs(tip[1] - self._last_tip[1]) > MOTION_PX:
                    self.last_motion_t = time.time()
                self._last_tip = tip
            draw(cv2, frame, self.regions, tips, z, self.robot_target, self.danger)
            ok_enc, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok_enc:
                self.jpeg = buf.tobytes()
            if self.show:
                cv2.imshow("hand tracker (q to quit)", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break


def draw(cv2, frame, regions, tips, active, robot_target=None, danger=False):
    for name, (x, y, w, h) in regions.items():
        col, label, thick = (200, 200, 200), name, 2
        if robot_target is not None and name == str(robot_target):
            col, label, thick = (0, 140, 255), f"{name} ROBOT", 3          # orange: robot working here
        if name == active:
            col = (0, 0, 255) if danger else (0, 200, 0)                   # red if it's the robot's slot
            thick = 4 if danger else thick
        cv2.rectangle(frame, (x, y), (x + w, y + h), col, thick)
        cv2.putText(frame, label, (x + 4, y + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
    for tx, ty in tips:
        cv2.circle(frame, (tx, ty), 8, (0, 0, 255), -1)
    if danger:
        cv2.rectangle(frame, (0, 0), (frame.shape[1] - 1, frame.shape[0] - 1), (0, 0, 255), 8)
        cv2.putText(frame, "MOVE YOUR HAND - ROBOT WORKING", (12, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)


def teach_regions(cam, out):
    import cv2
    cap = cv2.VideoCapture(cam)
    for _ in range(10):  # let auto-exposure settle
        ok, frame = cap.read()
    if not ok:
        raise SystemExit(f"camera {cam} gave no image")
    regions = {}
    for name in ZONES:
        print(f"Drag a box over zone '{name}', then press Enter (or c to skip)")
        x, y, w, h = cv2.selectROI(f"zone {name}: drag box, Enter", frame, showCrosshair=False)
        cv2.destroyAllWindows()
        if w and h:
            regions[name] = [int(x), int(y), int(w), int(h)]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (200, 200, 200), 2)
            cv2.putText(frame, name, (x + 4, y + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
    Path(out).write_text(json.dumps(regions, indent=1))
    print(f"saved {out}: {list(regions)}. Now run: python human_hand.py watch --cam {cam}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["cams", "regions", "watch"])
    ap.add_argument("--cam", type=int, default=0)
    ap.add_argument("--regions", default="regions.json")
    a = ap.parse_args()
    if a.cmd == "cams":
        import cv2
        for i in range(6):
            cap = cv2.VideoCapture(i)
            ok, f = cap.read()
            print(i, "OK" if ok else "-", f.shape if ok else "")
            cap.release()
    elif a.cmd == "regions":
        teach_regions(a.cam, a.regions)
    else:
        t = HandTracker(a.cam, a.regions, show=True)
        t.run()  # runs in the foreground with a window


if __name__ == "__main__":
    main()
