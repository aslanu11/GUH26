"""
vision.py - OWNER: Tanish. Tier 2: the robot agent's eyes when comms are down.

The overhead camera is plugged into the ROBOT laptop. When the human agent goes silent, the robot
asks: is the leader arm still moving? Moving = human alive but cut off (stay out of their way).
Still = human gone (take over their picks). Timeouts guess; we look.

    pip install opencv-python
    python vision.py list                       # which camera index is the overhead one
    python vision.py roi --cam 1                # drag a box around the LEADER arm's area, prints x,y,w,h
    python vision.py watch --cam 1 --roi 100,80,300,260   # live motion score, tune THRESH

Then start the robot agent with:  --cam 1 --roi 100,80,300,260

The interface the agent uses (do not change without telling Aslan):
    MotionWatcher(cam_index, roi).start();  .moving() -> True | False | None (None = no camera)
"""
import argparse
import threading
import time

THRESH = 6.0        # mean pixel change inside the box that counts as movement
WINDOW_S = 1.5      # "moving" = movement seen within this window


class MotionWatcher(threading.Thread):
    def __init__(self, cam_index, roi=None, thresh=THRESH):
        super().__init__(daemon=True)
        self.cam_index, self.roi, self.thresh = cam_index, roi, thresh
        self.ok = False
        self.score = 0.0
        self.last_motion_t = 0.0

    def moving(self):
        if not self.ok:
            return None
        return (time.time() - self.last_motion_t) < WINDOW_S

    def run(self):
        import cv2
        cap = cv2.VideoCapture(self.cam_index)
        prev = None
        while True:
            ok, frame = cap.read()
            if not ok:
                self.ok = False
                time.sleep(0.5)
                continue
            self.ok = True
            if self.roi:
                x, y, w, h = self.roi
                frame = frame[y:y + h, x:x + w]
            g = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (9, 9), 0)
            if prev is not None:
                self.score = float(cv2.absdiff(g, prev).mean())
                if self.score > self.thresh:
                    self.last_motion_t = time.time()
            prev = g
            time.sleep(0.05)


def main():
    import cv2
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["list", "roi", "watch"])
    ap.add_argument("--cam", type=int, default=0)
    ap.add_argument("--roi")
    a = ap.parse_args()
    if a.cmd == "list":
        for i in range(6):
            cap = cv2.VideoCapture(i)
            ok, f = cap.read()
            print(i, "OK" if ok else "-", f.shape if ok else "")
            cap.release()
    elif a.cmd == "roi":
        cap = cv2.VideoCapture(a.cam)
        ok, f = cap.read()
        x, y, w, h = cv2.selectROI("drag a box around the leader arm, then Enter", f)
        print(f"--roi {x},{y},{w},{h}")
    else:
        roi = tuple(int(v) for v in a.roi.split(",")) if a.roi else None
        m = MotionWatcher(a.cam, roi)
        m.start()
        while True:
            time.sleep(0.2)
            print(f"score={m.score:5.1f}  moving={m.moving()}")


if __name__ == "__main__":
    main()
