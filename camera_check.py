#!/usr/bin/env python3
"""Camera grid viewer — capture all working cameras and display in a grid."""

import cv2
import numpy as np
import threading
import time
import sys

CELL_W, CELL_H = 320, 240
MAX_INDEX = 18  # scan video0..video17


def main():
    # --- Phase 1: probe which cameras open --------------------------------
    print("Probing /dev/video0../dev/video17 ...")
    caps: dict[int, cv2.VideoCapture] = {}
    for i in range(MAX_INDEX):
        cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, CELL_W)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CELL_H)
            ret, frame = cap.read()
            if ret and frame is not None:
                caps[i] = cap
                print(f"  video{i}: OK ({int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
                      f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))})")
            else:
                cap.release()
        else:
            cap.release()

    if not caps:
        print("No working cameras found.")
        sys.exit(1)

    n = len(caps)
    cols = int(np.ceil(np.sqrt(n)))
    rows = int(np.ceil(n / cols))
    indices = sorted(caps.keys())
    print(f"\n{n} cameras alive -> {cols}x{rows} grid.  Press 'q' to quit.\n")

    # --- Phase 2: threaded capture ----------------------------------------
    frames: dict[int, np.ndarray] = {}
    lock = threading.Lock()
    running = True

    def grab(idx: int, cap: cv2.VideoCapture):
        while running:
            ret, frame = cap.read()
            if ret:
                with lock:
                    frames[idx] = frame
            time.sleep(0.03)  # ~30 fps

    for idx, cap in caps.items():
        threading.Thread(target=grab, args=(idx, cap), daemon=True).start()

    # --- Phase 3: display grid --------------------------------------------
    time.sleep(0.3)  # let threads fill first frames
    canvas_w, canvas_h = cols * CELL_W, rows * CELL_H

    while True:
        canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)

        with lock:
            snapshot = dict(frames)

        for pos, idx in enumerate(indices):
            r, c = divmod(pos, cols)
            x, y = c * CELL_W, r * CELL_H

            if idx in snapshot:
                cell = cv2.resize(snapshot[idx], (CELL_W, CELL_H))
            else:
                cell = np.zeros((CELL_H, CELL_W, 3), dtype=np.uint8)

            canvas[y:y + CELL_H, x:x + CELL_W] = cell

            # label
            label = f"video{idx}"
            cv2.putText(canvas, label, (x + 8, y + 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            # border
            cv2.rectangle(canvas, (x, y), (x + CELL_W - 1, y + CELL_H - 1),
                          (80, 80, 80), 1)

        cv2.imshow("Camera Check (press q to quit)", canvas)
        if cv2.waitKey(30) & 0xFF == ord("q"):
            break

    # cleanup
    running = False
    time.sleep(0.1)
    for cap in caps.values():
        cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
