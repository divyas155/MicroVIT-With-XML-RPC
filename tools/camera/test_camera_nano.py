#!/usr/bin/env python3
"""
Run on the Nano (no ROS) to see which /dev/video* opens with OpenCV and why others fail.
Usage: python3 test_camera_nano.py
"""

import sys
import os

def main():
    print("=== Camera devices ===")
    for d in sorted(os.listdir("/dev") or []):
        if d.startswith("video"):
            path = "/dev/" + d
            try:
                st = os.stat(path)
                mode = oct(st.st_mode)[-3:]
                print(f"  {path}  mode={mode}")
            except Exception as e:
                print(f"  {path}  stat error: {e}")
    print()

    try:
        import cv2
    except ImportError:
        print("OpenCV not installed. Install: sudo apt install python3-opencv")
        sys.exit(1)

    # Try each index with V4L2 then default backend; try with and without MJPEG/640x480
    for idx in range(6):
        for backend_name, backend in [("V4L2", cv2.CAP_V4L2), ("default", None)]:
            cap = None
            try:
                if backend is not None:
                    cap = cv2.VideoCapture(idx, backend)
                else:
                    cap = cv2.VideoCapture(idx)
                if not cap.isOpened():
                    print(f"  video{idx} {backend_name}: open failed (isOpened=False)")
                    if cap:
                        cap.release()
                    continue
                # Try setting MJPEG 640x480 like the real node
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                ret, frame = cap.read()
                if ret and frame is not None:
                    print(f"  video{idx} {backend_name}: OK (frame shape {frame.shape}) -> use camera_device:=/dev/video{idx}")
                else:
                    print(f"  video{idx} {backend_name}: opened but read() failed (ret={ret})")
                cap.release()
            except Exception as e:
                print(f"  video{idx} {backend_name}: exception: {e}")
            finally:
                if cap is not None:
                    try:
                        cap.release()
                    except Exception:
                        pass
    print("\nIf one line shows OK, use that device in launch: camera_device:=/dev/videoN")
    print("If all fail: check permissions (sudo chmod 666 /dev/video0) and that no other app uses the camera.")

if __name__ == "__main__":
    main()
