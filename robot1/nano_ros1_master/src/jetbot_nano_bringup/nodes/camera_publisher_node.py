#!/usr/bin/env python3
"""
Standalone camera node - isolates camera from XML-RPC to prevent crashes.
Publishes JPEG CompressedImage to /nano/camera/image_compressed.
REAL CAMERA ONLY (no dummy). When camera is lost, node stays alive and retries every 30s.
Uses multiple backends (V4L2, GStreamer pipeline) for better compatibility on Jetson.
"""

import glob
import rospy
import cv2
from sensor_msgs.msg import CompressedImage

RETRY_INTERVAL = 30.0  # seconds between camera reconnect attempts

# Lower resolution (320x240) reduces USB bandwidth/power - helps when camera drops
# due to "Cannot enable" / "Maybe the USB cable is bad" on Nano with hub
DEFAULT_WIDTH = 320
DEFAULT_HEIGHT = 240


def _video_devices_exist():
    """Check if any /dev/video* exists (avoids OpenCV spam when none present)."""
    return bool(glob.glob("/dev/video*"))


def _get_video_paths():
    """Return list of /dev/video paths to try, preferring camera_device param."""
    paths = glob.glob("/dev/video*")
    # Sort: video0 first, then video1, etc. Prefer devices that look like cameras (usually video0).
    def sort_key(p):
        try:
            return int(p.replace("/dev/video", ""))
        except ValueError:
            return 999
    return sorted(paths, key=sort_key)


def _open_with_v4l2(device_path, width, height):
    """Try opening with V4L2 backend (avoids GStreamer 'Cannot identify device' on Jetson)."""
    try:
        cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            ret, _ = cap.read()
            if ret:
                return cap
            cap.release()
    except Exception:
        pass
    return None


def _open_with_gstreamer(device_path, width, height):
    """Try opening with explicit GStreamer pipeline (for Jetson with GStreamer-built OpenCV)."""
    gst = (
        f"v4l2src device={device_path} "
        f"! image/jpeg,width={width},height={height},framerate=5/1 "
        "! jpegdec ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1"
    )
    try:
        cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                return cap
            cap.release()
    except Exception:
        pass
    # Fallback: raw format
    gst2 = (
        f"v4l2src device={device_path} "
        f"! video/x-raw,width={width},height={height},framerate=5/1 "
        "! videoconvert ! video/x-raw,format=BGR ! appsink drop=1"
    )
    try:
        cap = cv2.VideoCapture(gst2, cv2.CAP_GSTREAMER)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                return cap
            cap.release()
    except Exception:
        pass
    return None


def _open_with_default(device_path, width, height):
    """Try opening with default backend (index as int)."""
    try:
        idx = int(device_path.replace("/dev/video", ""))
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            ret, _ = cap.read()
            if ret:
                return cap
            cap.release()
    except Exception:
        pass
    return None


def open_camera(camera_device_param, width, height):
    """Open camera using best available method. Returns VideoCapture or None."""
    paths = _get_video_paths()
    # Prefer param device if it exists
    if camera_device_param in paths:
        paths = [camera_device_param] + [p for p in paths if p != camera_device_param]
    for path in paths:
        cap = _open_with_v4l2(path, width, height)
        if cap is not None:
            rospy.loginfo("[Nano Camera] Opened %s via V4L2 (%dx%d)", path, width, height)
            return cap
        cap = _open_with_gstreamer(path, width, height)
        if cap is not None:
            rospy.loginfo("[Nano Camera] Opened %s via GStreamer (%dx%d)", path, width, height)
            return cap
        cap = _open_with_default(path, width, height)
        if cap is not None:
            rospy.loginfo("[Nano Camera] Opened %s via default backend (%dx%d)", path, width, height)
            return cap
    return None


def main():
    rospy.init_node("nano_camera_publisher", anonymous=False)
    camera_device = rospy.get_param("~camera_device", "/dev/video0")
    rate_hz = rospy.get_param("~rate", 5.0)
    width = int(rospy.get_param("~camera_width", DEFAULT_WIDTH))
    height = int(rospy.get_param("~camera_height", DEFAULT_HEIGHT))
    # Publish resolution for XML-RPC (uses same params)
    rospy.set_param("/camera_width", width)
    rospy.set_param("/camera_height", height)

    pub = rospy.Publisher("/nano/camera/image_compressed", CompressedImage, queue_size=1)
    rate = rospy.Rate(rate_hz)

    cap = None
    last_retry = rospy.Time.now()
    seq = 0

    rospy.on_shutdown(lambda: cap.release() if cap else None)

    while not rospy.is_shutdown():
        try:
            if cap is None:
                now = rospy.Time.now()
                if (now - last_retry).to_sec() >= RETRY_INTERVAL:
                    last_retry = now
                    if _video_devices_exist():
                        cap = open_camera(camera_device, width, height)
                        if cap is not None:
                            rospy.loginfo("[Nano Camera] Publishing to /nano/camera/image_compressed at %.1f Hz", rate_hz)
                    else:
                        rospy.loginfo_throttle(30, "[Nano Camera] No /dev/video* found. Retrying every %gs. Plug in camera.", RETRY_INTERVAL)
                if cap is None:
                    rate.sleep()
                    continue

            ret, frame = cap.read()
            if not ret or frame is None:
                rospy.logwarn_throttle(5, "[Nano Camera] read() failed, camera lost. Will retry.")
                try:
                    cap.release()
                except Exception:
                    pass
                cap = None
                rate.sleep()
                continue

            _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.header.seq = seq
            msg.header.frame_id = "camera"
            msg.format = "jpeg"
            msg.data = jpeg.tobytes()
            pub.publish(msg)
            seq += 1
        except Exception as e:
            rospy.logerr_throttle(5, "[Nano Camera] Error: %s", e)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
