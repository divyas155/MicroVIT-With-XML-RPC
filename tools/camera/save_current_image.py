#!/usr/bin/env python3
"""
Save current camera image from Nano to the project.
Requires: Nano bringup running, XML-RPC on port 8000.
Usage: python save_current_image.py [--output path/to/image.jpg]
"""

import argparse
import base64
import os
import sys
import time
import xmlrpc.client
from datetime import datetime

# Default Nano IP (set NANO_IP env to override)
NANO_IP = os.environ.get("NANO_IP", "10.13.68.184")
NANO_PORT = 8000
RETRY_INTERVAL = 2.0  # seconds between attempts
RETRY_TIMEOUT = 35.0   # total wait (covers camera node's 30s retry)


def main():
    parser = argparse.ArgumentParser(description="Save current Nano camera image to project")
    parser.add_argument("--output", "-o", help="Output path (default: captured_images/camera_YYYY-MM-DD_HH-MM-SS.jpg)")
    parser.add_argument("--nano-ip", default=NANO_IP, help=f"Nano IP (default: {NANO_IP})")
    parser.add_argument("--nano-port", type=int, default=NANO_PORT, help="Nano XML-RPC port")
    parser.add_argument("--no-retry", action="store_true", help="Do not retry when camera is not ready")
    args = parser.parse_args()

    url = f"http://{args.nano_ip}:{args.nano_port}"
    try:
        proxy = xmlrpc.client.ServerProxy(url, allow_none=True)
    except Exception as e:
        print(f"❌ Failed to connect to Nano at {url}: {e}")
        print("   Ensure Nano bringup is running and port 8000 is open.")
        sys.exit(1)

    response = None
    start = time.time()
    while True:
        try:
            response = proxy.get_camera_image()
        except Exception as e:
            print(f"❌ RPC error: {e}")
            sys.exit(1)
        if response.get("success") and response.get("image_data"):
            break
        elapsed = time.time() - start
        if args.no_retry or elapsed >= RETRY_TIMEOUT:
            print(f"❌ Camera not available: {response.get('message', 'Unknown error')}")
            if not args.no_retry:
                print("   (Camera node retries every 30s; run again or check Nano.)")
            sys.exit(1)
        print(f"   Waiting for camera... ({elapsed:.0f}s)")
        time.sleep(RETRY_INTERVAL)

    image_b64 = response.get("image_data", "")
    if not image_b64:
        print("❌ No image data in response")
        sys.exit(1)

    # Output path
    if args.output:
        out_path = args.output
    else:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.abspath(os.path.join(script_dir, "..", ".."))
        captured_dir = os.path.join(project_root, "captured_images")
        os.makedirs(captured_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        out_path = os.path.join(captured_dir, f"camera_{ts}.jpg")

    try:
        image_bytes = base64.b64decode(image_b64)
        with open(out_path, "wb") as f:
            f.write(image_bytes)
        print(f"✅ Saved image to {out_path} ({len(image_bytes)} bytes)")
    except Exception as e:
        print(f"❌ Failed to save: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
