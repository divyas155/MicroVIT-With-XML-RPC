#!/usr/bin/env python3
"""
Test Nano ↔ Orin connectivity (XML-RPC).
Run this ON THE ORIN to verify the Orin can reach the Nano's XML-RPC server.

Usage (on Orin):
  export NANO_IP=10.13.68.184   # or your Nano's IP
  export NANO_PORT=8000
  python3 tools/test_nano_orin_connection.py

Or from repo root on your Mac/laptop (to test a remote Orin's view of Nano):
  NANO_IP=10.13.68.184 NANO_PORT=8000 python3 tools/test_nano_orin_connection.py
"""
import os
import sys
import xmlrpc.client
import http.client

NANO_IP = os.getenv("NANO_IP", "10.13.68.184")
NANO_PORT = int(os.getenv("NANO_PORT", "8000"))
TIMEOUT = 10


class TimeoutTransport(xmlrpc.client.SafeTransport):
    def __init__(self, timeout=10, *a, **k):
        super().__init__(*a, **k)
        self.timeout = timeout
    def make_connection(self, host):
        return http.client.HTTPConnection(host, timeout=self.timeout)


def main():
    url = f"http://{NANO_IP}:{NANO_PORT}"
    print(f"Testing connection to Nano at {url} (timeout {TIMEOUT}s)...")
    try:
        transport = TimeoutTransport(timeout=TIMEOUT)
        proxy = xmlrpc.client.ServerProxy(url, transport=transport)
        status = proxy.get_robot_status()
        if status.get("success"):
            print("✅ SUCCESS: Orin can reach Nano.")
            print(f"   Camera available: {status.get('camera_available', False)}")
            print(f"   Status: {status.get('status_message', '')}")
        else:
            print("⚠️ Nano responded but success=False:", status.get("status_message"))
    except OSError as e:
        print(f"❌ CONNECTION FAILED: {e}")
        print(f"   From this machine run:  ping {NANO_IP}")
        print(f"   Then:  nc -zv {NANO_IP} {NANO_PORT}")
        print("   Ensure Nano bringup is running and listening on 0.0.0.0:8000.")
        sys.exit(1)
    except Exception as e:
        print(f"❌ ERROR: {type(e).__name__}: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
