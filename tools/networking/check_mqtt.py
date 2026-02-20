#!/usr/bin/env python3
"""
Check MQTT broker connectivity and topic subscriptions
"""

import paho.mqtt.client as mqtt
import sys
import time
import json

def on_connect(client, userdata, flags, rc):
    """Callback for MQTT connection"""
    if rc == 0:
        print("✅ Connected to MQTT broker")
        print(f"   Flags: {flags}")
    else:
        print(f"❌ Connection failed with code {rc}")
        sys.exit(1)

def on_message(client, userdata, msg):
    """Callback for received messages"""
    print(f"\n📨 Message received:")
    print(f"   Topic: {msg.topic}")
    try:
        payload = json.loads(msg.payload.decode())
        print(f"   Payload: {json.dumps(payload, indent=2)}")
    except:
        print(f"   Payload: {msg.payload.decode()[:100]}...")

def check_mqtt(broker_host="localhost", broker_port=1883, username="", password=""):
    """Check MQTT broker connectivity"""
    print("=" * 50)
    print("MQTT Broker Connectivity Check")
    print("=" * 50)
    print(f"Broker: {broker_host}:{broker_port}")
    print("")
    
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    
    if username and password:
        client.username_pw_set(username, password)
    
    try:
        print("Connecting...")
        client.connect(broker_host, broker_port, 60)
        client.loop_start()
        
        time.sleep(1)
        
        # Subscribe to test topics
        test_topics = [
            "robots/+/telemetry",
            "robots/+/alerts",
            "robots/+/status",
            "controller/analysis",
            "helper/status"
        ]
        
        print("\nSubscribing to topics:")
        for topic in test_topics:
            client.subscribe(topic, qos=1)
            print(f"   ✅ {topic}")
        
        print("\nListening for messages (10 seconds)...")
        print("(Publish test messages to verify connectivity)")
        time.sleep(10)
        
        client.loop_stop()
        client.disconnect()
        
        print("\n✅ MQTT check complete")
        
    except Exception as e:
        print(f"\n❌ MQTT error: {e}")
        print("   Check:")
        print("   - MQTT broker is running")
        print("   - Network connectivity")
        print("   - Firewall settings")
        sys.exit(1)

if __name__ == "__main__":
    broker_host = sys.argv[1] if len(sys.argv) > 1 else "localhost"
    broker_port = int(sys.argv[2]) if len(sys.argv) > 2 else 1883
    
    check_mqtt(broker_host, broker_port)
