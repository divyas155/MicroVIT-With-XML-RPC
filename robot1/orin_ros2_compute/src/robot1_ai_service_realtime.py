#!/usr/bin/env python3
"""
REALTIME VERSION - Robot1 AI Service for Jetson Orin

Uses REAL camera feed (from Nano via XML-RPC) and REAL LiDAR data for AI message generation.
Optimized for 8GB RAM systems with local Ollama AI.

USAGE:
  1. Copy this file to Orin: ~/robot1_ai_package/
  2. Ensure Nano is running xmlrpc_server_nano_realtime.py
  3. Set environment variables:
     export NANO_IP=10.13.68.184
     export NANO_PORT=8000
  4. Run: python3 robot1_ai_service_realtime.py --simulation
"""

import json
import time
import base64
import io
import random
import uuid
import requests
import cv2
import numpy as np
from datetime import datetime
from typing import Dict, Any, Optional
import paho.mqtt.client as mqtt
import logging
import os
from dotenv import load_dotenv
from PIL import Image, ImageDraw
import xmlrpc.client
import http.client
import sys
import math


class TimeoutXMLRPCTransport(xmlrpc.client.SafeTransport):
    """XML-RPC transport with connection timeout so Orin does not hang if Nano is unreachable."""
    def __init__(self, timeout=10, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.timeout = timeout

    def make_connection(self, host):
        return http.client.HTTPConnection(host, timeout=self.timeout)

# Try importing VILA integration
try:
    from vila_integration import VILAModel
    VILA_AVAILABLE = True
except ImportError:
    VILA_AVAILABLE = False

# Try importing VIT integration
try:
    from vit_integration import VITModel
    VIT_AVAILABLE = True
except ImportError:
    VIT_AVAILABLE = False

# Try importing MicroViT integration
import sys
import os
# Add common utils to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../../common/utils'))
try:
    from microvit_integration import MicroViTModel
    MICROVIT_AVAILABLE = True
except ImportError:
    MICROVIT_AVAILABLE = False

# Load environment variables (cwd, package dir, then config/ so NANO_IP etc. are found)
load_dotenv()
_pkg_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
load_dotenv(os.path.join(_pkg_dir, '.env'))
load_dotenv(os.path.join(_pkg_dir, 'config', '.env'))


def _nano_ip_from_config():
    """Read Nano IP from config/device_config.yaml so repo config overrides wrong NANO_IP in env."""
    import re
    path = os.path.join(_pkg_dir, 'config', 'device_config.yaml')
    if not os.path.isfile(path):
        return None
    try:
        with open(path, 'r') as f:
            content = f.read()
        m = re.search(r'nano_host:\s*["\']([^"\']+)["\']', content)
        if m:
            return m.group(1).strip()
    except Exception:
        pass
    return None

# Setup logging
_log_dir = os.getenv('LOG_DIR', './logs')
_handlers = [logging.StreamHandler()]
# Avoid crashing if ./logs doesn't exist (common on fresh setups / laptops).
if os.path.isdir(_log_dir):
    _handlers.insert(0, logging.FileHandler(os.path.join(_log_dir, 'robot1_ai_realtime.log')))
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=_handlers
)
logger = logging.getLogger(__name__)


class Robot1AIServiceRealtime:
    """
    Robot1 AI Service - REALTIME VERSION
    
    Uses REAL camera from Nano via XML-RPC and REAL LiDAR data.
    """
    
    def __init__(self):
        self.robot_id = os.getenv('ROBOT_ID', 'jetson1')
        self.ollama_host = os.getenv('OLLAMA_HOST', 'http://localhost:11434')
        self.ollama_model = os.getenv('OLLAMA_MODEL', 'phi3:mini')
        self.mqtt_broker_host = os.getenv('MQTT_BROKER_HOST', 'localhost')
        self.mqtt_broker_port = int(os.getenv('MQTT_BROKER_PORT', 1883))
        # Prefer device_config.yaml nano_host so correct Nano IP is used even if NANO_IP env is wrong
        _cfg_ip = _nano_ip_from_config()
        self.nano_ip = _cfg_ip or os.getenv('NANO_IP', '10.13.68.184')
        _cfg_port = None
        if os.path.isfile(os.path.join(_pkg_dir, 'config', 'device_config.yaml')):
            try:
                with open(os.path.join(_pkg_dir, 'config', 'device_config.yaml'), 'r') as f:
                    for line in f:
                        if 'nano_port:' in line:
                            _cfg_port = int(line.split(':', 1)[1].strip())
                            break
            except Exception:
                pass
        self.nano_port = _cfg_port if _cfg_port is not None else int(os.getenv('NANO_PORT', '8000'))
        self.mqtt_client = None
        self.detection_count = 0
        self.nano_xmlrpc = None
        # Keep last known odometry so we can fall back if Nano odom disappears mid-run.
        self._last_odom = {"x": 0.0, "y": 0.0, "z": 0.0, "theta": 0.0, "linear_v": 0.0, "angular_v": 0.0, "source": "FALLBACK"}
        # Keep last known LiDAR so we can fall back if Nano /scan disappears mid-run.
        self._last_lidar = {
            "nearest_distance": 0.5,
            "obstacle_direction": 0.0,
            "obstacle_size": 0.5,
            "confidence": 0.6,
            "timestamp": datetime.now().isoformat(),
            "sensor_type": "LiDAR (FALLBACK DUMMY)",
            "source": "FALLBACK"
        }
        
        # Check if using MicroViT (highest priority - fastest preprocessing)
        self.use_microvit = MICROVIT_AVAILABLE and os.getenv('USE_MICROVIT', 'false').lower() == 'true'
        self.microvit_model = None
        
        # Check if using VIT
        self.use_vit = VIT_AVAILABLE and ('vit' in self.ollama_model.lower() or 'blip' in self.ollama_model.lower())
        self.vit_model = None
        
        # Set text model based on vision model choice
        if self.use_microvit or self.use_vit:
            self.ollama_text_model = os.getenv('OLLAMA_TEXT_MODEL', 'phi3:mini')
            logger.info(f"📝 Vision model will use '{self.ollama_text_model}' for text generation")
        else:
            self.ollama_text_model = self.ollama_model
        
        # Initialize MicroViT (highest priority)
        if self.use_microvit:
            logger.info("🚀 Using MicroViT model for fast image preprocessing")
            try:
                use_cpu = os.getenv('MICROVIT_USE_CPU', 'false').lower() == 'true'
                model_name = os.getenv('MICROVIT_MODEL_NAME', 'apple/mobilevit-small')
                variant = os.getenv('MICROVIT_VARIANT', 'S1')
                self.microvit_model = MicroViTModel(model_name=model_name, use_cpu=use_cpu, variant=variant)
                self.microvit_model.load_model()
                logger.info("✅ MicroViT model loaded successfully!")
                logger.info(f"✅ Will use Ollama model '{self.ollama_text_model}' for text generation")
                logger.info("⚡ MicroViT provides ~9ms preprocessing (vs ~500ms for BLIP)")
            except Exception as e:
                logger.error(f"❌ Failed to load MicroViT: {e}")
                self.use_microvit = False
        
        # Initialize VIT/BLIP (fallback)
        if not self.use_microvit and self.use_vit:
            logger.info("🚀 Using VIT model for image analysis")
            try:
                use_cpu = os.getenv('VIT_USE_CPU', 'false').lower() == 'true'
                model_name = os.getenv('VIT_MODEL_NAME', 'Salesforce/blip-image-captioning-base')
                self.vit_model = VITModel(model_name=model_name, use_cpu=use_cpu)
                self.vit_model.load_model()
                logger.info("✅ VIT model loaded successfully!")
            except Exception as e:
                logger.error(f"❌ Failed to load VIT: {e}")
                self.use_vit = False
        
        # Initialize components
        self.setup_xmlrpc()
        self.setup_mqtt()
        self.setup_ollama()
        
        logger.info("🤖 Robot1 AI Service started in REALTIME MODE")
        logger.info("📡 Nano at %s:%s (from config/device_config.yaml if present, else NANO_IP env)", self.nano_ip, self.nano_port)
    
    def setup_xmlrpc(self):
        """Setup XML-RPC connection to Nano for REAL camera/LiDAR/odom. Uses 10s timeout so startup does not hang if Nano is down."""
        nano_url = f'http://{self.nano_ip}:{self.nano_port}'
        logger.info("Attempting XML-RPC connection to Nano at %s:%s (timeout 10s)...", self.nano_ip, self.nano_port)
        try:
            transport = TimeoutXMLRPCTransport(timeout=10)
            self.nano_xmlrpc = xmlrpc.client.ServerProxy(nano_url, transport=transport)

            # Test connection (will raise or return within timeout)
            status = self.nano_xmlrpc.get_robot_status()
            if status.get('success', False):
                logger.info("✅ XML-RPC connected to Nano at %s", nano_url)
                cam_ok = status.get('camera_available', False)
                logger.info("   Camera available: %s", cam_ok)
                if not cam_ok:
                    logger.warning("   Nano camera not available: %s", status.get('camera_message', 'unknown'))
            else:
                logger.warning("⚠️ Nano connection test failed: %s", status.get('status_message'))
        except OSError as e:
            err_msg = getattr(e, 'strerror', str(e))
            err_no = getattr(e, 'errno', None)
            logger.error(
                "❌ XML-RPC connection to Nano failed: %s (errno=%s). "
                "From Orin run: ping %s  and  nc -zv %s %s",
                err_msg, err_no, self.nano_ip, self.nano_ip, self.nano_port
            )
            logger.warning("   Ensure Nano bringup is running and port 8000 is open. AI will use test image/fallback until Nano is reachable.")
            self.nano_xmlrpc = None
        except Exception as e:
            exc_type = type(e).__name__
            logger.error("❌ XML-RPC connection to Nano failed: %s: %s", exc_type, e)
            logger.warning("   From Orin run: ping %s  and  nc -zv %s %s . AI will use test image/fallback until Nano is reachable.", self.nano_ip, self.nano_ip, self.nano_port)
            self.nano_xmlrpc = None
    
    def setup_mqtt(self):
        """Setup MQTT client"""
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        
        try:
            self.mqtt_client.connect(self.mqtt_broker_host, self.mqtt_broker_port, 60)
            self.mqtt_client.loop_start()
            logger.info(f"Connected to MQTT broker at {self.mqtt_broker_host}:{self.mqtt_broker_port}")
        except Exception as e:
            logger.error(f"MQTT connection failed: {e}")
    
    def setup_ollama(self):
        """Setup Ollama AI service"""
        try:
            response = requests.get(f"{self.ollama_host}/api/tags", timeout=5)
            if response.status_code == 200:
                models = response.json().get('models', [])
                model_names = [model['name'] for model in models]
                logger.info(f"Ollama connected. Available models: {model_names}")
        except Exception as e:
            logger.error(f"Ollama setup failed: {e}")
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            logger.info("MQTT Connected successfully")
            client.subscribe(f"robots/{self.robot_id}/command")
        else:
            logger.error(f"MQTT Connection failed with code {rc}")
    
    def capture_camera_image(self) -> str:
        """Capture REAL image from Nano camera via XML-RPC"""
        try:
            if not self.nano_xmlrpc:
                logger.warning("Using test image (no XML-RPC connection to Nano)")
                return self.create_test_image()
            response = self.nano_xmlrpc.get_camera_image()
            if response.get('success', False):
                image_data = response.get('image_data', '')
                if image_data:
                    logger.info(f"✅ Captured REAL image from Nano ({response.get('width')}x{response.get('height')})")
                    return image_data
                else:
                    logger.warning("Empty image data from Nano — check Nano camera init and permissions")
            else:
                logger.warning("Nano camera returned failure: %s", response.get('message', 'unknown'))
            # Fallback to test image
            logger.warning("Using test image (Nano camera unavailable). Fix: on Nano ensure camera init (chmod 666 /dev/video0, camera_device in launch), then restart Nano bringup.")
            return self.create_test_image()
        except Exception as e:
            logger.error("Image capture failed (Nano unreachable or RPC error): %s", e)
            return self.create_test_image()
    
    def create_test_image(self) -> str:
        """Create a test image for fallback"""
        try:
            img = Image.new('RGB', (640, 480), color='gray')
            draw = ImageDraw.Draw(img)
            draw.text((200, 200), "FALLBACK TEST IMAGE", fill='red')
            draw.text((180, 250), f"Robot {self.robot_id} (REALTIME)", fill='white')
            
            buffer = io.BytesIO()
            img.save(buffer, format='JPEG')
            return base64.b64encode(buffer.getvalue()).decode('utf-8')
        except Exception as e:
            logger.error(f"Test image creation failed: {e}")
            return ""
    
    def get_real_lidar_data(self) -> Dict[str, Any]:
        """Get REAL LiDAR data from Nano via XML-RPC"""
        try:
            if self.nano_xmlrpc:
                response = self.nano_xmlrpc.get_lidar_data()
                if response.get('success', False):
                    logger.debug(f"✅ Got REAL LiDAR: min={response.get('min_range')}m, readings={response.get('num_readings')}")
                    # Prefer direction computed on Nano if available (min_angle_deg), else fall back to 0.0
                    direction_deg = response.get('min_angle_deg', None)
                    if direction_deg is None:
                        # fallback: use min_angle radians if present
                        min_angle = response.get('min_angle', None)
                        if min_angle is not None:
                            try:
                                direction_deg = float(min_angle) * 180.0 / math.pi
                            except Exception:
                                direction_deg = 0.0
                        else:
                            direction_deg = 0.0

                    lidar_source = response.get("lidar_source", "UNKNOWN")
                    if lidar_source == "DUMMY":
                        sensor_type = "LiDAR (SIMULATED DUMMY)"
                        confidence = 0.6
                        source = "SIMULATED"
                    elif lidar_source == "RPLIDAR":
                        sensor_type = "LiDAR (REAL RPLIDAR)"
                        confidence = 0.95
                        source = "REAL"
                    elif lidar_source == "FALLBACK":
                        # Nano returned fallback (no /scan yet, or rplidar down).
                        sensor_type = "LiDAR (FALLBACK DUMMY)"
                        confidence = 0.6
                        source = "FALLBACK"
                    else:
                        # We got scan data, but we can't confidently identify the publisher.
                        sensor_type = "LiDAR (UNKNOWN SOURCE)"
                        confidence = 0.8
                        source = "UNKNOWN"

                    lidar = {
                        "nearest_distance": float(response.get('min_range', 0.0)),
                        "obstacle_direction": float(direction_deg),
                        "obstacle_size": 0.5,  # placeholder size (no size estimation yet)
                        "confidence": confidence,
                        "timestamp": datetime.now().isoformat(),
                        "sensor_type": sensor_type,
                        "source": source,
                        "num_readings": int(response.get('num_readings', 0)),
                        "raw_data": response
                    }
                    self._last_lidar = lidar
                    return lidar
            
            # If REAL LiDAR isn't available, fall back to last known LiDAR (or defaults).
            fallback = dict(self._last_lidar)
            fallback["timestamp"] = datetime.now().isoformat()
            fallback["source"] = "FALLBACK"
            return fallback
            
        except Exception as e:
            logger.error(f"Real LiDAR fetch failed: {e}")
            fallback = dict(self._last_lidar)
            fallback["timestamp"] = datetime.now().isoformat()
            fallback["source"] = "FALLBACK"
            return fallback
    
    def get_real_odometry(self) -> Dict[str, Any]:
        """Get REAL odometry from Nano via XML-RPC"""
        try:
            if self.nano_xmlrpc:
                response = self.nano_xmlrpc.get_odometry_data()
                if response.get('success', False):
                    nano_odom_source = str(response.get("odom_source", "UNKNOWN"))
                    # Convert quaternion -> yaw (theta)
                    qx = float(response.get('orientation_x', 0.0))
                    qy = float(response.get('orientation_y', 0.0))
                    qz = float(response.get('orientation_z', 0.0))
                    qw = float(response.get('orientation_w', 1.0))
                    # yaw (Z axis rotation)
                    siny_cosp = 2.0 * (qw * qz + qx * qy)
                    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                    theta = math.atan2(siny_cosp, cosy_cosp)

                    # Only mark as REAL if Nano confirms /odom is from motor driver.
                    # If dummy is present (or mixed publishers), do not claim "REAL".
                    if nano_odom_source == "MOTOR_DRIVER":
                        source = "REAL"
                    elif nano_odom_source == "DUMMY":
                        source = "SIMULATED"
                    elif "MOTOR_DRIVER" in nano_odom_source and "DUMMY" in nano_odom_source:
                        source = "MIXED"
                    else:
                        source = "UNKNOWN"

                    odom = {
                        "x": float(response.get('position_x', 0.0)),
                        "y": float(response.get('position_y', 0.0)),
                        "z": float(response.get('position_z', 0.0)),
                        "theta": float(theta),
                        "linear_v": float(response.get('linear_velocity', 0.0)),
                        "angular_v": float(response.get('angular_velocity', 0.0)),
                        "source": source
                    }
                    self._last_odom = odom
                    return odom

            # If REAL odometry isn't available, fall back to last known odom (or defaults).
            fallback = dict(self._last_odom)
            fallback["source"] = "FALLBACK"
            return fallback
        except Exception as e:
            logger.error(f"Real odometry fetch failed: {e}")
            fallback = dict(self._last_odom)
            fallback["source"] = "FALLBACK"
            return fallback
    
    def simulate_lidar_data(self, x: float, y: float) -> Dict[str, Any]:
        """Fallback simulated LiDAR data"""
        return {
            "nearest_distance": round(random.uniform(0.5, 3.0), 2),
            "obstacle_direction": round(random.uniform(0, 360), 1),
            "obstacle_size": round(random.uniform(0.2, 1.5), 2),
            "confidence": round(random.uniform(0.7, 0.95), 2),
            "timestamp": datetime.now().isoformat(),
            "sensor_type": "LiDAR (SIMULATED FALLBACK)",
            "robot_position": {"x": x, "y": y}
        }
    
    def generate_ai_message(self, image_data: str, x: float, y: float, lidar_data: Dict[str, Any], odom: Optional[Dict[str, Any]] = None) -> str:
        """Generate AI message using MicroViT/VIT + Ollama"""
        try:
            # Priority: MicroViT > VIT > Fallback
            if self.use_microvit and self.microvit_model:
                return self._generate_with_microvit(image_data, x, y, lidar_data, odom=odom)
            elif self.use_vit and self.vit_model:
                return self._generate_with_vit(image_data, x, y, lidar_data, odom=odom)
            else:
                return self.generate_fallback_message(x, y, lidar_data)
                
        except Exception as e:
            logger.error(f"AI message generation failed: {e}")
            import traceback
            logger.debug(f"Traceback: {traceback.format_exc()}")
            return self.generate_fallback_message(x, y, lidar_data)
    
    def _generate_with_microvit(self, image_data: str, x: float, y: float, lidar_data: Dict[str, Any], odom: Optional[Dict[str, Any]] = None) -> str:
        """Generate AI message using MicroViT for fast image preprocessing + Ollama for text generation"""
        try:
            # Step 1: MicroViT preprocessing (~9ms on GPU, ~50-200ms on CPU)
            image_description = self.microvit_model.analyze_image(image_data)
            logger.debug(f"MicroViT image description: {image_description}")
            
            # Step 2: Use Ollama to generate creative message from preprocessed features + LiDAR data
            theta = 0.0 if not odom else float(odom.get("theta", 0.0))
            linear_v = 0.0 if not odom else float(odom.get("linear_v", 0.0))
            angular_v = 0.0 if not odom else float(odom.get("angular_v", 0.0))
            odom_source = "UNKNOWN" if not odom else odom.get("source", "UNKNOWN")

            prompt = f"""You are Robot {self.robot_id}.

Current odometry:
- Position: ({x:.2f}, {y:.2f})
- Heading (theta): {theta:.3f} rad
- Linear velocity: {linear_v:.3f} m/s
- Angular velocity: {angular_v:.3f} rad/s
- Odometry source: {odom_source}

Based on my visual analysis (preprocessed with MicroViT), I can see: {image_description}

My LiDAR sensors report:
- Distance to nearest obstacle: {lidar_data.get('nearest_distance', 'unknown')} meters
- Obstacle direction: {lidar_data.get('obstacle_direction', 'unknown')} degrees
- Obstacle size: {lidar_data.get('obstacle_size', 'unknown')} meters
- Sensor type: {lidar_data.get('sensor_type', 'unknown')}
- Confidence level: {lidar_data.get('confidence', 0.5)}

Generate a creative, informative status message (under 150 words) that:
1. Describes what I can see visually based on the preprocessed image features
2. Reports what my LiDAR sensors detect
3. Provides a combined assessment of the situation
4. Suggests what action I should take

Make it sound like a robot reporting to other robots. Be engaging and specific about my location and motion."""

            # Use Ollama to generate the final message
            ai_message = self._generate_with_ollama(prompt)
            # Log the full AI message so it is fully visible in the terminal/logs
            logger.info("✅ Generated AI message with MicroViT+Ollama (FULL):\n%s", ai_message.strip())
            return ai_message.strip()
            
        except Exception as e:
            logger.error(f"❌ MicroViT generation failed: {e}")
            import traceback
            logger.debug(f"Traceback: {traceback.format_exc()}")
            return self.generate_fallback_message(x, y, lidar_data)
    
    def _generate_with_vit(self, image_data: str, x: float, y: float, lidar_data: Dict[str, Any], odom: Optional[Dict[str, Any]] = None) -> str:
        """Generate AI message using VIT/BLIP for image analysis + Ollama for text generation"""
        try:
            # Use VIT for image analysis
            image_description = self.vit_model.analyze_image(image_data)
            logger.debug(f"VIT image description: {image_description}")
            
            theta = 0.0 if not odom else float(odom.get("theta", 0.0))
            linear_v = 0.0 if not odom else float(odom.get("linear_v", 0.0))
            angular_v = 0.0 if not odom else float(odom.get("angular_v", 0.0))
            odom_source = "UNKNOWN" if not odom else odom.get("source", "UNKNOWN")

            prompt = f"""You are Robot {self.robot_id}.

Current odometry:
- Position: ({x:.2f}, {y:.2f})
- Heading (theta): {theta:.3f} rad
- Linear velocity: {linear_v:.3f} m/s
- Angular velocity: {angular_v:.3f} rad/s
- Odometry source: {odom_source}

Based on my visual analysis, I can see: {image_description}

My LiDAR sensors report:
- Distance to nearest obstacle: {lidar_data.get('nearest_distance', 'unknown')} meters
- Sensor type: {lidar_data.get('sensor_type', 'unknown')}
- Confidence level: {lidar_data.get('confidence', 0.5)}

Generate a creative, informative status message (under 150 words) describing what I see and detect.
Make it sound like a robot reporting to other robots. Mention my motion if relevant."""

            ai_message = self._generate_with_ollama(prompt)
            logger.info("✅ Generated AI message with VIT+Ollama (FULL):\n%s", ai_message.strip())
            return ai_message.strip()
            
        except Exception as e:
            logger.error(f"❌ VIT generation failed: {e}")
            return self.generate_fallback_message(x, y, lidar_data)
    
    def _generate_with_ollama(self, prompt: str) -> str:
        """Generate text using Ollama. Retries on 500 or connection errors (Ollama may still be loading)."""
        model_to_use = getattr(self, 'ollama_text_model', self.ollama_model)
        timeout_seconds = int(os.getenv('OLLAMA_TIMEOUT', '120'))
        max_tokens = int(os.getenv('OLLAMA_MAX_TOKENS', '500'))
        stream_enabled = os.getenv('OLLAMA_STREAM', 'false').lower() == 'true'
        stream_print = os.getenv('OLLAMA_STREAM_PRINT', 'true').lower() == 'true'
        max_retries = 2  # retry on 500 or connection error (e.g. Ollama runner still starting)

        last_error = None
        for attempt in range(max_retries + 1):
            try:
                if not stream_enabled:
                    response = requests.post(
                        f"{self.ollama_host}/api/generate",
                        json={
                            "model": model_to_use,
                            "prompt": prompt,
                            "stream": False,
                            "options": {"temperature": 0.8, "num_predict": max_tokens}
                        },
                        timeout=timeout_seconds
                    )
                    if response.status_code == 500 and attempt < max_retries:
                        logger.warning("Ollama returned 500 (runner may be starting), retrying in 5s...")
                        time.sleep(5)
                        continue
                    response.raise_for_status()
                    return response.json().get('response', '').strip()

                # Streaming mode
                with requests.post(
                    f"{self.ollama_host}/api/generate",
                    json={
                        "model": model_to_use,
                        "prompt": prompt,
                        "stream": True,
                        "options": {"temperature": 0.8, "num_predict": max_tokens}
                    },
                    timeout=timeout_seconds,
                    stream=True
                ) as response:
                    if response.status_code == 500 and attempt < max_retries:
                        logger.warning("Ollama returned 500 (runner may be starting), retrying in 5s...")
                        time.sleep(5)
                        continue
                    response.raise_for_status()
                    full_text_parts = []
                    for line in response.iter_lines(decode_unicode=True):
                        if not line:
                            continue
                        try:
                            chunk = json.loads(line)
                        except Exception:
                            continue
                        token = chunk.get('response', '')
                        if token:
                            full_text_parts.append(token)
                            if stream_print:
                                print(token, end='', flush=True)
                        if chunk.get('done', False):
                            break
                    if stream_print:
                        print('', flush=True)
                    return ''.join(full_text_parts).strip()
            except (requests.exceptions.ConnectionError, requests.exceptions.Timeout) as e:
                last_error = e
                if attempt < max_retries:
                    logger.warning("Ollama connection error (%s), retrying in 5s...", e)
                    time.sleep(5)
                else:
                    raise
            except requests.exceptions.HTTPError as e:
                last_error = e
                if attempt < max_retries and e.response is not None and e.response.status_code == 500:
                    logger.warning("Ollama returned 500, retrying in 5s...")
                    time.sleep(5)
                else:
                    logger.error("Ollama generation failed: %s", e)
                    raise
            except Exception as e:
                last_error = e
                logger.error("Ollama generation failed: %s", e)
                raise
        if last_error:
            raise last_error
        return ""
    
    def generate_fallback_message(self, x: float, y: float, lidar_data: Dict[str, Any]) -> str:
        """Generate fallback message"""
        distance = lidar_data.get('nearest_distance', 'unknown')
        sensor_type = lidar_data.get('sensor_type', 'unknown')
        return f"Robot {self.robot_id} at ({x:.1f}, {y:.1f}): {sensor_type} shows {distance}m to nearest obstacle."
    
    def detect_obstacle_and_generate_message(self):
        """Main detection and AI message generation process using REAL data"""
        try:
            # Get REAL odometry
            odom = self.get_real_odometry()
            x, y = odom['x'], odom['y']
            logger.info(f"Position ({odom['source']}): ({x:.2f}, {y:.2f})")
            
            # Capture REAL image from Nano
            image_data = self.capture_camera_image()
            if not image_data:
                logger.error("Failed to capture image")
                return
            
            # Get REAL LiDAR data from Nano
            lidar_data = self.get_real_lidar_data()
            logger.info(f"LiDAR ({lidar_data.get('sensor_type')}): {lidar_data.get('nearest_distance')}m")
            
            # Generate AI message
            ai_message = self.generate_ai_message(image_data, x, y, lidar_data, odom=odom)
            
            # Create obstacle event (schema aligned with controller expectations)
            confidence = float(lidar_data.get('confidence', 0.5))
            obstacle_event = {
                "event": "obstacle_detected",
                "robot_id": self.robot_id,
                "context_id": str(uuid.uuid4()),
                "time": datetime.now().isoformat(),
                "location": {"x": x, "y": y},
                "natural_message": ai_message,
                "message_type": "ai_generated_with_vision",
                "obstacle_type": "detection",
                "severity": "medium",
                "confidence": confidence,
                "proposed_action": "report",
                "ai_model": self.ollama_model,
                "lidar_data": lidar_data,
                "odometry_source": odom['source'],
                "mode": "REALTIME"
            }
            
            # Publish via MQTT
            topic = f"robots/{self.robot_id}/obstacle"
            self.mqtt_client.publish(topic, json.dumps(obstacle_event), qos=1)
            
            # Also log the full message that was published
            logger.info("Published REALTIME obstacle event (FULL):\n%s", ai_message.strip())
            self.detection_count += 1
            
        except Exception as e:
            logger.error(f"Obstacle detection failed: {e}")
    
    def run_continuous_detection(self, interval: int = 5):
        """Run continuous obstacle detection with REAL data. One failed cycle never stops the loop."""
        logger.info("Starting continuous REALTIME detection every %s seconds. Press Ctrl+C to stop.", interval)
        
        try:
            while True:
                try:
                    self.detect_obstacle_and_generate_message()
                except Exception as e:
                    logger.error("Detection cycle failed (will retry): %s", e)
                try:
                    time.sleep(interval)
                except KeyboardInterrupt:
                    raise
        except KeyboardInterrupt:
            logger.info("Stopping detection...")
        finally:
            if self.mqtt_client:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Robot1 AI Service (REALTIME MODE)")
    parser.add_argument('--simulation', action='store_true', help='Run in continuous mode')
    parser.add_argument('--interval', type=int, default=5, help='Detection interval in seconds (default: 5)')
    args = parser.parse_args()
    
    print("=" * 60)
    print("Robot1 AI Service - REALTIME MODE")
    print("Using REAL camera, LiDAR, and odometry from Nano")
    print("=" * 60)
    
    ai_service = Robot1AIServiceRealtime()
    
    if args.simulation:
        ai_service.run_continuous_detection(interval=args.interval)
    else:
        ai_service.run_continuous_detection(interval=args.interval)
