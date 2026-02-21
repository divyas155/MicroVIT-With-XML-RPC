#!/usr/bin/env python3
"""
Controller AI Service - Complete Package
Lightweight AI system for decision making on Controller (8GB RAM)
Uses Ollama with lightweight models for local AI processing
"""

import json
import time
import uuid
import requests
from datetime import datetime
from typing import Dict, List, Any
import paho.mqtt.client as mqtt
import logging
import os
from dotenv import load_dotenv

# Load environment variables from config/.env (so systemd-run service picks up AI_TIMEOUT, etc.)
_config_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
load_dotenv(os.path.join(_config_dir, '.env'))
load_dotenv()  # override with cwd .env if present

# Configure logging
_log_dir = os.getenv('LOG_DIR', './logs')
_handlers = [logging.StreamHandler()]
# Avoid crashing if ./logs doesn't exist (common on fresh setups / laptops).
if os.path.isdir(_log_dir):
    _handlers.insert(0, logging.FileHandler(os.path.join(_log_dir, 'controller_ai.log')))
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=_handlers
)
logger = logging.getLogger(__name__)

class ControllerAIService:
    def __init__(self):
        # Configuration from environment
        self.mqtt_broker_host = os.getenv('MQTT_BROKER_HOST', 'localhost')
        self.mqtt_broker_port = int(os.getenv('MQTT_BROKER_PORT', 1883))
        self.mqtt_username = os.getenv('MQTT_BROKER_USERNAME', '')
        self.mqtt_password = os.getenv('MQTT_BROKER_PASSWORD', '')
        self.ollama_host = os.getenv('OLLAMA_HOST', 'http://localhost:11434')
        self.ollama_model = os.getenv('OLLAMA_MODEL', 'phi3:mini')
        self.controller_id = os.getenv('CONTROLLER_ID', 'central_controller')
        # AI tuning (qwen2.5 on Pi needs longer timeout to finish; increase AI_TIMEOUT in .env if still timing out)
        self.ai_temperature = float(os.getenv('AI_TEMPERATURE', '0.7'))
        self.ai_max_tokens = int(os.getenv('AI_MAX_TOKENS', '60'))
        self.ai_timeout = int(os.getenv('AI_TIMEOUT', '25'))
        
        # Initialize MQTT client
        self.client = mqtt.Client()
        self.robot_states = {}  # Track robot states
        self.active_obstacles = {}  # Track active obstacles
        
        # MQTT setup
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect
        
        if self.mqtt_username and self.mqtt_password:
            self.client.username_pw_set(self.mqtt_username, self.mqtt_password)
        
        # Set Last Will Testament
        self.client.will_set("controller/status", json.dumps({
            "status": "offline",
            "time": datetime.now().isoformat()
        }), qos=1, retain=True)
        
        # Setup Ollama
        self.setup_ollama()

    def setup_ollama(self):
        """Setup Ollama AI service and pre-load the model so first obstacle gets a fast response."""
        try:
            # Test Ollama connection
            response = requests.get(f"{self.ollama_host}/api/tags", timeout=5)
            if response.status_code == 200:
                models = response.json().get('models', [])
                model_names = [model['name'] for model in models]
                logger.info(f"Controller Ollama connected. Available models: {model_names}")
                
                if self.ollama_model not in model_names:
                    logger.warning(f"Model {self.ollama_model} not found. Available: {model_names}")
                    if model_names:
                        self.ollama_model = model_names[0]
                        logger.info(f"Using model: {self.ollama_model}")
            else:
                logger.error(f"Ollama connection failed: {response.status_code}")
                return
        except Exception as e:
            logger.error(f"Ollama setup failed: {e}")
            return

        # Warmup: load the model into memory now so first obstacle request doesn't timeout (Pi is slow to load).
        warmup_timeout = max(120, self.ai_timeout * 4)
        try:
            logger.info(f"Warming up Ollama model {self.ollama_model} (may take 1-2 min on Pi)...")
            r = requests.post(
                f"{self.ollama_host}/api/generate",
                json={
                    "model": self.ollama_model,
                    "prompt": "Say OK in one word.",
                    "stream": False,
                    "options": {"num_predict": 2},
                },
                timeout=warmup_timeout,
            )
            if r.status_code == 200:
                logger.info("Ollama model ready.")
            else:
                logger.warning(f"Ollama warmup returned {r.status_code}; first decision may be slow.")
        except requests.exceptions.Timeout:
            logger.warning("Ollama warmup timed out; model may still be loading. First decision may be slow.")
        except Exception as e:
            logger.warning(f"Ollama warmup failed: {e}; first decision may be slow.")

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            logger.info("Connected to MQTT broker")
            
            # Subscribe to all robot obstacle topics
            client.subscribe("robots/+/obstacle", qos=1)
            client.subscribe("robots/+/status", qos=1)
            
            # Publish controller online status
            client.publish("controller/status", json.dumps({
                "status": "online",
                "time": datetime.now().isoformat()
            }), qos=1, retain=True)
        else:
            logger.error(f"Failed to connect to MQTT broker: {rc}")

    def on_message(self, client, userdata, msg):
        try:
            topic_parts = msg.topic.split('/')
            
            if len(topic_parts) >= 3:
                robot_id = topic_parts[1]
                message_type = topic_parts[2]
                
                if message_type == "obstacle":
                    self.handle_obstacle_event(robot_id, msg.payload.decode())
                elif message_type == "status":
                    self.handle_status_update(robot_id, msg.payload.decode())
                    
        except Exception as e:
            logger.error(f"Error processing message: {e}")

    def on_disconnect(self, client, userdata, rc):
        logger.warning("Disconnected from MQTT broker")

    def handle_obstacle_event(self, robot_id: str, payload: str):
        """Handle obstacle detection events using local AI for decision making."""
        try:
            data = json.loads(payload)
            # Support both Robot1 payload (obstacle_type, message_type, event) and legacy
            obstacle_type = data.get('obstacle_type') or data.get('message_type') or data.get('event', 'unknown')
            loc = data.get('location', {})
            logger.info(f"Obstacle detected by {robot_id}: {obstacle_type} at ({loc.get('x', 'N/A')}, {loc.get('y', 'N/A')})")
            
            # Store obstacle
            obstacle_id = data.get('context_id', str(uuid.uuid4()))
            self.active_obstacles[obstacle_id] = data
            
            # Get AI-generated message from Robot1
            natural_message = data.get('natural_message', 'No message provided')
            logger.info(f"Robot1 AI Message: {natural_message}")
            
            # Use local AI to analyze the situation and make a decision
            ai_decision = self.get_ai_decision(data, natural_message, robot_id)
            
            if ai_decision:
                logger.info(f"AI Decision: {ai_decision}")
                self.execute_ai_decision(ai_decision, data, robot_id)
            else:
                logger.warning("No AI decision received")
                
        except Exception as e:
            logger.error(f"Error handling obstacle event: {e}")

    def get_ai_decision(self, obstacle_data: Dict[str, Any], natural_message: str, robot_id: str) -> Dict[str, Any]:
        """Use local Ollama AI to analyze the situation and make a decision."""
        try:
            # Prepare the situation for AI analysis
            # Derive confidence from top-level or lidar_data (Robot1 sends both)
            confidence = obstacle_data.get('confidence')
            if confidence is None and isinstance(obstacle_data.get('lidar_data'), dict):
                confidence = obstacle_data['lidar_data'].get('confidence', 0.5)
            confidence = confidence if confidence is not None else 0.5
            loc = obstacle_data.get('location', {})
            obs_coords = obstacle_data.get('obstacle_coordinates') or loc
            obs_type = obstacle_data.get('obstacle_type') or obstacle_data.get('message_type') or 'unknown'
            # Keep prompt short so Ollama on Pi can respond within timeout (qwen2.5 is slow)
            msg_preview = (natural_message[:280] + "...") if len(natural_message) > 280 else natural_message
            situation = (
                f"Robot {robot_id} reported obstacle ({obs_type}, severity {obstacle_data.get('severity', 'medium')}). "
                f"Robot position: ({loc.get('x', 'N/A')}, {loc.get('y', 'N/A')}). "
                f"Obstacle coordinates: ({obs_coords.get('x', 'N/A')}, {obs_coords.get('y', 'N/A')}). "
                f"Summary: {msg_preview}\n\n"
                "You must reply in two parts.\n"
                "Part 1 - One short sentence in plain English for the human operator (what is happening and what you recommend). Do not use JSON here.\n"
                "Part 2 - On the next line, only a JSON object with: action_type (investigation, obstacle_removal, reroute, or monitor), target_robot (robot1_orin or helper_robot), reasoning (brief), priority (high, medium, or low). "
                "Use helper_robot for investigation/obstacle_removal so it navigates to the obstacle coordinates.\n"
                "Example Part 1: Recommend sending the robot to investigate the obstacle and adjust path if needed.\n"
                "Example Part 2: {\"action_type\":\"investigation\",\"target_robot\":\"robot1_orin\",\"reasoning\":\"obstacle detected\",\"priority\":\"medium\"}"
            )
            
            # Call Ollama API
            response = requests.post(
                f"{self.ollama_host}/api/generate",
                json={
                    "model": self.ollama_model,
                    "prompt": situation,
                    "stream": False,
                    "options": {
                        "temperature": self.ai_temperature,
                        "top_p": 0.9,
                        "num_predict": self.ai_max_tokens  # Ollama native param; max_tokens can cause 500 on older versions
                    }
                },
                timeout=self.ai_timeout
            )
            
            if response.status_code == 200:
                result = response.json()
                ai_response = result.get('response', '').strip()
                logger.info(f"AI Response raw: {ai_response}")
                
                # Try to parse JSON response while also extracting a human-readable message
                try:
                    # Extract JSON from response
                    import re
                    json_match = re.search(r'\{.*\}', ai_response, re.DOTALL)
                    if json_match:
                        json_text = json_match.group()
                        try:
                            decision_data = json.loads(json_text)
                        except json.JSONDecodeError:
                            decision_data = {}
                        # Everything before the JSON block = natural language message for operator
                        human_message = ai_response[: json_match.start()].strip()
                        # Strip "Part 1 - " or "Part 1:" if model included it
                        for prefix in ("Part 1 - ", "Part 1 -", "Part 1:", "Part 1 "):
                            if human_message.startswith(prefix):
                                human_message = human_message[len(prefix):].strip()
                                break
                        # If model echoed the instruction or returned only JSON, use a built sentence instead
                        if (not human_message or human_message.strip().startswith('{') or
                                "one short sentence in plain english" in human_message.lower() or
                                "do not use json here" in human_message.lower()):
                            action = decision_data.get('action_type', 'investigation')
                            target = decision_data.get('target_robot', 'robot1_orin')
                            reason = decision_data.get('reasoning', '') or 'AI analysis of situation'
                            human_message = f"Recommend {action.replace('_', ' ')} for {target}. {str(reason)[:120]}{'...' if len(str(reason)) > 120 else ''}"
                        logger.info(f"Controller AI Message: {human_message}")
                        return {
                            "decision": ai_response,
                            "target_robot": decision_data.get('target_robot', 'jetson2'),
                            "action_type": decision_data.get('action_type', 'investigation'),
                            "reasoning": decision_data.get('reasoning', 'AI decision'),
                            "priority": decision_data.get('priority', 'medium'),
                            "confidence": 0.8,
                            "message": human_message,
                        }
                except json.JSONDecodeError:
                    logger.warning("Could not parse AI response as JSON, using fallback")
                
                # Fallback: parse response text
                return self.parse_ai_response(ai_response)
            else:
                logger.error(f"Ollama request failed: {response.status_code}")
                
        except Exception as e:
            logger.error(f"Error getting AI decision: {e}")
        
        return None

    def parse_ai_response(self, ai_response: str) -> Dict[str, Any]:
        """Parse AI response text to extract decision information."""
        response_lower = ai_response.lower()
        
        # Determine action type
        if any(word in response_lower for word in ['remove', 'clear', 'obstacle', 'help']):
            action_type = 'obstacle_removal'
        elif any(word in response_lower for word in ['investigate', 'check', 'examine']):
            action_type = 'investigation'
        elif any(word in response_lower for word in ['avoid', 'reroute', 'bypass']):
            action_type = 'reroute'
        elif any(word in response_lower for word in ['wait', 'standby', 'monitor']):
            action_type = 'monitor'
        else:
            action_type = 'investigation'
        
        # Determine target robot
        if 'jetson1' in response_lower:
            target_robot = 'jetson1'
        else:
            target_robot = 'jetson2'
        
        # Determine priority
        if any(word in response_lower for word in ['urgent', 'critical', 'immediate']):
            priority = 'high'
        elif any(word in response_lower for word in ['low', 'minor']):
            priority = 'low'
        else:
            priority = 'medium'
        
        # Prefer natural language for operator; if response is raw JSON, build a sentence
        raw = ai_response.strip()
        if raw.startswith('{'):
            message = f"Recommend {action_type.replace('_', ' ')} for {target_robot}. Priority: {priority}."
        else:
            message = raw[:250] + ("..." if len(raw) > 250 else "")
        logger.info(f"Controller AI Message (parsed): {message}")
        return {
            "decision": ai_response,
            "target_robot": target_robot,
            "action_type": action_type,
            "reasoning": "AI analysis of situation",
            "priority": priority,
            "confidence": 0.7,
            "message": message,
        }

    def execute_ai_decision(self, ai_decision: Dict[str, Any], obstacle_data: Dict[str, Any], reporting_robot: str):
        """Execute the AI decision by sending appropriate commands."""
        try:
            action_type = ai_decision.get('action_type', 'investigation')
            target_robot = ai_decision.get('target_robot', 'jetson2')
            decision_text = ai_decision.get('decision', '')
            reasoning = ai_decision.get('reasoning', '')
            priority = ai_decision.get('priority', 'medium')
            human_message = ai_decision.get('message', '')
            
            # Map AI target names to MQTT robot IDs
            target_robot = {'helper_robot': 'jetson2', 'robot1_orin': 'jetson1', 'jetson1': 'jetson1', 'jetson2': 'jetson2'}.get(target_robot, target_robot)
            
            if human_message:
                logger.info(f"Controller AI Message: {human_message}")
            
            logger.info(f"Executing AI decision: {action_type} for {target_robot}")
            
            # Use obstacle coordinates for task location when delegating to helper (so helper navigates to obstacle)
            loc = obstacle_data.get('location', {})
            obs_coords = obstacle_data.get('obstacle_coordinates')
            task_location = obs_coords if obs_coords and target_robot == 'jetson2' else loc
            
            obs_type = obstacle_data.get('obstacle_type') or obstacle_data.get('message_type') or 'unknown'
            command = {
                "command_id": str(uuid.uuid4()),
                "timestamp": datetime.now().isoformat(),
                "sender": self.controller_id,
                "recipient": target_robot,
                "action": action_type,
                "priority": priority,
                "location": task_location,
                "obstacle_coordinates": obs_coords or loc,
                "robot_position": loc,
                "obstacle_type": obs_type,
                "ai_decision": decision_text,
                "ai_reasoning": reasoning,
                "ai_message": human_message or reasoning or decision_text,
                "context_id": obstacle_data.get('context_id', str(uuid.uuid4())),
                "ai_model": self.ollama_model
            }
            
            # Send command via MQTT (topic: robots/<robot_id>/command)
            topic = f"robots/{target_robot}/command"
            self.client.publish(topic, json.dumps(command), qos=1)
            
            logger.info(f"Sent {action_type} command to {target_robot}")
            
            # Also send a status update
            status_update = {
                "controller_status": "processing",
                "ai_decision": decision_text,
                "ai_reasoning": reasoning,
                "action_taken": action_type,
                "target_robot": target_robot,
                "priority": priority,
                "timestamp": datetime.now().isoformat(),
                "ai_model": self.ollama_model
            }
            
            self.client.publish("controller/ai_decision", json.dumps(status_update), qos=1)
            
        except Exception as e:
            logger.error(f"Error executing AI decision: {e}")

    def handle_status_update(self, robot_id: str, payload: str):
        """Handle robot status updates."""
        try:
            data = json.loads(payload)
            self.robot_states[robot_id] = data
            logger.info(f"Status update from {robot_id}: {data.get('status', 'unknown')}")
        except Exception as e:
            logger.error(f"Error handling status update: {e}")

    def start(self):
        """Start the controller AI service."""
        try:
            logger.info("Starting Controller AI Service...")
            logger.info(f"Connecting to MQTT broker at {self.mqtt_broker_host}:{self.mqtt_broker_port}")
            logger.info(f"Using Ollama AI at {self.ollama_host} with model {self.ollama_model}")
            
            self.client.connect(self.mqtt_broker_host, self.mqtt_broker_port, 60)
            self.client.loop_forever()
            
        except Exception as e:
            logger.error(f"Controller AI error: {e}")

    def stop(self):
        """Stop the controller AI service."""
        logger.info("Stopping Controller AI Service...")
        self.client.loop_stop()
        self.client.disconnect()

if __name__ == "__main__":
    controller_ai = ControllerAIService()
    try:
        controller_ai.start()
    except KeyboardInterrupt:
        logger.info("Controller AI stopped by user")
        controller_ai.stop()
    except Exception as e:
        logger.error(f"Controller AI error: {e}")
        controller_ai.stop()

