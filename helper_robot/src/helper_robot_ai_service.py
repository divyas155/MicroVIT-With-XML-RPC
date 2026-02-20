#!/usr/bin/env python3
"""
Helper Robot AI Service - Complete Package
Lightweight AI system for task execution on Helper Robot (8GB RAM)
Uses Ollama with lightweight models for local AI processing
"""

import json
import time
import uuid
import requests
from datetime import datetime
from typing import Dict, Any
import paho.mqtt.client as mqtt
import logging
import os
from dotenv import load_dotenv
from flask import Flask, request, jsonify
import threading

# Load environment variables
load_dotenv()

# Setup logging
_log_dir = os.getenv('LOG_DIR', './logs')
_handlers = [logging.StreamHandler()]
# Avoid crashing if ./logs doesn't exist (common on fresh setups / laptops).
if os.path.isdir(_log_dir):
    _handlers.insert(0, logging.FileHandler(os.path.join(_log_dir, 'helper_robot_ai.log')))
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=_handlers
)
logger = logging.getLogger(__name__)

class HelperRobotAIService:
    def __init__(self):
        self.robot_id = os.getenv('ROBOT_ID', 'jetson2')
        self.ollama_host = os.getenv('OLLAMA_HOST', 'http://localhost:11434')
        self.ollama_model = os.getenv('OLLAMA_MODEL', 'phi3:mini')
        self.mqtt_broker_host = os.getenv('MQTT_BROKER_HOST', 'localhost')
        self.mqtt_broker_port = int(os.getenv('MQTT_BROKER_PORT', 1883))
        self.flask_port = int(os.getenv('FLASK_PORT', 5000))
        
        # Initialize components
        self.mqtt_client = None
        self.flask_app = None
        self.setup_mqtt()
        self.setup_ollama()
        self.setup_flask()
    
    def setup_mqtt(self):
        """Setup MQTT client"""
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        try:
            self.mqtt_client.connect(self.mqtt_broker_host, self.mqtt_broker_port, 60)
            self.mqtt_client.loop_start()
            logger.info(f"Connected to MQTT broker at {self.mqtt_broker_host}:{self.mqtt_broker_port}")
        except Exception as e:
            logger.error(f"MQTT connection failed: {e}")
    
    def setup_ollama(self):
        """Setup Ollama AI service"""
        try:
            # Test Ollama connection
            response = requests.get(f"{self.ollama_host}/api/tags", timeout=5)
            if response.status_code == 200:
                models = response.json().get('models', [])
                model_names = [model['name'] for model in models]
                logger.info(f"Helper Robot Ollama connected. Available models: {model_names}")
                
                if self.ollama_model not in model_names:
                    logger.warning(f"Model {self.ollama_model} not found. Available: {model_names}")
                    if model_names:
                        self.ollama_model = model_names[0]
                        logger.info(f"Using model: {self.ollama_model}")
            else:
                logger.error(f"Ollama connection failed: {response.status_code}")
        except Exception as e:
            logger.error(f"Ollama setup failed: {e}")
    
    def setup_flask(self):
        """Setup Flask web service for motor control"""
        self.flask_app = Flask(__name__)
        
        @self.flask_app.route('/')
        def index():
            return f"Helper Robot {self.robot_id} AI Service is running!"
        
        @self.flask_app.route('/health', methods=['GET'])
        def health():
            return jsonify({
                "status": "healthy",
                "robot_id": self.robot_id,
                "ai_model": self.ollama_model
            })
        
        @self.flask_app.route('/move', methods=['POST'])
        def move():
            """Execute movement command with AI analysis"""
            try:
                data = request.get_json()
                direction = data.get('direction', 'forward')
                distance = data.get('distance', 1.0)
                speed = data.get('speed', 0.5)
                
                # Use AI to analyze the movement
                ai_analysis = self.analyze_movement_with_ai(direction, distance, speed)
                
                # Simulate movement execution
                result = self.execute_movement(direction, distance, speed)
                
                return jsonify({
                    "success": True,
                    "direction": direction,
                    "distance": distance,
                    "speed": speed,
                    "ai_analysis": ai_analysis,
                    "execution_result": result
                })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.flask_app.route('/task', methods=['POST'])
        def execute_task():
            """Execute AI-analyzed task"""
            try:
                data = request.get_json()
                task_type = data.get('task_type', 'investigation')
                location = data.get('location', {})
                priority = data.get('priority', 'medium')
                
                # Use AI to plan task execution
                ai_plan = self.plan_task_with_ai(task_type, location, priority)
                
                # Execute the task
                result = self.execute_task_plan(ai_plan)
                
                return jsonify({
                    "success": True,
                    "task_type": task_type,
                    "ai_plan": ai_plan,
                    "execution_result": result
                })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        if rc == 0:
            logger.info("MQTT Connected successfully")
            client.subscribe(f"robots/{self.robot_id}/command")
        else:
            logger.error(f"MQTT Connection failed with code {rc}")
    
    def on_mqtt_message(self, client, userdata, msg):
        """Handle incoming MQTT messages"""
        try:
            message = json.loads(msg.payload.decode())
            logger.info(f"Received command: {message}")
            
            # Process command with AI
            self.process_command_with_ai(message)
                
        except Exception as e:
            logger.error(f"Error processing MQTT message: {e}")
    
    def analyze_movement_with_ai(self, direction: str, distance: float, speed: float) -> str:
        """Use AI to analyze movement parameters"""
        try:
            prompt = f"""
            You are Helper Robot {self.robot_id}. You need to move {direction} for {distance} meters at speed {speed}.
            Analyze this movement and provide:
            1. Safety considerations
            2. Optimal path planning
            3. Expected duration
            4. Potential obstacles to watch for
            
            Keep response under 100 words and be practical.
            """
            
            response = requests.post(
                f"{self.ollama_host}/api/generate",
                json={
                    "model": self.ollama_model,
                    "prompt": prompt,
                    "stream": False,
                    "options": {
                        "temperature": 0.6,
                        "top_p": 0.8,
                        "max_tokens": 150
                    }
                },
                timeout=20
            )
            
            if response.status_code == 200:
                result = response.json()
                ai_analysis = result.get('response', '').strip()
                logger.info(f"Movement AI analysis: {ai_analysis}")
                return ai_analysis
            else:
                return f"Movement analysis: {direction} for {distance}m at {speed} speed"
                
        except Exception as e:
            logger.error(f"Movement AI analysis failed: {e}")
            return f"Standard movement: {direction} for {distance}m"
    
    def plan_task_with_ai(self, task_type: str, location: Dict, priority: str) -> Dict[str, Any]:
        """Use AI to plan task execution"""
        try:
            prompt = f"""
            You are Helper Robot {self.robot_id}. You need to execute a {task_type} task.
            Location: {location}
            Priority: {priority}
            
            Create a task execution plan with:
            1. Step-by-step actions
            2. Safety measures
            3. Expected outcomes
            4. Contingency plans
            
            Respond in JSON format:
            {{
                "steps": ["step1", "step2", "step3"],
                "safety_measures": ["measure1", "measure2"],
                "expected_outcome": "description",
                "contingency": "backup plan"
            }}
            """
            
            response = requests.post(
                f"{self.ollama_host}/api/generate",
                json={
                    "model": self.ollama_model,
                    "prompt": prompt,
                    "stream": False,
                    "options": {
                        "temperature": 0.7,
                        "top_p": 0.9,
                        "max_tokens": 200
                    }
                },
                timeout=25
            )
            
            if response.status_code == 200:
                result = response.json()
                ai_response = result.get('response', '').strip()
                logger.info(f"Task AI plan: {ai_response}")
                
                # Try to parse JSON response
                try:
                    import re
                    json_match = re.search(r'\{.*\}', ai_response, re.DOTALL)
                    if json_match:
                        return json.loads(json_match.group())
                except json.JSONDecodeError:
                    pass
                
                # Fallback plan
                return {
                    "steps": [f"Execute {task_type} task", "Monitor progress", "Report completion"],
                    "safety_measures": ["Check environment", "Maintain safe distance"],
                    "expected_outcome": f"Successful {task_type} execution",
                    "contingency": "Retreat and report if obstacles encountered"
                }
            else:
                return self.create_fallback_plan(task_type, priority)
                
        except Exception as e:
            logger.error(f"Task AI planning failed: {e}")
            return self.create_fallback_plan(task_type, priority)
    
    def create_fallback_plan(self, task_type: str, priority: str) -> Dict[str, Any]:
        """Create fallback task plan when AI is unavailable"""
        return {
            "steps": [
                f"Navigate to task location",
                f"Execute {task_type} operation",
                "Monitor for obstacles",
                "Complete task and report"
            ],
            "safety_measures": [
                "Maintain safe operating distance",
                "Stop if obstacles detected",
                "Report status regularly"
            ],
            "expected_outcome": f"Successful completion of {task_type} task",
            "contingency": "Stop and request assistance if issues arise"
        }
    
    def process_command_with_ai(self, command: Dict[str, Any]):
        """Process incoming command with AI analysis"""
        try:
            action = command.get('action', 'unknown')
            location = command.get('location', {})
            priority = command.get('priority', 'medium')
            ai_decision = command.get('ai_decision', '')
            
            logger.info(f"Processing {action} command with AI analysis")
            
            # Use AI to understand the command
            ai_understanding = self.understand_command_with_ai(action, ai_decision, location)
            
            # Execute based on action type
            if action == 'investigation':
                self.execute_investigation(location, ai_understanding)
            elif action == 'obstacle_removal':
                self.execute_obstacle_removal(location, ai_understanding)
            elif action == 'reroute':
                self.execute_reroute(location, ai_understanding)
            elif action == 'monitor':
                self.execute_monitor(location, ai_understanding)
            else:
                logger.warning(f"Unknown action: {action}")
            
        except Exception as e:
            logger.error(f"Error processing command: {e}")
    
    def understand_command_with_ai(self, action: str, ai_decision: str, location: Dict) -> str:
        """Use AI to understand and interpret the command"""
        try:
            prompt = f"""
            You are Helper Robot {self.robot_id}. You received a command to perform: {action}
            AI Decision Context: {ai_decision}
            Target Location: {location}
            
            Interpret this command and explain:
            1. What you need to do
            2. How to approach the task
            3. What to watch out for
            4. How to report back
            
            Keep response practical and under 100 words.
            """
            
            response = requests.post(
                f"{self.ollama_host}/api/generate",
                json={
                    "model": self.ollama_model,
                    "prompt": prompt,
                    "stream": False,
                    "options": {
                        "temperature": 0.6,
                        "top_p": 0.8,
                        "max_tokens": 150
                    }
                },
                timeout=20
            )
            
            if response.status_code == 200:
                result = response.json()
                understanding = result.get('response', '').strip()
                logger.info(f"Command AI understanding: {understanding}")
                return understanding
            else:
                return f"Executing {action} at {location} as commanded"
                
        except Exception as e:
            logger.error(f"Command AI understanding failed: {e}")
            return f"Standard execution of {action} command"
    
    def execute_investigation(self, location: Dict, ai_understanding: str):
        """Execute investigation task"""
        logger.info(f"Executing investigation at {location} - {ai_understanding}")
        # Simulate investigation
        time.sleep(2)
        self.report_status("investigation_complete", location)
    
    def execute_obstacle_removal(self, location: Dict, ai_understanding: str):
        """Execute obstacle removal task"""
        logger.info(f"Executing obstacle removal at {location} - {ai_understanding}")
        # Simulate obstacle removal
        time.sleep(3)
        self.report_status("obstacle_removed", location)
    
    def execute_reroute(self, location: Dict, ai_understanding: str):
        """Execute reroute task"""
        logger.info(f"Executing reroute at {location} - {ai_understanding}")
        # Simulate reroute
        time.sleep(1)
        self.report_status("reroute_complete", location)
    
    def execute_monitor(self, location: Dict, ai_understanding: str):
        """Execute monitoring task"""
        logger.info(f"Executing monitoring at {location} - {ai_understanding}")
        # Simulate monitoring
        time.sleep(1)
        self.report_status("monitoring_active", location)
    
    def execute_movement(self, direction: str, distance: float, speed: float) -> Dict[str, Any]:
        """Execute movement command"""
        logger.info(f"Moving {direction} for {distance}m at {speed} speed")
        # Simulate movement
        time.sleep(1)
        return {"status": "completed", "direction": direction, "distance": distance}
    
    def execute_task_plan(self, plan: Dict[str, Any]) -> Dict[str, Any]:
        """Execute task according to AI plan"""
        logger.info(f"Executing task plan: {plan.get('steps', [])}")
        # Simulate task execution
        time.sleep(2)
        return {"status": "completed", "plan_executed": True}
    
    def report_status(self, status: str, location: Dict):
        """Report status back to controller"""
        status_message = {
            "robot_id": self.robot_id,
            "status": status,
            "location": location,
            "timestamp": datetime.now().isoformat(),
            "ai_model": self.ollama_model
        }
        
        topic = f"robots/{self.robot_id}/status"
        self.mqtt_client.publish(topic, json.dumps(status_message), qos=1)
        logger.info(f"Reported status: {status}")
    
    def start(self):
        """Start the helper robot AI service"""
        try:
            logger.info("Starting Helper Robot AI Service...")
            logger.info(f"Using Ollama AI at {self.ollama_host} with model {self.ollama_model}")
            logger.info(f"Flask service on port {self.flask_port}")
            
            # Start Flask in a separate thread
            flask_thread = threading.Thread(
                target=lambda: self.flask_app.run(host='0.0.0.0', port=self.flask_port, debug=False)
            )
            flask_thread.daemon = True
            flask_thread.start()
            
            # Keep MQTT running
            while True:
                time.sleep(1)
                
        except KeyboardInterrupt:
            logger.info("Helper Robot AI stopped by user")
        except Exception as e:
            logger.error(f"Helper Robot AI error: {e}")

if __name__ == "__main__":
    helper_ai = HelperRobotAIService()
    try:
        helper_ai.start()
    except KeyboardInterrupt:
        logger.info("Helper Robot AI stopped by user")
    except Exception as e:
        logger.error(f"Helper Robot AI error: {e}")

