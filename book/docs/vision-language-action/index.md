import Chatbot from '@site/src/components/Chatbot';

---
title: Vision-Language-Action Systems
sidebar_position: 1
---

# Vision-Language-Action Systems

Vision-Language-Action (VLA) systems represent the integration of visual perception, natural language understanding, and robotic action execution. This chapter explores how modern AI models like GPT, Whisper, and vision transformers can be combined with robotic systems to create intelligent agents that understand human commands and execute complex tasks.

## Learning Outcomes

By the end of this chapter, you will:
- Understand the architecture of Vision-Language-Action systems
- Implement speech-to-text using Whisper for voice commands
- Create action planning with GPT for robotic tasks
- Build action graphs for complex multi-step operations
- Integrate vision models for scene understanding
- Design perception-action loops for closed-loop control
- Deploy VLA systems using ROS 2

## VLA System Architecture

Vision-Language-Action systems integrate three key components in a closed-loop architecture:

```
"O"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"    "O"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"    "O"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?
",    Vision       ",    ",   Language      ",    ",     Action      ,
",                 ",    ",                 ",    ",                 ,
", ? Camera Input  ","?,"?"?"?-?," ? Text/NLP      ","?,"?"?"?-?," ? Robot Control ,
", ? Object Detect ",    ", ? Command PARS  ",    ", ? Trajectory    ,
", ? Scene Understanding",  ", ? Planning    ",    ", ? Task Execution",
""?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?~    ""?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?~    ""?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?
         ?                       ?                      ?
         ",                       ",                      ,
         ""?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?'?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"
                           Feedback
```

## Speech Recognition with Whisper

### Real-time Speech-to-Text Processing

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
import whisper
import numpy as np
import torch
import pyaudio
import wave
import threading
import queue

class WhisperSpeechToText(Node):
    def __init__(self):
        super().__init__('whisper_speech_to_text')

        # Publisher for recognized text
        self.text_pub = self.create_publisher(String, 'recognized_text', 10)

        # Audio parameters
        self.rate = 16000
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.record_seconds = 5

        # Initialize Whisper model
        self.get_logger().info("Loading Whisper model...")
        self.model = whisper.load_model("base")  # Use "tiny" for faster inference on edge
        self.get_logger().info("Whisper model loaded")

        # Audio streaming
        self.audio_queue = queue.Queue()
        self.recording = False
        self.audio_thread = None

        # Create timer to process audio
        self.process_timer = self.create_timer(0.1, self.process_audio)

    def start_recording(self):
        """Start audio recording in a separate thread"""
        self.recording = True
        self.audio_thread = threading.Thread(target=self.record_audio)
        self.audio_thread.start()

    def stop_recording(self):
        """Stop audio recording"""
        self.recording = False
        if self.audio_thread:
            self.audio_thread.join()

    def record_audio(self):
        """Record audio from microphone"""
        p = pyaudio.PyAudio()

        stream = p.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        while self.recording:
            data = stream.read(self.chunk)
            self.audio_queue.put(data)

        stream.stop_stream()
        stream.close()
        p.terminate()

    def process_audio(self):
        """Process audio chunks and run transcription"""
        if not self.audio_queue.empty():
            audio_data = []

            # Collect all available audio chunks
            while not self.audio_queue.empty():
                chunk = self.audio_queue.get()
                audio_data.append(chunk)

            if audio_data:
                # Convert to numpy array
                audio_np = np.frombuffer(b''.join(audio_data), dtype=np.int16)
                audio_float = audio_np.astype(np.float32) / 32768.0

                # Transcribe with Whisper
                result = self.model.transcribe(audio_float)

                if result["text"].strip():
                    # Publish recognized text
                    text_msg = String()
                    text_msg.data = result["text"].strip()
                    self.text_pub.publish(text_msg)
                    self.get_logger().info(f"Recognized: {text_msg.data}")

class VoiceCommandProcessor(Node):
    def __init__(self):
        super().__init__('voice_command_processor')

        # Subscribe to recognized text
        self.text_sub = self.create_subscription(
            String,
            'recognized_text',
            self.text_callback,
            10
        )

        # Publisher for processed commands
        self.command_pub = self.create_publisher(String, 'robot_command', 10)

        # Initialize Whisper STT
        self.stt_node = WhisperSpeechToText()

    def text_callback(self, msg):
        """Process recognized text and extract robot commands"""
        text = msg.data.lower()

        # Simple command recognition (in practice, use more sophisticated NLP)
        if "move forward" in text:
            command = "FORWARD"
        elif "move backward" in text:
            command = "BACKWARD"
        elif "turn left" in text:
            command = "LEFT"
        elif "turn right" in text:
            command = "RIGHT"
        elif "pick up" in text or "grasp" in text:
            command = "GRASP"
        elif "place down" in text or "release" in text:
            command = "RELEASE"
        else:
            command = "UNKNOWN"

        if command != "UNKNOWN":
            cmd_msg = String()
            cmd_msg.data = command
            self.command_pub.publish(cmd_msg)
            self.get_logger().info(f"Command: {command}")

def main():
    rclpy.init()
    node = VoiceCommandProcessor()

    try:
        # Start audio recording
        node.stt_node.start_recording()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stt_node.stop_recording()
    finally:
        node.stt_node.stop_recording()
        node.destroy_node()
        rclpy.shutdown()
```

## Action Planning with GPT

### High-Level Task Planning Using Large Language Models

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from typing import List, Dict
import openai
import json
import time

class GPTPlanner(Node):
    def __init__(self):
        super().__init__('gpt_planner')

        # Subscriber for high-level commands
        self.command_sub = self.create_subscription(
            String,
            'high_level_command',
            self.command_callback,
            10
        )

        # Publisher for action plans
        self.action_pub = self.create_publisher(String, 'action_plan', 10)
        self.waypoint_pub = self.create_publisher(Pose, 'waypoints', 10)

        # Initialize knowledge base
        self.known_objects = {
            "cup": {"position": [1.0, 0.5, 0.0], "properties": {"graspable": True}},
            "box": {"position": [2.0, -1.0, 0.0], "properties": {"graspable": True}},
            "table": {"position": [1.5, 0.0, 0.0], "properties": {"surface": True}},
            "charger": {"position": [0.0, 0.0, 0.0], "properties": {"recharge_station": True}}
        }

        self.known_rooms = {
            "kitchen": ["cup", "table"],
            "living_room": ["box"],
            "bedroom": ["charger"]
        }

        # Set OpenAI API key (in practice, use secure configuration)
        # openai.api_key = "your-api-key-here"

        self.get_logger().info("GPT Planner initialized")

    def command_callback(self, msg):
        """Process high-level command and generate action plan"""
        command = msg.data
        self.get_logger().info(f"Processing command: {command}")

        # Generate detailed action plan using GPT
        plan = self.generate_action_plan(command)

        if plan:
            # Publish action plan
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.action_pub.publish(plan_msg)

            self.execute_plan(plan)

    def generate_action_plan(self, command: str) -> List[Dict]:
        """Generate action plan using GPT"""
        prompt = f"""
        You are a robot task planner. Given the command "{command}", create a detailed action plan.

        Known objects and positions:
        {json.dumps(self.known_objects, indent=2)}

        Known rooms and their objects:
        {json.dumps(self.known_rooms, indent=2)}

        Return a JSON list of actions with these types:
        - NAVIGATE: Move to a location
        - DETECT: Look for objects
        - GRASP: Pick up an object
        - PLACE: Put down an object
        - SPEAK: Say something to user

        Each action should include:
        - type: action type
        - target: object or location
        - position: [x, y, z] coordinates if applicable
        - description: human-readable explanation

        Example response format:
        [
            {{
                "type": "NAVIGATE",
                "target": "table",
                "position": [1.0, 0.5, 0.0],
                "description": "Move to the table"
            }},
            {{
                "type": "GRASP",
                "target": "cup",
                "position": [1.5, 0.0, 0.8],
                "description": "Pick up the cup"
            }}
        ]
        """

        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.3,
                max_tokens=500
            )

            plan_text = response.choices[0].message['content'].strip()

            # Extract JSON from response (handle possible markdown formatting)
            if "```json" in plan_text:
                plan_text = plan_text.split("```json")[1].split("```")[0]
            elif "```" in plan_text:
                plan_text = plan_text.split("```")[1].split("```")[0]

            plan = json.loads(plan_text)
            self.get_logger().info(f"Generated plan: {plan}")
            return plan

        except Exception as e:
            self.get_logger().error(f"Error generating plan: {e}")
            return []

    def execute_plan(self, plan: List[Dict]):
        """Execute the generated action plan"""
        for action in plan:
            self.get_logger().info(f"Executing: {action['description']}")

            if action['type'] == 'NAVIGATE':
                self.navigate_to(action['position'])
            elif action['type'] == 'GRASP':
                self.grasp_object(action['target'])
            elif action['type'] == 'PLACE':
                self.place_object(action['position'])
            elif action['type'] == 'SPEAK':
                self.speak(action['target'])

            time.sleep(1)  # Wait between actions

    def navigate_to(self, position: List[float]):
        """Navigate to specified position"""
        pose_msg = Pose()
        pose_msg.position.x = position[0]
        pose_msg.position.y = position[1]
        pose_msg.position.z = position[2]
        # Set orientation (facing direction)
        pose_msg.orientation.w = 1.0  # No rotation

        self.waypoint_pub.publish(pose_msg)
        self.get_logger().info(f"Navigating to: {position}")

    def grasp_object(self, target: str):
        """Grasp specified object"""
        self.get_logger().info(f"Grasping: {target}")
        # In practice, this would call manipulation service

    def place_object(self, position: List[float]):
        """Place object at specified position"""
        self.get_logger().info(f"Placing at: {position}")
        # In practice, this would call manipulation service

    def speak(self, text: str):
        """Speak text to user"""
        self.get_logger().info(f"Speaking: {text}")
        # In practice, this would use text-to-speech service

def main():
    rclpy.init()
    planner = GPTPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()
```

## Vision Processing Integration

### Real-time Object Detection and Scene Understanding

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import torch
import torchvision.transforms as T
from transformers import DetrForObjectDetection, DetrImageProcessor
import numpy as np

class VisionProcessor(Node):
    def __init__(self):
        super().__init__('vision_processor')

        # Image subscription
        self.image_sub = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )

        # Detection publisher
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/object_detections',
            10
        )

        # Initialize vision model
        self.get_logger().info("Loading DETR vision model...")
        self.processor = DetrImageProcessor.from_pretrained("facebook/detr-resnet-50")
        self.model = DetrForObjectDetection.from_pretrained("facebook/detr-resnet-50")

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        # Confidence threshold
        self.confidence_threshold = 0.7

    def image_callback(self, msg):
        """Process incoming image and detect objects"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Process image with DETR
            inputs = self.processor(images=cv_image, return_tensors="pt")

            with torch.no_grad():
                outputs = self.model(**inputs)

            # Convert outputs to detection format
            target_sizes = torch.tensor([cv_image.shape[:2]])
            results = self.processor.post_process_object_detection(
                outputs,
                target_sizes=target_sizes,
                threshold=self.confidence_threshold
            )[0]

            # Create Detection2DArray message
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header

            # Process detections
            for score, label, box in zip(
                results["scores"],
                results["labels"],
                results["boxes"]
            ):
                detection = Detection2D()
                detection.header = msg.header

                # Get object class
                object_class = self.model.config.id2label[int(label)]

                # Create hypothesis
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = object_class
                hypothesis.hypothesis.score = float(score)

                detection.results.append(hypothesis)

                # Convert bounding box to center + size
                box = box.cpu().numpy()
                x_min, y_min, x_max, y_max = box
                center_x = (x_min + x_max) / 2.0
                center_y = (y_min + y_max) / 2.0
                width = x_max - x_min
                height = y_max - y_min

                # Set center point
                center_point = Point()
                center_point.x = float(center_x)
                center_point.y = float(center_y)
                center_point.z = 1.0  # Placeholder depth

                detection.bbox.center = center_point
                detection.bbox.size_x = float(width)
                detection.bbox.size_y = float(height)

                # Add to detections
                detections_msg.detections.append(detection)

            # Publish detections
            self.detection_pub.publish(detections_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

class SceneUnderstandingNode(Node):
    def __init__(self):
        super().__init__('scene_understanding')

        # Subscribe to detections
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/object_detections',
            self.detections_callback,
            10
        )

        # Publisher for scene description
        self.scene_pub = self.create_publisher(String, '/scene_description', 10)

        # Maintain scene knowledge
        self.scene_objects = {}
        self.update_timer = self.create_timer(2.0, self.update_scene_description)

    def detections_callback(self, msg):
        """Update scene knowledge with new detections"""
        for detection in msg.detections:
            if detection.results:
                obj_class = detection.results[0].hypothesis.class_id
                confidence = detection.results[0].hypothesis.score
                center = detection.bbox.center

                if confidence > 0.7:  # Confidence threshold
                    self.scene_objects[obj_class] = {
                        'position': [center.x, center.y, center.z],
                        'confidence': confidence
                    }

    def update_scene_description(self):
        """Publish current scene description"""
        if self.scene_objects:
            scene_desc = {
                'objects': self.scene_objects,
                'timestamp': self.get_clock().now().to_msg()
            }

            desc_msg = String()
            desc_msg.data = json.dumps(scene_desc)
            self.scene_pub.publish(desc_msg)
            self.get_logger().info(f"Scene: {self.scene_objects}")

def main():
    rclpy.init()
    vision_node = VisionProcessor()
    scene_node = SceneUnderstandingNode()

    rclpy.spin(vision_node)
    vision_node.destroy_node()
    scene_node.destroy_node()
    rclpy.shutdown()
```

## Action Graph Implementation

### State Machine for Complex Task Execution

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from action_msgs.msg import GoalStatus
from enum import Enum
import time
from typing import Dict, Any, Callable

class TaskState(Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    DETECTING = "detecting"
    MANIPULATING = "manipulating"
    COMPLETED = "completed"
    FAILED = "failed"

class ActionGraph:
    def __init__(self):
        self.state = TaskState.IDLE
        self.graph = {}
        self.current_node = None
        self.parameters = {}

    def add_node(self, name: str, action_func: Callable,
                 transitions: Dict[str, str], condition_func=None):
        """Add a node to the action graph"""
        self.graph[name] = {
            'action': action_func,
            'transitions': transitions,
            'condition': condition_func
        }

    def set_parameter(self, key: str, value: Any):
        """Set a parameter for the action graph"""
        self.parameters[key] = value

    def get_parameter(self, key: str, default=None):
        """Get a parameter from the action graph"""
        return self.parameters.get(key, default)

    def execute_graph(self, start_node: str):
        """Execute the action graph starting from a node"""
        self.current_node = start_node
        self.state = TaskState.IDLE

        while self.state not in [TaskState.COMPLETED, TaskState.FAILED]:
            if self.current_node not in self.graph:
                self.state = TaskState.FAILED
                return False

            node = self.graph[self.current_node]

            # Check pre-condition
            if node['condition'] and not node['condition']():
                self.state = TaskState.FAILED
                return False

            # Execute action
            self.state = TaskState(node['action'].__name__.replace('action_', '').upper())
            result = node['action']()

            if not result:
                self.state = TaskState.FAILED
                return False

            # Determine next node based on result
            self.current_node = node['transitions'].get(result, 'failed')

            if self.current_node == 'completed':
                self.state = TaskState.COMPLETED
                return True
            elif self.current_node == 'failed':
                self.state = TaskState.FAILED
                return False

        return True

class ActionGraphExecutor(Node):
    def __init__(self):
        super().__init__('action_graph_executor')

        # Subscribers
        self.command_sub = self.create_subscription(
            String,
            'action_command',
            self.command_callback,
            10
        )

        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/object_detections',
            self.detection_callback,
            10
        )

        # Publishers
        self.status_pub = self.create_publisher(String, 'action_status', 10)
        self.completed_pub = self.create_publisher(Bool, 'task_completed', 10)

        # Initialize action graph
        self.action_graph = ActionGraph()
        self.setup_action_graph()

        self.current_detections = []
        self.action_lock = False

    def setup_action_graph(self):
        """Setup the action graph for pick-and-place task"""
        # Navigate to object
        self.action_graph.add_node(
            'navigate_to_object',
            self.action_navigate_to_object,
            transitions={'success': 'detect_object', 'failed': 'failed'}
        )

        # Detect object
        self.action_graph.add_node(
            'detect_object',
            self.action_detect_object,
            transitions={'found': 'grasp_object', 'not_found': 'failed', 'failed': 'failed'}
        )

        # Grasp object
        self.action_graph.add_node(
            'grasp_object',
            self.action_grasp_object,
            transitions={'success': 'navigate_to_target', 'failed': 'failed'}
        )

        # Navigate to target
        self.action_graph.add_node(
            'navigate_to_target',
            self.action_navigate_to_target,
            transitions={'success': 'place_object', 'failed': 'failed'}
        )

        # Place object
        self.action_graph.add_node(
            'place_object',
            self.action_place_object,
            transitions={'success': 'completed', 'failed': 'failed'}
        )

    def command_callback(self, msg):
        """Process action command"""
        if self.action_lock:
            return  # Already executing

        command = msg.data
        self.action_lock = True
        self.get_logger().info(f"Executing action graph for: {command}")

        # Set target object based on command
        if "cup" in command.lower():
            self.action_graph.set_parameter('target_object', 'cup')
        elif "box" in command.lower():
            self.action_graph.set_parameter('target_object', 'box')

        # Execute graph
        success = self.action_graph.execute_graph('navigate_to_object')

        # Publish completion
        completed_msg = Bool()
        completed_msg.data = success
        self.completed_pub.publish(completed_msg)

        self.action_lock = False

        # Publish status
        status_msg = String()
        status_msg.data = "completed" if success else "failed"
        self.status_pub.publish(status_msg)

    def detection_callback(self, msg):
        """Update current detections"""
        self.current_detections = msg.detections

    def action_navigate_to_object(self):
        """Navigate to the target object location"""
        target_obj = self.action_graph.get_parameter('target_object', 'unknown')
        self.get_logger().info(f"Navigating to {target_obj}")

        # In practice, this would call navigation service
        # Here we simulate with a delay
        time.sleep(2)

        # Publish navigation goal (simplified)
        pose_msg = Pose()
        # Calculate approximate position based on detections
        for detection in self.current_detections:
            if detection.results and detection.results[0].hypothesis.class_id == target_obj:
                pose_msg.position.x = detection.bbox.center.x / 100.0  # Scale appropriately
                pose_msg.position.y = detection.bbox.center.y / 100.0
                break

        return 'success'

    def action_detect_object(self):
        """Detect the target object"""
        target_obj = self.action_graph.get_parameter('target_object', 'unknown')
        self.get_logger().info(f"Detecting {target_obj}")

        # Check if target object is detected
        for detection in self.current_detections:
            if (detection.results and
                detection.results[0].hypothesis.class_id == target_obj):
                confidence = detection.results[0].hypothesis.score
                if confidence > 0.7:  # Confidence threshold
                    self.action_graph.set_parameter('object_pose', detection.bbox.center)
                    return 'found'

        return 'not_found'

    def action_grasp_object(self):
        """Grasp the detected object"""
        self.get_logger().info("Grasping object")

        # In practice, this would call manipulation service
        time.sleep(3)  # Simulate grasp time

        # Check if grasp was successful (simplified)
        success = True  # In practice, check feedback from gripper
        return 'success' if success else 'failed'

    def action_navigate_to_target(self):
        """Navigate to placement target"""
        self.get_logger().info("Navigating to placement target")

        # In practice, navigate to predefined target location
        time.sleep(2)
        return 'success'

    def action_place_object(self):
        """Place the object at target location"""
        self.get_logger().info("Placing object")

        # In practice, call placement service
        time.sleep(2)
        return 'success'

def main():
    rclpy.init()
    executor = ActionGraphExecutor()
    rclpy.spin(executor)
    executor.destroy_node()
    rclpy.shutdown()
```

## Integration Example: Full VLA System

### Complete Vision-Language-Action Pipeline

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import threading
import queue

class VisionLanguageActionSystem(Node):
    def __init__(self):
        super().__init__('vla_system')

        # Publishers and subscribers
        self.voice_cmd_pub = self.create_publisher(String, 'voice_command', 10)
        self.high_level_cmd_pub = self.create_publisher(String, 'high_level_command', 10)
        self.action_cmd_pub = self.create_publisher(String, 'action_command', 10)

        # Subscribe to system status
        self.task_status_sub = self.create_subscription(
            String, 'action_status', self.status_callback, 10
        )

        # Initialize components
        self.whisper_node = WhisperSpeechToText()
        self.gpt_planner = GPTPlanner()
        self.vision_processor = VisionProcessor()
        self.action_executor = ActionGraphExecutor()

        # System state
        self.system_active = True

        # Start processing threads
        self.voice_thread = threading.Thread(target=self.run_voice_system)
        self.planning_thread = threading.Thread(target=self.run_planning_system)

        self.get_logger().info("Vision-Language-Action System initialized")

    def run_voice_system(self):
        """Run voice recognition system"""
        rclpy.spin(self.whisper_node)

    def run_planning_system(self):
        """Run planning and execution system"""
        rclpy.spin(self.gpt_planner)
        rclpy.spin(self.action_executor)

    def status_callback(self, msg):
        """Handle system status updates"""
        self.get_logger().info(f"Task status: {msg.data}")

        if msg.data == "completed":
            # Announce completion
            completion_msg = String()
            completion_msg.data = "Task completed successfully"
            self.voice_cmd_pub.publish(completion_msg)

    def start_system(self):
        """Start the VLA system"""
        self.whisper_node.start_recording()

        # Start processing threads
        self.voice_thread.start()
        self.planning_thread.start()

        self.get_logger().info("VLA system started")

    def stop_system(self):
        """Stop the VLA system"""
        self.system_active = False
        self.whisper_node.stop_recording()

        # Wait for threads to finish
        if self.voice_thread.is_alive():
            self.voice_thread.join()
        if self.planning_thread.is_alive():
            self.planning_thread.join()

def main():
    rclpy.init()
    vla_system = VisionLanguageActionSystem()

    try:
        vla_system.start_system()
        rclpy.spin(vla_system)
    except KeyboardInterrupt:
        vla_system.get_logger().info("Shutting down VLA system...")
    finally:
        vla_system.stop_system()
        vla_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Closed-Loop Perception-Action Systems

### Continuous Monitoring and Correction

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String, Bool
import numpy as np
import cv2
from cv_bridge import CvBridge

class ClosedLoopController(Node):
    def __init__(self):
        super().__init__('closed_loop_controller')

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_sub = self.create_subscription(Image, '/camera/rgb/image_raw',
                                                 self.image_callback, 10)
        self.goal_sub = self.create_subscription(Pose, '/navigation_goal',
                                                self.goal_callback, 10)

        self.bridge = CvBridge()
        self.current_goal = None
        self.current_image = None
        self.object_center = None
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # Control parameters
        self.linear_k = 0.5
        self.angular_k = 1.0
        self.goal_tolerance = 0.1

    def goal_callback(self, msg):
        """Set navigation goal"""
        self.current_goal = msg

    def image_callback(self, msg):
        """Process camera image for visual servoing"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Simple color-based object detection for demonstration
            hsv = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2HSV)

            # Detect red objects (as example target)
            lower_red = np.array([0, 120, 70])
            upper_red = np.array([10, 255, 255])
            mask1 = cv2.inRange(hsv, lower_red, upper_red)

            lower_red = np.array([170, 120, 70])
            upper_red = np.array([180, 255, 255])
            mask2 = cv2.inRange(hsv, lower_red, upper_red)

            mask = mask1 + mask2
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > 100:  # Minimum area threshold
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        self.object_center = (cx, cy)

                        # Draw center on image for visualization
                        cv2.circle(self.current_image, (cx, cy), 5, (0, 255, 0), -1)
                        # Convert back to ROS image and publish for visualization
                        # (would need image transport publisher in real implementation)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def control_loop(self):
        """Main control loop for closed-loop system"""
        if self.object_center is not None and self.current_goal is not None:
            # Calculate error to target
            img_center_x = self.current_image.shape[1] / 2
            error_x = self.object_center[0] - img_center_x

            # Simple proportional control
            cmd = Twist()
            cmd.linear.x = self.linear_k * min(0.5, 1.0 - abs(error_x) / img_center_x)  # Forward speed based on alignment
            cmd.angular.z = -self.angular_k * error_x / img_center_x  # Turn to center

            self.cmd_vel_pub.publish(cmd)
        elif self.current_goal is not None:
            # Navigate to goal position if object not detected
            cmd = Twist()
            cmd.linear.x = 0.2  # Move forward
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)

def main():
    rclpy.init()
    controller = ClosedLoopController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()
```

## Best Practices

1. **Fallback Strategies**: Always have backup plans when AI models fail
2. **Safety First**: Implement safety checks and limits for all actions
3. **Robust Communication**: Handle message timeouts and retransmissions
4. **Modular Design**: Keep vision, language, and action components separate
5. **Performance Optimization**: Use appropriate models for computational constraints
6. **Continuous Learning**: Implement systems to improve from experience

## Quiz

1. What does VLA stand for in robotics?
   A) Visual Localization and Action
   B) Vision-Language-Action
   C) Vector Learning Algorithm
   D) Vision-Laser-Audio

2. Which model is commonly used for speech recognition in VLA systems?
   A) GPT
   B) Whisper
   C) DETR
   D) ResNet

3. What is the primary purpose of action graphs in VLA systems?
   A) Image processing
   B) Task planning and execution flow
   C) Speech synthesis
   D) Sensor fusion

Answers: 1-B, 2-B, 3-B

## Summary

Vision-Language-Action systems integrate visual perception, natural language understanding, and robotic action execution to create intelligent agents capable of understanding and responding to human commands. These systems require careful integration of multiple AI models and control systems, with proper fallback mechanisms and safety considerations. The closed-loop nature of these systems enables continuous perception-action cycles that allow robots to adapt to dynamic environments and complete complex tasks through high-level human instructions.

<Chatbot />