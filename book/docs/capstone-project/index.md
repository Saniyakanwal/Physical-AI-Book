---
title: "Capstone: Full Humanoid Voice-to-Action Project"
sidebar_position: 1
---

# Capstone: Full Humanoid Voice-to-Action Project

The capstone project integrates all concepts from previous chapters to create a complete autonomous humanoid robot system. This comprehensive project combines voice interaction, vision processing, path planning, humanoid control, and reinforcement learning into a single functional system that can understand voice commands and perform tasks autonomously.

## Learning Outcomes

By the end of this chapter, you will:
- Integrate all previously learned concepts into a complete system
- Implement end-to-end voice command processing for humanoid robots
- Design and build a complete software architecture using ROS 2
- Deploy reinforcement learning agents to real hardware or simulation
- Create a perception-action loop for autonomous task execution
- Test and validate the complete system in simulation and/or hardware

## System Architecture

The complete system architecture integrates all components studied in previous chapters:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           HUMANOID ROBOT SYSTEM                         │
├─────────────────────────────────────────────────────────────────────────┤
│ ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐     │
│ │   Voice     │  │   Vision    │  │   Action    │  │   Control   │     │
│ │ Recognition │  │ Processing  │  │ Planning    │  │ System      │     │
│ │ • Whisper   │  │ • DETR      │  │ • GPT       │  │ • Balance   │     │
│ │ • STT       │  │ • YOLOv8    │  │ • Action    │  │ • Walking   │     │
│ │ • ASR       │  │ • Segmentation│  │ • Graphs   │  │ • IK/PD     │     │
│ └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘     │
│         │               │                │                │            │
│         └───────────────┼────────────────┼────────────────┘            │
│                         │                │                             │
│         ┌───────────────▼────────────────▼─────────────────────────┐   │
│         │                    TASK EXECUTION                       │   │
│         │                    PLANNER                              │   │
│         │ • State Machine                                         │   │
│         │ • Execution Manager                                     │   │
│         │ • Error Recovery                                        │   │
│         └─────────────────────────────────────────────────────────┘   │
│                         │                                             │
│         ┌───────────────▼─────────────────────────────────────────┐   │
│         │                    HUMANOID                             │   │
│         │                    SIMULATION                         │   │
│         │ • Gazebo / Isaac Sim                                  │   │
│         │ • Physics Simulation                                  │   │
│         │ • Sensor Simulation                                   │   │
│         └─────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
```

## Complete System Implementation

### Main System Orchestrator

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Pose, Twist
from builtin_interfaces.msg import Time
import threading
import json
import time
from typing import Dict, Any, Optional

class HumanoidSystemOrchestrator(Node):
    def __init__(self):
        super().__init__('humanoid_system_orchestrator')

        # Publishers
        self.voice_cmd_pub = self.create_publisher(String, 'voice_command', 10)
        self.navigation_goal_pub = self.create_publisher(Pose, 'navigation_goal', 10)
        self.manipulation_cmd_pub = self.create_publisher(String, 'manipulation_command', 10)
        self.system_status_pub = self.create_publisher(String, 'system_status', 10)

        # Subscribers
        self.voice_recognition_sub = self.create_subscription(
            String, 'recognized_text', self.voice_callback, 10
        )
        self.scene_description_sub = self.create_subscription(
            String, 'scene_description', self.scene_callback, 10
        )
        self.task_status_sub = self.create_subscription(
            String, 'task_status', self.task_status_callback, 10
        )
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )

        # System state
        self.current_task = None
        self.system_state = "IDLE"  # IDLE, PROCESSING, EXECUTING, ERROR
        self.scene_knowledge = {}
        self.robot_state = {}
        self.voice_commands = {
            "move forward": self.handle_move_forward,
            "move backward": self.handle_move_backward,
            "turn left": self.handle_turn_left,
            "turn right": self.handle_turn_right,
            "pick up": self.handle_pick_up,
            "pick up object": self.handle_pick_up,
            "place down": self.handle_place_down,
            "dance": self.handle_dance,
            "walk to kitchen": self.handle_goto_kitchen,
            "bring me the cup": self.handle_fetch_cup,
            "find the red ball": self.handle_find_red_ball
        }

        # System timers
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.health_check_timer = self.create_timer(5.0, self.health_check)

        # Initialize subsystems
        self.initialize_subsystems()

        self.get_logger().info("Humanoid System Orchestrator initialized")

    def initialize_subsystems(self):
        """Initialize all subsystems"""
        self.get_logger().info("Initializing humanoid subsystems...")

        # In a real system, this would start all nodes
        # For this example, we just log the initialization
        self.get_logger().info("✓ Voice recognition subsystem")
        self.get_logger().info("✓ Vision processing subsystem")
        self.get_logger().info("✓ Task planning subsystem")
        self.get_logger().info("✓ Motion control subsystem")

        self.system_state = "READY"

    def voice_callback(self, msg):
        """Handle recognized voice commands"""
        command = msg.data.lower().strip()
        self.get_logger().info(f"Received voice command: '{command}'")

        # Update system state
        self.system_state = "PROCESSING"

        # Process command in separate thread to avoid blocking
        threading.Thread(target=self.process_command, args=(command,)).start()

    def process_command(self, command: str):
        """Process voice command and execute appropriate action"""
        try:
            # Look for matching command
            matched = False
            for cmd_key, handler in self.voice_commands.items():
                if cmd_key in command:
                    self.get_logger().info(f"Executing command: {cmd_key}")
                    handler(command)
                    matched = True
                    break

            if not matched:
                # Use GPT for more complex command interpretation
                self.interpret_complex_command(command)

        except Exception as e:
            self.get_logger().error(f"Error processing command: {e}")
            self.system_state = "ERROR"
            self.speak_error("Sorry, I couldn't process that command.")

    def interpret_complex_command(self, command: str):
        """Use GPT to interpret complex commands"""
        # This would call the GPT-based command interpreter
        # For simulation, we'll use simple keyword matching
        if "cup" in command:
            self.handle_fetch_cup(command)
        elif "ball" in command:
            self.handle_find_red_ball(command)
        else:
            self.speak_error("I don't understand that command.")

    def scene_callback(self, msg):
        """Update scene knowledge"""
        try:
            scene_data = json.loads(msg.data)
            self.scene_knowledge = scene_data
            self.get_logger().info(f"Updated scene knowledge: {len(scene_data.get('objects', {}))} objects")
        except json.JSONDecodeError:
            self.get_logger().error("Invalid scene description format")

    def task_status_callback(self, msg):
        """Handle task execution status"""
        status = msg.data
        self.get_logger().info(f"Task status: {status}")

        if status == "completed":
            self.system_state = "IDLE"
            self.speak_success("Task completed successfully!")
        elif status.startswith("error"):
            self.system_state = "ERROR"
            self.speak_error(f"Task failed: {status}")
        else:
            self.system_state = status.upper()

    def joint_state_callback(self, msg):
        """Update robot state"""
        self.robot_state = {
            'position': msg.position,
            'velocity': msg.velocity,
            'effort': msg.effort
        }

    def publish_status(self):
        """Publish system status"""
        status_msg = String()
        status_msg.data = f"STATE:{self.system_state}|TASK:{self.current_task or 'None'}"
        self.system_status_pub.publish(status_msg)

    def health_check(self):
        """Perform system health check"""
        # Check if all subsystems are responsive
        self.get_logger().info(f"System health - State: {self.system_state}, Objects: {len(self.scene_knowledge.get('objects', {}))}")

    def speak_success(self, message: str):
        """Speak success message"""
        # In practice, this would publish to text-to-speech node
        self.get_logger().info(f"Speaking: {message}")

    def speak_error(self, message: str):
        """Speak error message"""
        self.get_logger().info(f"Speaking error: {message}")

    # Command handlers
    def handle_move_forward(self, command: str):
        """Handle move forward command"""
        self.current_task = "move_forward"
        twist = Twist()
        twist.linear.x = 0.5  # Move forward at 0.5 m/s
        # Publish to navigation system
        self.get_logger().info("Moving forward")

    def handle_move_backward(self, command: str):
        """Handle move backward command"""
        self.current_task = "move_backward"
        twist = Twist()
        twist.linear.x = -0.5  # Move backward
        self.get_logger().info("Moving backward")

    def handle_turn_left(self, command: str):
        """Handle turn left command"""
        self.current_task = "turn_left"
        twist = Twist()
        twist.angular.z = 0.5  # Turn left
        self.get_logger().info("Turning left")

    def handle_turn_right(self, command: str):
        """Handle turn right command"""
        self.current_task = "turn_right"
        twist = Twist()
        twist.angular.z = -0.5  # Turn right
        self.get_logger().info("Turning right")

    def handle_pick_up(self, command: str):
        """Handle pick up command"""
        self.current_task = "pick_up"
        cmd_msg = String()
        cmd_msg.data = "GRASP_OBJECT"
        self.manipulation_cmd_pub.publish(cmd_msg)
        self.get_logger().info("Attempting to grasp object")

    def handle_place_down(self, command: str):
        """Handle place down command"""
        self.current_task = "place_down"
        cmd_msg = String()
        cmd_msg.data = "RELEASE_OBJECT"
        self.manipulation_cmd_pub.publish(cmd_msg)
        self.get_logger().info("Releasing object")

    def handle_dance(self, command: str):
        """Handle dance command"""
        self.current_task = "dance"
        # Execute pre-programmed dance sequence
        self.execute_dance_sequence()

    def handle_goto_kitchen(self, command: str):
        """Handle go to kitchen command"""
        self.current_task = "goto_kitchen"
        # Navigate to kitchen coordinates
        kitchen_pose = Pose()
        kitchen_pose.position.x = 3.0
        kitchen_pose.position.y = 2.0
        kitchen_pose.position.z = 0.0
        self.navigation_goal_pub.publish(kitchen_pose)
        self.get_logger().info("Navigating to kitchen")

    def handle_fetch_cup(self, command: str):
        """Handle fetch cup command"""
        self.current_task = "fetch_cup"

        # Look for cup in scene
        if 'cup' in self.scene_knowledge.get('objects', {}):
            cup_pos = self.scene_knowledge['objects']['cup']['position']

            # Navigate to cup location
            target_pose = Pose()
            target_pose.position.x = cup_pos[0]
            target_pose.position.y = cup_pos[1]
            target_pose.position.z = 0.0
            self.navigation_goal_pub.publish(target_pose)

            # After reaching, grasp the cup
            time.sleep(3)  # Wait for navigation
            self.handle_pick_up(command)

        else:
            self.speak_error("I can't find the cup.")

    def handle_find_red_ball(self, command: str):
        """Handle find red ball command"""
        self.current_task = "find_red_ball"
        # This would trigger object detection for red ball
        search_cmd = String()
        search_cmd.data = "SEARCH_RED_BALL"
        # Publish search command to vision system

    def execute_dance_sequence(self):
        """Execute pre-programmed dance sequence"""
        self.get_logger().info("Executing dance sequence...")
        # In practice, this would send joint trajectories for dance moves
        dance_commands = [
            ("WAVE_ARMS", 2.0),
            ("TURN_CIRCLE", 4.0),
            ("BOUNCE", 2.0),
        ]

        for dance_cmd, duration in dance_commands:
            cmd_msg = String()
            cmd_msg.data = dance_cmd
            self.manipulation_cmd_pub.publish(cmd_msg)
            time.sleep(duration)

def main():
    rclpy.init()

    # Create orchestrator node
    orchestrator = HumanoidSystemOrchestrator()

    try:
        orchestrator.get_logger().info("Starting humanoid system orchestrator...")
        rclpy.spin(orchestrator)
    except KeyboardInterrupt:
        orchestrator.get_logger().info("Shutting down humanoid system...")
    finally:
        orchestrator.destroy_node()
        rclpy.shutdown()
```

## Reinforcement Learning Integration

### RL Training Environment for Humanoid Tasks

```python
from omni.isaac.gym.tasks.base.rl_task import RLTask
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.torch.maths import torch_rand_float, tensor_clamp
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim

class HumanoidTask(RLTask):
    def __init__(self, name, offset=None):
        # Task parameters
        self._num_envs = 16
        self._env_spacing = 2.5
        self._max_episode_length = 500

        # Observation and action spaces
        self._num_observations = 60  # Joint positions, velocities, IMU data, etc.
        self._num_actions = 12       # Lower body joints for walking

        RLTask.__init__(self, name, offset, self._num_envs, self._env_spacing)

    def set_up_scene(self, scene) -> None:
        # Add ground plane
        self._ground_plane = scene.add_default_ground_plane()

        # Load humanoid robot asset
        assets_root_path = get_assets_root_path()
        robot_path = "/Isaac/Robots/Humanoid/humanoid_instanceable.usd"
        add_reference_to_stage(
            usd_path=assets_root_path + robot_path,
            prim_path=self.default_zero_env_path + "/Humanoid"
        )

        # Add target object to reach for
        sphere_path = "/Isaac/Props/Obstacles/sphere.usd"
        add_reference_to_stage(
            usd_path=assets_root_path + sphere_path,
            prim_path=self.default_zero_env_path + "/Target"
        )

        super().set_up_scene(scene)

        # Create articulation view for humanoid robots
        self._humanoids = ArticulationView(
            prim_paths_expr="/World/envs/.*/Humanoid",
            name="humanoid_view",
            reset_xform_properties=False
        )
        scene.add(self._humanoids)

        # Create target views
        self._targets = ArticulationView(
            prim_paths_expr="/World/envs/.*/Target",
            name="target_view",
            reset_xform_properties=False
        )
        scene.add(self._targets)

    def get_observations(self):
        # Get humanoid states
        humanoid_pos = self._humanoids.get_world_poses(clone=False)[0]
        humanoid_rot = self._humanoids.get_world_poses(clone=False)[1]
        joint_pos = self._humanoids.get_joint_positions(clone=False)
        joint_vel = self._humanoids.get_joint_velocities(clone=False)

        # Get target positions
        target_pos, target_rot = self._targets.get_world_poses(clone=False)

        # Calculate relative positions
        rel_pos = target_pos - humanoid_pos

        # Create observation tensor
        self.obs_buf = torch.cat([
            joint_pos,
            joint_vel,
            rel_pos,
            humanoid_rot
        ], dim=-1)

        return self.obs_buf

    def pre_physics_step(self, actions) -> None:
        reset_envs = self.reset_buf.nonzero(as_tuple=False).squeeze(-1)
        if len(reset_envs) > 0:
            self.reset_idx(reset_envs)

        actions = tensor_clamp(actions, -1.0, 1.0)
        self._actions = actions

        # Apply actions as joint position targets
        env_ids_int32 = torch.arange(
            self._humanoids.count, dtype=torch.int32, device=self._device
        )

        # Map actions to joint targets (simplified)
        current_pos = self._humanoids.get_joint_positions(clone=False)
        new_pos = current_pos + actions * 0.1  # Small incremental changes
        self._humanoids.set_joint_position_targets(new_pos, indices=env_ids_int32)

    def calculate_rewards(self):
        # Get current positions
        humanoid_pos, _ = self._humanoids.get_world_poses(clone=False)
        target_pos, _ = self._targets.get_world_poses(clone=False)

        # Calculate distance to target
        dist_to_target = torch.norm(target_pos - humanoid_pos, p=2, dim=-1)

        # Reward based on proximity to target
        reward = 1.0 / (1.0 + dist_to_target)

        # Additional rewards for stable walking
        # (check joint positions, velocities for natural movement)

        self.rew_buf[:] = reward

    def is_done(self):
        # Check if episode is done
        humanoid_pos, _ = self._humanoids.get_world_poses(clone=False)

        # Reset if humanoid falls (simplified check)
        reset = torch.where(humanoid_pos[:, 2] < 0.5, 1, 0)  # If height < 0.5m
        reset = torch.where(self.progress_buf >= self._max_episode_length, 1, reset)

        self.reset_buf[:] = reset

    def reset_idx(self, env_ids):
        indices = env_ids.to(dtype=torch.int32)

        # Reset humanoid positions
        new_positions = torch_rand_float(-0.5, 0.5, (len(env_ids), 3), device=self._device)
        new_positions[:, 2] = 1.0  # Set height
        self._humanoids.set_world_poses(
            new_positions,
            indices=indices
        )

        # Reset joint positions to default
        default_positions = torch.zeros(
            (len(env_ids), self._humanoids.num_dof),
            device=self._device
        )
        self._humanoids.set_joint_positions(default_positions, indices=indices)

        # Reset targets to random positions around origin
        target_positions = torch_rand_float(
            -2.0, 2.0, (len(env_ids), 3), device=self._device
        )
        target_positions[:, 2] = 0.0  # Ground level
        self._targets.set_world_poses(target_positions, indices=indices)

        # Reset buffers
        self.reset_buf[env_ids] = 0
        self.progress_buf[env_ids] = 0

    def post_reset(self):
        self._humanoids.set_solver_velocity_iteration_count(4)
        self._humanoids.set_solver_position_iteration_count(4)

        # Initialize buffers
        self._actions = torch.zeros(
            (self._num_envs, self._num_actions),
            dtype=torch.float32,
            device=self._device,
        )

        self.progress_buf = torch.zeros(
            self._num_envs, dtype=torch.long, device=self._device
        )
        self.reset_buf = torch.ones(
            self._num_envs, dtype=torch.long, device=self._device
        )

# Simple neural network for humanoid control
class HumanoidActorCritic(nn.Module):
    def __init__(self, obs_dim, action_dim, hidden_dim=256):
        super().__init__()

        self.actor = nn.Sequential(
            nn.Linear(obs_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim),
            nn.Tanh()
        )

        self.critic = nn.Sequential(
            nn.Linear(obs_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1)
        )

    def forward(self, x):
        action = self.actor(x)
        value = self.critic(x)
        return action, value
```

## Simulation Integration

### Isaac Sim Integration for the Full System

```python
import omni
from omni.isaac.kit import SimulationApp
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
from omni.isaac.core.articulations import ArticulationView
import carb
import numpy as np

class HumanoidSimulationManager:
    def __init__(self):
        # Launch Isaac Sim
        self.config = {
            'headless': False,
            'render': 'OGL',
            'experience': f'{get_assets_root_path()}/Isaac/Samples/Environments/SmallRoom.usd'
        }

        self.simulation_app = SimulationApp(self.config)
        self.world = None
        self.humanoid = None
        self.humanoid_view = None

    def setup_world(self):
        """Setup the simulation world"""
        # Create world
        self.world = World(stage_units_in_meters=1.0)

        # Add ground plane
        self.world.scene.add_default_ground_plane()

        # Load humanoid robot
        assets_root_path = get_assets_root_path()
        robot_path = "/Isaac/Robots/Humanoid/humanoid_instanceable.usd"

        # Add humanoid to stage
        add_reference_to_stage(
            usd_path=assets_root_path + robot_path,
            prim_path="/World/Humanoid"
        )

        # Create robot instance
        self.humanoid = Robot(
            prim_path="/World/Humanoid",
            name="humanoid_robot",
            position=np.array([0.0, 0.0, 1.0]),
            orientation=np.array([0.0, 0.0, 0.0, 1.0])
        )

        # Add to world
        self.world.scene.add(self.humanoid)

        # Create articulation view for batch operations
        self.humanoid_view = ArticulationView(
            prim_paths_expr="/World/Humanoid",
            name="humanoid_view",
            reset_xform_properties=False
        )
        self.world.scene.add(self.humanoid_view)

        # Add some objects for the robot to interact with
        self.add_interactive_objects()

        # Reset the world
        self.world.reset()

        print("Simulation world setup complete")

    def add_interactive_objects(self):
        """Add objects for the robot to interact with"""
        # Add a table
        self.world.scene.add_default_table(
            prim_path="/World/Table",
            name="table",
            position=np.array([2.0, 0.0, 0.0]),
            size=np.array([1.0, 0.6, 0.8])
        )

        # Add a cup on the table
        from omni.isaac.core.objects import DynamicCuboid
        cup = DynamicCuboid(
            prim_path="/World/Cup",
            name="cup",
            position=np.array([2.1, 0.1, 0.85]),
            size=0.05,
            color=np.array([1.0, 0.0, 0.0])  # Red cup
        )
        self.world.scene.add(cup)

        # Add a ball
        ball = DynamicCuboid(
            prim_path="/World/Ball",
            name="ball",
            position=np.array([-1.0, 1.0, 0.2]),
            size=0.1,
            color=np.array([1.0, 0.0, 0.0])  # Red ball
        )
        self.world.scene.add(ball)

    def run_simulation(self, steps=1000):
        """Run the simulation for specified steps"""
        for i in range(steps):
            self.world.step(render=True)

            if i % 100 == 0:
                print(f"Simulation step: {i}")

                # Check robot state
                joint_positions = self.humanoid_view.get_joint_positions()
                print(f"Joint positions: {joint_positions[0, :5]}...")  # First 5 joints
        print("Simulation completed")

    def cleanup(self):
        """Clean up simulation"""
        if self.world:
            self.world.clear()
        self.simulation_app.close()

# ROS 2 Node for controlling the simulation
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Duration

class SimulationControllerNode(Node):
    def __init__(self):
        super().__init__('simulation_controller')

        # Publishers for robot control
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers for system commands
        self.command_sub = self.create_subscription(
            String, 'system_commands', self.command_callback, 10
        )

        # Initialize simulation manager
        self.sim_manager = HumanoidSimulationManager()
        self.sim_manager.setup_world()

        # Control timer
        self.control_timer = self.create_timer(0.02, self.control_loop)  # 50 Hz

        self.current_command = None
        self.joint_targets = np.zeros(28)  # 28 DOF humanoid

    def command_callback(self, msg):
        """Handle system commands"""
        command = msg.data
        self.current_command = command
        self.get_logger().info(f"Received command: {command}")

        if command == "START_SIMULATION":
            # The simulation is already started in setup
            pass
        elif command == "RESET_ROBOT":
            self.reset_robot()
        elif command.startswith("MOVE_"):
            self.parse_move_command(command)

    def parse_move_command(self, command):
        """Parse movement commands"""
        if "FORWARD" in command:
            self.move_forward()
        elif "BACKWARD" in command:
            self.move_backward()
        elif "LEFT" in command:
            self.turn_left()
        elif "RIGHT" in command:
            self.turn_right()

    def move_forward(self):
        """Send forward movement command"""
        twist = Twist()
        twist.linear.x = 0.5
        self.cmd_vel_pub.publish(twist)

    def move_backward(self):
        """Send backward movement command"""
        twist = Twist()
        twist.linear.x = -0.5
        self.cmd_vel_pub.publish(twist)

    def turn_left(self):
        """Send left turn command"""
        twist = Twist()
        twist.angular.z = 0.5
        self.cmd_vel_pub.publish(twist)

    def turn_right(self):
        """Send right turn command"""
        twist = Twist()
        twist.angular.z = -0.5
        self.cmd_vel_pub.publish(twist)

    def reset_robot(self):
        """Reset robot to initial position"""
        # This would reset the simulation robot in practice
        pass

    def control_loop(self):
        """Main control loop"""
        # Update simulation
        if self.sim_manager.world:
            self.sim_manager.world.step(render=True)

        # Send joint commands if we have targets
        if np.any(self.joint_targets):
            cmd_msg = JointState()
            cmd_msg.name = [f"joint_{i}" for i in range(len(self.joint_targets))]
            cmd_msg.position = self.joint_targets.tolist()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            self.joint_cmd_pub.publish(cmd_msg)

    def destroy_node(self):
        """Clean up when shutting down"""
        if self.sim_manager:
            self.sim_manager.cleanup()
        super().destroy_node()

def run_simulation_with_ros():
    """Run the complete simulation with ROS 2 integration"""
    # Initialize ROS 2
    rclpy.init()

    # Create simulation controller
    controller = SimulationControllerNode()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("Shutting down simulation...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()
```

## System Launch and Configuration

### Complete System Launch File

```xml
<launch>
  <!-- Load robot description -->
  <param name="robot_description"
         value="$(find-pkg-share humanoid_description)/urdf/humanoid.urdf"/>

  <!-- Start robot state publisher -->
  <node pkg="robot_state_publisher"
        exec="robot_state_publisher"
        name="robot_state_publisher">
    <param name="publish_frequency" value="50.0"/>
  </node>

  <!-- Start joint state publisher -->
  <node pkg="joint_state_publisher"
        exec="joint_state_publisher"
        name="joint_state_publisher"/>

  <!-- Start voice recognition -->
  <node pkg="voice_recognition"
        exec="whisper_node"
        name="whisper_node"
        output="screen">
    <param name="model_size" value="base"/>
  </node>

  <!-- Start vision processing -->
  <node pkg="vision_processing"
        exec="detr_node"
        name="detr_node"
        output="screen">
    <param name="model_name" value="facebook/detr-resnet-50"/>
    <param name="confidence_threshold" value="0.7"/>
  </node>

  <!-- Start task planner -->
  <node pkg="task_planning"
        exec="gpt_planner"
        name="gpt_planner"
        output="screen">
    <param name="api_key_path" value="/path/to/api/key"/>
  </node>

  <!-- Start motion control -->
  <node pkg="motion_control"
        exec="humanoid_controller"
        name="humanoid_controller"
        output="screen"/>

  <!-- Start main orchestrator -->
  <node pkg="humanoid_system"
        exec="system_orchestrator"
        name="system_orchestrator"
        output="screen">
    <remap from="~voice_command" to="/recognized_text"/>
    <remap from="~navigation_goal" to="/move_base_simple/goal"/>
    <remap from="~manipulation_command" to="/manipulation/command"/>
  </node>

  <!-- Start simulation (if in sim mode) -->
  <include file="$(find-pkg-share isaac_sim)/launch/standalone.launch.py">
    <arg name="headless" value="False"/>
  </include>

  <!-- Load navigation parameters -->
  <group>
    <param from="$(find-pkg-share navigation2)/config/nav2_params.yaml"/>
  </group>
</launch>
```

## Testing and Validation

### System Integration Tests

```python
import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import time

class TestHumanoidSystem(unittest.TestCase):
    def setUp(self):
        self.node = rclpy.create_node('test_humanoid_system')

        # Publishers for sending test commands
        self.voice_pub = self.node.create_publisher(String, 'voice_command', 10)
        self.nav_pub = self.node.create_publisher(Pose, 'navigation_goal', 10)

        # Subscribers for receiving system responses
        self.status_sub = self.node.create_subscription(
            String, 'system_status', self.status_callback, 10
        )
        self.joint_sub = self.node.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10
        )

        self.status_msg = None
        self.joint_msg = None

    def status_callback(self, msg):
        self.status_msg = msg

    def joint_callback(self, msg):
        self.joint_msg = msg

    def test_voice_command_processing(self):
        """Test that voice commands are processed correctly"""
        # Send a voice command
        cmd_msg = String()
        cmd_msg.data = "move forward"
        self.voice_pub.publish(cmd_msg)

        # Wait for response
        start_time = time.time()
        while self.status_msg is None and (time.time() - start_time) < 5.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Check that the system processed the command
        self.assertIsNotNone(self.status_msg)
        self.assertIn("PROCESSING", self.status_msg.data)

    def test_navigation_command(self):
        """Test that navigation commands work"""
        # Send navigation command
        pose_msg = Pose()
        pose_msg.position.x = 1.0
        pose_msg.position.y = 0.0
        pose_msg.position.z = 0.0
        self.nav_pub.publish(pose_msg)

        # Wait for joint state update (robot movement)
        start_time = time.time()
        while self.joint_msg is None and (time.time() - start_time) < 5.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Check that robot joints are updated
        self.assertIsNotNone(self.joint_msg)
        self.assertGreater(len(self.joint_msg.position), 0)

    def test_system_integration(self):
        """Test full system integration"""
        # Send a complex command that requires multiple subsystems
        cmd_msg = String()
        cmd_msg.data = "go to the kitchen and find the red ball"
        self.voice_pub.publish(cmd_msg)

        # Wait and verify system goes through states
        states_observed = []
        start_time = time.time()

        while (time.time() - start_time) < 10.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)

            if self.status_msg:
                state = self.status_msg.data.split('|')[0].split(':')[1]
                if state not in states_observed:
                    states_observed.append(state)

        # Check that multiple states were reached
        self.assertGreater(len(states_observed), 1)

    def tearDown(self):
        self.node.destroy_node()

def run_tests():
    """Run all tests"""
    rclpy.init()
    test_suite = unittest.TestLoader().loadTestsFromTestCase(TestHumanoidSystem)
    test_runner = unittest.TextTestRunner(verbosity=2)
    result = test_runner.run(test_suite)
    rclpy.shutdown()
    return result.wasSuccessful()

if __name__ == '__main__':
    success = run_tests()
    exit(0 if success else 1)
```

## Deployment Considerations

### Real Hardware Deployment

```python
class HardwareDeploymentManager:
    def __init__(self):
        self.hardware_connected = False
        self.safety_limits = {
            'max_torque': 50.0,  # Nm
            'max_velocity': 2.0, # rad/s
            'max_effort': 100.0  # A
        }

    def check_hardware_safety(self):
        """Check hardware safety before deployment"""
        checks = [
            self.check_motor_health(),
            self.check_battery_level(),
            self.check_joint_limits(),
            self.check_imu_calibration()
        ]

        return all(checks)

    def check_motor_health(self):
        """Check motor health and temperature"""
        # In practice, this would check actual motor feedback
        return True

    def check_battery_level(self):
        """Check battery level is sufficient"""
        # In practice, check actual battery status
        battery_level = 80  # percent
        return battery_level > 20

    def check_joint_limits(self):
        """Check joints are within safe limits"""
        # In practice, check current joint positions
        return True

    def check_imu_calibration(self):
        """Check IMU is properly calibrated"""
        # In practice, check IMU bias and alignment
        return True

    def deploy_to_hardware(self, simulation_model_path, hardware_config_path):
        """Deploy trained model to hardware"""
        print("Deploying trained model to hardware...")

        # Validate safety
        if not self.check_hardware_safety():
            raise RuntimeError("Hardware safety checks failed")

        # Load trained model
        model = self.load_trained_model(simulation_model_path)

        # Adjust model for hardware characteristics
        calibrated_model = self.calibrate_for_hardware(model, hardware_config_path)

        # Deploy to robot
        self.upload_model_to_robot(calibrated_model)

        print("Model deployed successfully!")

    def load_trained_model(self, path):
        """Load trained reinforcement learning model"""
        # This would load the actual trained model
        return "trained_model"

    def calibrate_for_hardware(self, model, config_path):
        """Calibrate model for hardware differences"""
        # Adjust for different dynamics, friction, etc.
        return model

    def upload_model_to_robot(self, model):
        """Upload model to robot's control system"""
        # This would upload the model to the robot's onboard computer
        pass

# Main deployment script
def deploy_humanoid_robot():
    """Main deployment function"""
    deployer = HardwareDeploymentManager()

    try:
        print("Starting humanoid robot deployment...")

        # Check safety
        if not deployer.check_hardware_safety():
            print("ERROR: Safety checks failed!")
            return False

        # Deploy
        deployer.deploy_to_hardware(
            "models/trained_humanoid_policy.pth",
            "config/hardware_calibrations.yaml"
        )

        print("Deployment successful!")
        return True

    except Exception as e:
        print(f"Deployment failed: {e}")
        return False

if __name__ == "__main__":
    success = deploy_humanoid_robot()
    exit(0 if success else 1)
```

## Performance Optimization

### System Optimization Strategies

```python
import psutil
import time
import threading
from collections import deque

class PerformanceMonitor:
    def __init__(self):
        self.cpu_history = deque(maxlen=100)
        self.memory_history = deque(maxlen=100)
        self.ros_latency_history = deque(maxlen=100)

        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self.monitor_loop)
        self.monitor_thread.start()

    def monitor_loop(self):
        """Monitor system performance in background"""
        while self.monitoring:
            # CPU usage
            cpu_percent = psutil.cpu_percent(interval=0.1)
            self.cpu_history.append(cpu_percent)

            # Memory usage
            memory_percent = psutil.virtual_memory().percent
            self.memory_history.append(memory_percent)

            # Log if thresholds exceeded
            if cpu_percent > 80:
                print(f"WARNING: High CPU usage: {cpu_percent}%")
            if memory_percent > 85:
                print(f"WARNING: High memory usage: {memory_percent}%")

            time.sleep(1.0)

    def get_performance_report(self):
        """Get current performance report"""
        if len(self.cpu_history) == 0:
            return "No data collected"

        avg_cpu = sum(self.cpu_history) / len(self.cpu_history)
        avg_memory = sum(self.memory_history) / len(self.memory_history)
        max_cpu = max(self.cpu_history)
        max_memory = max(self.memory_history)

        return f"CPU: avg={avg_cpu:.1f}%, max={max_cpu:.1f}% | Memory: avg={avg_memory:.1f}%, max={max_memory:.1f}%"

    def stop_monitoring(self):
        """Stop performance monitoring"""
        self.monitoring = False
        self.monitor_thread.join()

class OptimizationStrategies:
    def __init__(self):
        self.performance_monitor = PerformanceMonitor()
        self.optimization_enabled = True

    def optimize_perception_pipeline(self):
        """Optimize computer vision pipeline"""
        # Strategies:
        # 1. Use lower resolution for initial detection
        # 2. Scale up only when tracking specific objects
        # 3. Use model quantization
        # 4. Implement multi-threading
        pass

    def optimize_communication(self):
        """Optimize ROS 2 communication"""
        # Strategies:
        # 1. Use intra-process communication where possible
        # 2. Adjust QoS settings for real-time requirements
        # 3. Implement message filtering
        # 4. Use compression for large messages
        pass

    def optimize_control_frequency(self):
        """Optimize control loop frequencies"""
        # Different components need different frequencies:
        # - High-level planning: 1-5 Hz
        # - Navigation: 10-20 Hz
        # - Low-level control: 100-500 Hz
        pass

def main_optimization_loop():
    """Main optimization control loop"""
    optimizer = OptimizationStrategies()

    try:
        while True:
            # Check performance
            report = optimizer.performance_monitor.get_performance_report()
            print(f"Performance: {report}")

            # Apply optimizations based on performance
            if "WARNING" in report:
                # Reduce processing load
                print("Applying performance optimizations...")
                optimizer.optimize_perception_pipeline()
                optimizer.optimize_communication()

            time.sleep(5.0)  # Check every 5 seconds

    except KeyboardInterrupt:
        print("Stopping optimization monitoring...")
        optimizer.performance_monitor.stop_monitoring()

if __name__ == "__main__":
    main_optimization_loop()
```

## Quiz

1. What is the main purpose of the system orchestrator in the VLA system?
   A) To control robot joints directly
   B) To coordinate and manage all subsystems
   C) To process visual information
   D) To generate speech output

2. Which reinforcement learning method is typically used for humanoid robot control?
   A) Q-Learning
   B) Deep Q-Networks
   C) Policy Gradient (Actor-Critic)
   D) Supervised Learning

3. What does the acronym VLA stand for in the context of this capstone?
   A) Vision-Language-Actuation
   B) Vision-Language-Action
   C) Virtual-Learning-Assistant
   D) Vision-Localization-Action

Answers: 1-B, 2-C, 3-B

## Summary

The capstone project demonstrates the complete integration of all concepts covered throughout the book into a functional autonomous humanoid robot system. This includes voice recognition with Whisper, computer vision with DETR, task planning with GPT, humanoid kinematics and control, simulation in Isaac Sim, and reinforcement learning for adaptive behavior. The system showcases how Physical AI principles can be applied to create embodied agents that understand natural language commands and execute complex tasks in real-world environments. The project emphasizes the importance of system architecture, safety considerations, performance optimization, and the integration of multiple AI technologies to achieve robust autonomous behavior.