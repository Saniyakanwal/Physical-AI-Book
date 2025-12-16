---
title: "Humanoid Robotics Fundamentals"
sidebar_position: 1
---

# Humanoid Robotics Fundamentals

Humanoid robotics represents one of the most challenging areas in robotics, requiring sophisticated control of multiple degrees of freedom to achieve stable locomotion, balance, and manipulation. This chapter covers the essential kinematics, dynamics, and control strategies that make humanoid robots possible.

## Learning Outcomes

By the end of this chapter, you will:
- Understand humanoid robot kinematics and inverse kinematics
- Analyze the dynamics of bipedal locomotion
- Implement balance control strategies (ZMP, Capture Point)
- Design walking patterns and gait generation
- Address challenges specific to humanoid robots
- Implement ROS 2 controllers for humanoid systems

## Humanoid Robot Topology

Humanoid robots typically follow a specific topological structure to enable dexterous manipulation and bipedal locomotion:

```
                 Head
                   |
                Torso
              /       \
           Left       Right
        Arm/Hand    Arm/Hand
            |           |
           Hip         Hip
            |           |
        Left Leg    Right Leg
            |           |
         Ankle       Ankle
            |           |
         Foot        Foot
```

### Common Humanoid Configurations

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

class HumanoidRobot:
    def __init__(self):
        # Define joint names and configuration
        self.joints = {
            # Torso
            'torso_pitch': 1,
            'torso_roll': 1,
            'torso_yaw': 1,

            # Left arm
            'l_shoulder_pitch': 1,
            'l_shoulder_roll': 1,
            'l_shoulder_yaw': 1,
            'l_elbow_pitch': 1,
            'l_wrist_pitch': 1,
            'l_wrist_yaw': 1,

            # Right arm
            'r_shoulder_pitch': 1,
            'r_shoulder_roll': 1,
            'r_shoulder_yaw': 1,
            'r_elbow_pitch': 1,
            'r_wrist_pitch': 1,
            'r_wrist_yaw': 1,

            # Left leg
            'l_hip_pitch': 1,
            'l_hip_roll': 1,
            'l_hip_yaw': 1,
            'l_knee_pitch': 1,
            'l_ankle_pitch': 1,
            'l_ankle_roll': 1,

            # Right leg
            'r_hip_pitch': 1,
            'r_hip_roll': 1,
            'r_hip_yaw': 1,
            'r_knee_pitch': 1,
            'r_ankle_pitch': 1,
            'r_ankle_roll': 1,
        }

        # Joint limits (degrees)
        self.joint_limits = {
            'l_hip_pitch': (-120, 30),
            'l_hip_roll': (-20, 20),
            'l_hip_yaw': (-20, 20),
            'l_knee_pitch': (0, 130),
            'l_ankle_pitch': (-30, 30),
            'l_ankle_roll': (-20, 20),
        }

    def get_dof_count(self):
        return sum(self.joints.values())
```

## Forward and Inverse Kinematics

### Forward Kinematics

```python
class ForwardKinematics:
    def __init__(self):
        # DH parameters for a simplified humanoid leg
        self.dh_params = {
            'hip_yaw': {'a': 0, 'alpha': 0, 'd': 0.1, 'theta': 0},
            'hip_pitch': {'a': 0, 'alpha': -np.pi/2, 'd': 0, 'theta': 0},
            'hip_roll': {'a': 0, 'alpha': np.pi/2, 'd': 0, 'theta': 0},
            'knee': {'a': 0.2, 'alpha': 0, 'd': 0, 'theta': 0},
            'ankle_pitch': {'a': 0.2, 'alpha': 0, 'd': 0, 'theta': 0},
        }

    def dh_transform(self, a, alpha, d, theta):
        """Denavit-Hartenberg transformation matrix"""
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def compute_foot_position(self, joint_angles):
        """Compute foot position from joint angles"""
        T = np.eye(4)  # Identity transformation

        # Extract angles for leg joints
        hip_yaw, hip_pitch, hip_roll, knee, ankle_pitch = joint_angles

        # Update DH parameters with current joint angles
        params = [
            self.dh_params['hip_yaw'].copy(),
            self.dh_params['hip_pitch'].copy(),
            self.dh_params['hip_roll'].copy(),
            self.dh_params['knee'].copy(),
            self.dh_params['ankle_pitch'].copy()
        ]

        params[0]['theta'] = hip_yaw
        params[1]['theta'] = hip_pitch
        params[2]['theta'] = hip_roll
        params[3]['theta'] = knee
        params[4]['theta'] = ankle_pitch

        # Multiply transformation matrices
        for param in params:
            T = T @ self.dh_transform(
                param['a'], param['alpha'], param['d'], param['theta']
            )

        return T[:3, 3]  # Return position vector
```

### Inverse Kinematics

```python
import numpy as np
from scipy.optimize import minimize

class InverseKinematics:
    def __init__(self):
        self.l1 = 0.2  # Thigh length
        self.l2 = 0.2  # Shin length
        self.epsilon = 1e-6

    def leg_ik(self, target_x, target_y, target_z):
        """
        Inverse kinematics for planar leg (simplified 2D)
        target_x, target_y, target_z: desired foot position
        """
        # Calculate distance from hip to target
        d = np.sqrt(target_x**2 + target_y**2 + target_z**2)

        # Check if target is reachable
        if d > (self.l1 + self.l2):
            # Return elbow-up configuration for reachable target
            target_x *= (self.l1 + self.l2) / d
            target_y *= (self.l1 + self.l2) / d
            target_z *= (self.l1 + self.l2) / d
            d = self.l1 + self.l2

        # Hip pitch angle
        hip_pitch = np.arctan2(target_z, np.sqrt(target_x**2 + target_y**2))

        # Knee angle calculation
        # Law of cosines to find knee angle
        cos_knee = (self.l1**2 + self.l2**2 - d**2) / (2 * self.l1 * self.l2)
        knee_angle = np.pi - np.arccos(np.clip(cos_knee, -1, 1))

        # Hip angle at knee
        cos_hip = (self.l1**2 + d**2 - self.l2**2) / (2 * self.l1 * d)
        hip_offset = np.arccos(np.clip(cos_hip, -1, 1))

        # Hip pitch offset
        hip_pitch_final = hip_pitch - hip_offset

        # Ankle pitch for foot orientation
        ankle_pitch = -(hip_pitch_final + knee_angle)

        return np.array([hip_pitch, knee_angle, ankle_pitch])

    def optimize_ik(self, target_pos, current_angles):
        """
        Optimize IK using numerical methods for more complex configurations
        """
        def cost_function(angles):
            # Forward kinematics to get current position
            current_pos = self.forward_kinematics(angles)
            # Calculate distance to target
            return np.linalg.norm(current_pos - target_pos)

        result = minimize(
            cost_function,
            current_angles,
            method='BFGS',
            options={'disp': False}
        )

        return result.x

    def forward_kinematics(self, angles):
        """Simple forward kinematics for optimization"""
        hip_pitch, knee_angle, ankle_pitch = angles
        # Simplified implementation
        x = self.l1 * np.sin(hip_pitch) + self.l2 * np.sin(hip_pitch + knee_angle)
        z = self.l1 * np.cos(hip_pitch) + self.l2 * np.cos(hip_pitch + knee_angle)
        return np.array([0, 0, z])  # Simplified 1D case
```

## Balance Control and Zero Moment Point (ZMP)

### ZMP Calculation

```python
import numpy as np

class ZMPController:
    def __init__(self, robot_mass, gravity=9.81):
        self.mass = robot_mass
        self.gravity = gravity
        self.com_height = 0.8  # Center of mass height

    def calculate_zmp(self, com_pos, com_acc, foot_pos):
        """
        Calculate Zero Moment Point (ZMP) position
        com_pos: [x, y, z] center of mass position
        com_acc: [x, y, z] center of mass acceleration
        foot_pos: [x, y] supporting foot position
        """
        x_com, y_com, z_com = com_pos
        x_acc, y_acc, z_acc = com_acc

        # ZMP equations assuming constant CoM height
        zmp_x = x_com - (self.com_height * x_acc) / (self.gravity + z_acc)
        zmp_y = y_com - (self.com_height * y_acc) / (self.gravity + z_acc)

        return np.array([zmp_x, zmp_y])

    def is_balanced(self, zmp, support_polygon):
        """
        Check if ZMP is within support polygon
        support_polygon: [[x1, y1], [x2, y2], ...] vertices of support area
        """
        zmp_x, zmp_y = zmp
        n = len(support_polygon)
        inside = False

        p1x, p1y = support_polygon[0]
        for i in range(1, n + 1):
            p2x, p2y = support_polygon[i % n]
            if zmp_y > min(p1y, p2y):
                if zmp_y <= max(p1y, p2y):
                    if zmp_x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (zmp_y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or zmp_x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y

        return inside

class BalanceController:
    def __init__(self):
        self.kp = 10.0  # Proportional gain
        self.ki = 1.0   # Integral gain
        self.zmp_controller = ZMPController(robot_mass=50)
        self.integral_error = np.array([0.0, 0.0])
        self.previous_error = np.array([0.0, 0.0])

    def balance_correction(self, desired_zmp, actual_zmp, dt=0.01):
        """
        Compute balance correction using PID control
        """
        error = desired_zmp - actual_zmp

        # Integral term
        self.integral_error += error * dt

        # Derivative term
        derivative = (error - self.previous_error) / dt if dt > 0 else 0

        # PID output
        output = (self.kp * error +
                 self.ki * self.integral_error +
                 0.1 * derivative)  # 0.1 for derivative gain

        self.previous_error = error

        return output
```

## Walking Pattern Generation

### Capture Point and Walking Control

```python
class WalkingController:
    def __init__(self, com_height=0.8, gravity=9.81, dt=0.01):
        self.com_height = com_height
        self.gravity = gravity
        self.dt = dt
        self.omega = np.sqrt(gravity / com_height)

        # Walking parameters
        self.step_length = 0.3
        self.step_width = 0.2
        self.step_height = 0.05
        self.step_duration = 1.0

    def capture_point(self, com_pos, com_vel):
        """
        Calculate Capture Point - where to step to stop the robot
        com_pos: [x, y, z] center of mass position
        com_vel: [x, y, z] center of mass velocity
        """
        x_com, y_com, _ = com_pos
        x_vel, y_vel, _ = com_vel

        # Capture point equations
        capture_x = x_com + x_vel / self.omega
        capture_y = y_com + y_vel / self.omega

        return np.array([capture_x, capture_y])

    def generate_trajectory(self, start_pos, goal_pos, step_count):
        """
        Generate walking trajectory to reach goal position
        """
        trajectory = []
        current_pos = start_pos.copy()

        # Calculate step direction
        direction = goal_pos - start_pos
        distance = np.linalg.norm(direction)
        step_vector = direction / distance * self.step_length if distance > 0 else np.array([0, 0])

        for i in range(step_count):
            # Calculate next foot position
            next_pos = current_pos + step_vector

            # Generate step trajectory (simple sinusoid for foot lift)
            step_trajectory = self.generate_step_trajectory(current_pos, next_pos)
            trajectory.extend(step_trajectory)

            current_pos = next_pos

        return trajectory

    def generate_step_trajectory(self, start_foot, end_foot):
        """
        Generate foot trajectory for a single step
        """
        step_points = []
        step_duration = self.step_duration
        num_points = int(step_duration / self.dt)

        for i in range(num_points):
            t = i / num_points  # Normalized time [0, 1]

            # Linear interpolation for x, y
            x = start_foot[0] + (end_foot[0] - start_foot[0]) * t
            y = start_foot[1] + (end_foot[1] - start_foot[1]) * t

            # Sinusoidal trajectory for z (foot lift)
            z = self.step_height * np.sin(np.pi * t)

            step_points.append([x, y, z])

        return step_points

class GaitGenerator:
    def __init__(self):
        self.phase = 0.0
        self.stance_duration = 0.8
        self.swing_duration = 0.2
        self.total_phase = 1.0

    def get_gait_phase(self, time):
        """
        Calculate current gait phase (0.0 to 1.0)
        0.0-0.8: Stance phase, 0.8-1.0: Swing phase
        """
        # Normalize time to gait cycle
        cycle_time = self.stance_duration + self.swing_duration
        normalized_time = (time % cycle_time) / cycle_time
        return normalized_time

    def generate_leg_trajectory(self, start_pos, target_pos, phase):
        """
        Generate trajectory for leg movement based on gait phase
        """
        if phase <= self.stance_duration:
            # Stance phase - leg stays in contact
            return start_pos
        else:
            # Swing phase - leg moves to target
            swing_phase = (phase - self.stance_duration) / self.swing_duration
            # Smooth interpolation using cubic curve
            smooth_factor = 3 * swing_phase**2 - 2 * swing_phase**3

            return start_pos + (target_pos - start_pos) * smooth_factor
```

## Dynamics and Control

### Equations of Motion

```python
import numpy as np
from scipy.integrate import solve_ivp

class HumanoidDynamics:
    def __init__(self, robot_mass=50):
        self.mass = robot_mass
        self.gravity = 9.81

    def eom(self, t, state, tau):
        """
        Equations of Motion for humanoid robot
        state: [q, q_dot] - joint positions and velocities
        tau: joint torques
        """
        # Extract state variables
        n = len(state) // 2
        q = state[:n]      # Joint positions
        q_dot = state[n:]  # Joint velocities

        # Compute Mass Matrix (M)
        M = self.mass_matrix(q)

        # Compute Coriolis and Centrifugal terms (C)
        C = self.coriolis_matrix(q, q_dot)

        # Compute gravitational terms (G)
        G = self.gravity_vector(q)

        # Solve: M(q) * q_ddot + C(q, qdot) * qdot + G(q) = tau
        q_ddot = np.linalg.solve(M, tau - C @ q_dot - G)

        # Return derivatives
        return np.concatenate([q_dot, q_ddot])

    def mass_matrix(self, q):
        """Compute mass matrix using recursive Newton-Euler algorithm (simplified)"""
        # Identity matrix as placeholder
        # In practice, this would be computed using full rigid body dynamics
        n = len(q)
        return np.eye(n) * 1.0  # Simplified diagonal matrix

    def coriolis_matrix(self, q, q_dot):
        """Compute Coriolis and centrifugal terms"""
        # Simplified: zero matrix for this example
        n = len(q)
        return np.zeros((n, n))

    def gravity_vector(self, q):
        """Compute gravity terms"""
        # Simplified: gravity compensation based on joint positions
        return np.zeros(len(q))

class ComputedTorqueController:
    def __init__(self, robot_dynamics):
        self.dyn = robot_dynamics
        self.kp = np.diag([100] * 12)  # Position gains
        self.kd = np.diag([20] * 12)   # Velocity gains

    def compute_torques(self, q, q_dot, q_des, qd_des, qdd_des):
        """
        Compute torques using computed torque control
        q, q_dot: actual joint positions and velocities
        q_des, qd_des, qdd_des: desired joint positions, velocities, accelerations
        """
        # Error terms
        q_error = q_des - q
        qd_error = qd_des - q_dot

        # Feedforward + feedback control
        qdd_cmd = qdd_des + self.kp @ q_error + self.kd @ qd_error

        # Compute torques using inverse dynamics
        M = self.dyn.mass_matrix(q)
        C = self.dyn.coriolis_matrix(q, q_dot)
        G = self.dyn.gravity_vector(q)

        tau = M @ qdd_cmd + C @ q_dot + G

        return tau
```

## ROS 2 Implementation for Humanoid Control

### Humanoid Joint Controller

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
import numpy as np
from scipy.interpolate import interp1d

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Joint state subscriber
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Command publishers
        self.joint_cmd_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Balance controller publisher
        self.balance_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/balance_control_commands',
            10
        )

        # Current joint states
        self.current_positions = {}
        self.current_velocities = {}
        self.joint_names = [
            'l_hip_pitch', 'l_hip_roll', 'l_hip_yaw',
            'l_knee_pitch', 'l_ankle_pitch', 'l_ankle_roll',
            'r_hip_pitch', 'r_hip_roll', 'r_hip_yaw',
            'r_knee_pitch', 'r_ankle_pitch', 'r_ankle_roll'
        ]

        # Initialize balance controller
        self.balance_controller = BalanceController()
        self.walking_controller = WalkingController()

        # Control timer
        self.control_timer = self.create_timer(0.01, self.control_loop)

    def joint_state_callback(self, msg):
        """Update current joint states"""
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                if i < len(msg.position):
                    self.current_positions[name] = msg.position[i]
                if i < len(msg.velocity):
                    self.current_velocities[name] = msg.velocity[i]

    def control_loop(self):
        """Main control loop for humanoid balance and locomotion"""
        # Get current CoM estimate (simplified)
        current_com = self.estimate_com()
        current_com_vel = self.estimate_com_velocity()

        # Calculate ZMP
        zmp = self.balance_controller.zmp_controller.calculate_zmp(
            current_com,
            np.array([0, 0, 0]),  # Acceleration (simplified)
            self.get_support_polygon_center()
        )

        # Check balance and compute corrections
        desired_zmp = np.array([0.0, 0.0])  # Desired ZMP at center
        balance_correction = self.balance_controller.balance_correction(
            desired_zmp, zmp
        )

        # Publish balance corrections
        balance_msg = Float64MultiArray()
        balance_msg.data = balance_correction.tolist()
        self.balance_cmd_pub.publish(balance_msg)

        # Generate walking commands if needed
        self.execute_walking_pattern()

    def estimate_com(self):
        """Estimate center of mass (simplified)"""
        # In practice, this would use forward kinematics and mass distribution
        return np.array([0.0, 0.0, 0.8])

    def estimate_com_velocity(self):
        """Estimate CoM velocity"""
        return np.array([0.0, 0.0, 0.0])

    def get_support_polygon_center(self):
        """Estimate support polygon center between feet"""
        # Simplified: return center point
        return np.array([0.0, 0.0])

    def execute_walking_pattern(self):
        """Execute walking gait"""
        # Generate walking trajectory
        start_pos = np.array([0.0, 0.0])
        goal_pos = np.array([1.0, 0.0])

        trajectory = self.walking_controller.generate_trajectory(
            start_pos, goal_pos, step_count=10
        )

        # Send trajectory to joints
        self.send_joint_trajectory(trajectory)

    def send_joint_trajectory(self, trajectory):
        """Send joint trajectory to robot"""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        # Create trajectory points
        for i, pos in enumerate(trajectory):
            point = JointTrajectoryPoint()
            point.positions = [0.0] * len(self.joint_names)  # Placeholder
            point.velocities = [0.0] * len(self.joint_names)
            point.accelerations = [0.0] * len(self.joint_names)
            point.time_from_start = Duration(sec=0, nanosec=int(i * 0.1 * 1e9))
            msg.points.append(point)

        self.joint_cmd_pub.publish(msg)

def main():
    rclpy.init()
    controller = HumanoidController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Challenges in Humanoid Robotics

### Key Challenges and Solutions

1. **Stability**: Maintaining balance with limited support polygon
   - Solution: Advanced balance control algorithms (ZMP, Capture Point)

2. **Real-time Control**: Processing complex dynamics at high frequencies
   - Solution: Optimized control algorithms and parallel processing

3. **Complexity**: Managing many degrees of freedom simultaneously
   - Solution: Hierarchical control and task decomposition

4. **Energy Efficiency**: Operating on limited battery power
   - Solution: Efficient trajectory planning and passive dynamics

5. **Robustness**: Handling uncertainties and disturbances
   - Solution: Robust control and adaptive algorithms

## Quiz

1. What does ZMP stand for in humanoid robotics?
   A) Zero Motion Point
   B) Zero Moment Point
   C) Zero Momentum Point
   D) Zero Motor Point

2. Which algorithm is used for generating walking patterns?
   A) PID Control
   B) Capture Point method
   C) Inverse Dynamics
   D) Forward Kinematics

3. What is the typical number of joints in a humanoid leg?
   A) 3
   B) 4
   C) 6
   D) 8

Answers: 1-B, 2-B, 3-C

## Summary

Humanoid robotics requires sophisticated understanding of kinematics, dynamics, and control theory. The challenges of bipedal locomotion, balance maintenance, and dexterous manipulation require advanced algorithms and careful system design. Modern humanoid robots implement hierarchical control systems that handle everything from low-level joint control to high-level task planning, with ROS 2 providing the communication infrastructure to manage these complex systems effectively.