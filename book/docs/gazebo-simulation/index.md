import Chatbot from '@site/src/components/Chatbot';

---
title: "Gazebo Simulation"
sidebar_position: 1
---

# Gazebo Simulation

Gazebo is a physics-based simulation environment that enables testing of robot algorithms, design of robots, and training of AI systems in a safe, reproducible virtual environment. It provides realistic physics simulation, high-quality graphics, and sensor simulation essential for robotics development.

## Learning Outcomes

By the end of this chapter, you will:
- Understand URDF and SDF robot description formats
- Create robot models with joints, links, and sensors
- Configure physics properties and materials
- Integrate Gazebo with ROS 2 using gazebo_ros_pkgs
- Implement sensor simulation for cameras, IMUs, and LiDARs

## Gazebo Architecture

Gazebo operates using a client-server architecture with multiple components working together:

```
"O"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"    "O"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"    "O"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?
",   Gazebo GUI    ",    ",   Gazebo Server ",    ",     Plugins     ",
",   (Client)      ",    ",                 ",    ",                 ",
", ? Visualization ",    ", ? Physics Engine",    ", ? Sensor        ",
", ? Controls      ",    ", ? Collision Det.",    ", ? ROS Interface ",
", ? Scene         ",    ", ? Dynamics      ",    ", ? Controllers   ",
""?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"~    ""?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"~    ""?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?~
         ",                        ",                       ",
         ""?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?
                                  ",
                    "O"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?
                    ",   ROS 2 Bridge  ",
                    ",                 ",
                    ", ? Topics        ",
                    ", ? Services      ",
                    ", ? Actions       ",
                    ""?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?
```

## URDF Robot Description

URDF (Unified Robot Description Format) is an XML format that describes robot models in terms of links, joints, and properties.

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Wheel links -->
  <link name="wheel_front_left">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.004"/>
    </inertial>
  </link>

  <!-- Joint connecting wheel to base -->
  <joint name="front_left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_front_left"/>
    <origin xyz="0.2 0.25 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```

## SDF (Simulation Description Format)

SDF is Gazebo's native format that provides more features than URDF, including world descriptions and physics properties.

### SDF World File

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_world">
    <!-- Physics engine -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun light -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- A simple robot -->
    <model name="simple_robot">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="chassis">
        <pose>0 0 0.1 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>1.0 0.5 0.2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.0 0.5 0.2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.4 1 1</ambient>
            <diffuse>0.4 0.4 1 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.058</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.125</iyy>
            <iyz>0</iyz>
            <izz>0.166</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

## URDF with Gazebo Extensions

To use URDF models in Gazebo, we add Gazebo-specific extensions:

```xml
<?xml version="1.0"?>
<robot name="robot_with_gazebo_extensions" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Gazebo-specific material definition -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>

  <!-- Transmission for ROS control -->
  <transmission name="front_left_wheel_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="front_left_wheel_joint">
      <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    </joint>
    <actuator name="front_left_wheel_motor">
      <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <!-- Gazebo plugin for ROS control -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/simple_robot</robotNamespace>
      <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    </plugin>
  </gazebo>

  <!-- Sensor simulation -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera1">
      <update_rate>30.0</update_rate>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>800</width>
          <height>600</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>0.0</updateRate>
        <cameraName>simple_camera</cameraName>
        <imageTopicName>image_raw</imageTopicName>
        <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <frameName>camera_link</frameName>
        <hackBaseline>0.07</hackBaseline>
        <distortionK1>0.0</distortionK1>
        <distortionK2>0.0</distortionK2>
        <distortionK3>0.0</distortionK3>
        <distortionT1>0.0</distortionT1>
        <distortionT2>0.0</distortionT2>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

## Sensor Simulation

Gazebo supports various sensor types with realistic physics simulation.

### LiDAR Sensor Implementation

```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="laser_scanner">
    <pose>0 0 0 0 0 0</pose>
    <visualize>false</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="laser_controller" filename="libgazebo_ros_laser.so">
      <topicName>/simple_robot/laser_scan</topicName>
      <frameName>lidar_link</frameName>
    </plugin>
  </sensor>
</gazebo>
```

### IMU Sensor Implementation

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>false</visualize>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
      <topicName>/simple_robot/imu</topicName>
      <bodyName>imu_link</bodyName>
      <frameName>imu_link</frameName>
      <serviceName>/simple_robot/imu_service</serviceName>
      <gaussianNoise>0.01</gaussianNoise>
      <updateRate>100.0</updateRate>
    </plugin>
  </sensor>
</gazebo>
```

## ROS 2 Integration

### Launch File for Gazebo Simulation

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Get package directory
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_simple_robot = get_package_share_directory('simple_robot_description')

    # Launch Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': os.path.join(pkg_simple_robot, 'worlds', 'simple_world.world')}.items(),
    )

    # Launch Gazebo client
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'simple_robot',
            '-file', os.path.join(pkg_simple_robot, 'urdf', 'simple_robot.urdf'),
            '-x', '0', '-y', '0', '-z', '0.5'
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[os.path.join(pkg_simple_robot, 'urdf', 'simple_robot.urdf')]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        gzserver,
        gzclient,
        spawn_entity,
        robot_state_publisher,
    ])
```

### Controlling the Robot in Gazebo

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class GazeboController(Node):

    def __init__(self):
        super().__init__('gazebo_controller')

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/simple_robot/cmd_vel', 10)

        # Subscribers for sensor data
        self.laser_sub = self.create_subscription(
            LaserScan, '/simple_robot/laser_scan', self.laser_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/simple_robot/odom', self.odom_callback, 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.laser_data = None
        self.odom_data = None

    def laser_callback(self, msg):
        self.laser_data = msg

    def odom_callback(self, msg):
        self.odom_data = msg

    def control_loop(self):
        cmd = Twist()

        # Simple obstacle avoidance
        if self.laser_data:
            min_distance = min(self.laser_data.ranges)
            if min_distance < 1.0:
                cmd.angular.z = 1.0  # Turn
            else:
                cmd.linear.x = 0.5   # Move forward

        self.cmd_vel_pub.publish(cmd)

def main():
    rclpy.init()
    controller = GazeboController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Physics Configuration

Physics properties significantly affect simulation realism and performance:

```
"O"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?
",                Physics Parameters                           ",
"o"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?
", ? Gravity: [0, 0, -9.81] m/sA                            ",
", ? Real-time factor: 1.0 (match real time)                  ",
", ? Max step size: 0.001s (smaller = more accurate)          ",
", ? Solver: ODE, Bullet, Simbody                             ",
", ? Friction: Control surface interactions                    ",
", ? Damping: Energy dissipation parameters                    ",
""?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?"?
```

## Best Practices

1. **Model Complexity**: Balance detail with performance
2. **Sensor Noise**: Add realistic noise models
3. **Physics Parameters**: Tune for realistic behavior
4. **Validation**: Test in simulation before real hardware
5. **Modular Design**: Use Xacro for complex models
6. **Reproducibility**: Set random seeds for consistent results

## Quiz

1. What does URDF stand for?
   A) Universal Robot Description Format
   B) Unified Robot Description Format
   C) Universal Robot Design Framework
   D) Unified Robotic Development Format

2. Which SDF element defines physics properties?
   A) `<dynamics>`
   B) `<physics>`
   C) `<engine>`
   D) `<properties>`

3. What type of sensor is represented by `<sensor type="ray">`?
   A) Camera
   B) IMU
   C) LiDAR
   D) GPS

Answers: 1-B, 2-B, 3-C

## Summary

Gazebo provides a powerful simulation environment for robotics development, supporting both URDF and SDF robot descriptions with realistic physics and sensor simulation. Proper integration with ROS 2 allows seamless transition from simulation to real hardware. Understanding URDF/SDF formats, sensor configurations, and physics parameters is essential for effective simulation-based development.

<Chatbot />