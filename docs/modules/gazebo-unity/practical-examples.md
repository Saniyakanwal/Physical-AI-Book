---
title: Gazebo and Unity Practical Examples
sidebar_position: 4
---

# Gazebo and Unity Practical Examples

## Overview
This chapter provides practical examples of implementing robotics simulations in both Gazebo and Unity.

## Learning Objectives
After completing this chapter, you will:
- Implement a complete robot simulation in Gazebo
- Create a robot simulation in Unity with ROS integration
- Understand the workflow for both platforms
- Learn to debug and validate simulations

## Gazebo Practical Example: TurtleBot3 Simulation

### Setting up the Environment
First, install the TurtleBot3 packages:

```bash
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-gazebo
```

### Launching the Simulation
```bash
# Set the environment
export TURTLEBOT3_MODEL=burger

# Launch the simulation
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### Custom World Creation
Create a custom world file `my_world.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="my_world">
    <include>
      <uri>model://sun</uri>
    </include>
    
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Add a simple wall -->
    <model name="wall">
      <pose>0 2 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>4 0.2 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>4 0.2 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.1 1</ambient>
            <diffuse>0.8 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Controlling the Robot
Use teleop to control the robot:

```bash
# In a new terminal
ros2 run turtlebot3_teleop teleop_keyboard
```

## Unity Practical Example: Robot Simulation

### Setting up Unity for Robotics
1. Install Unity Hub and Unity Editor (2021.3 LTS recommended)
2. Create a new 3D project
3. Install the Unity Robotics Package via Package Manager
4. Install ROS# for ROS communication

### Basic Robot Setup in Unity
1. Create a robot model using primitives or import from CAD
2. Add colliders to each part
3. Add rigidbodies to parts that need physics
4. Configure joints to connect parts
5. Add sensors (camera, LIDAR, etc.)

### Example: Simple Differential Drive Robot
```csharp
using UnityEngine;

public class DifferentialDriveController : MonoBehaviour
{
    public float linearVelocity = 1.0f;
    public float angularVelocity = 1.0f;
    
    public Transform leftWheel;
    public Transform rightWheel;
    
    private float leftWheelRadius = 0.1f;
    private float rightWheelRadius = 0.1f;
    private float wheelBase = 0.4f; // Distance between wheels
    
    void Update()
    {
        // Simulate velocity commands (in a real implementation, these would come from ROS)
        float v = linearVelocity; // Linear velocity
        float w = angularVelocity; // Angular velocity
        
        // Calculate wheel velocities
        float rightWheelVel = (2 * v + w * wheelBase) / (2 * rightWheelRadius);
        float leftWheelVel = (2 * v - w * wheelBase) / (2 * leftWheelRadius);
        
        // Apply rotation to wheels
        if (leftWheel != null)
            leftWheel.Rotate(Vector3.right, leftWheelVel * Mathf.Rad2Deg * Time.deltaTime);
        if (rightWheel != null)
            rightWheel.Rotate(Vector3.right, rightWheelVel * Mathf.Rad2Deg * Time.deltaTime);
    }
}
```

### ROS Integration in Unity
Using ROS# to connect Unity to ROS:

1. Add ROSConnection component to a GameObject
2. Create publisher and subscriber scripts
3. Send and receive messages between Unity and ROS

Example publisher script:

```csharp
using UnityEngine;
using RosSharp.RosBridgeClient;

public class UnityPublisher : MonoBehaviour
{
    private RosSocket rosSocket;
    
    void Start()
    {
        // Connect to ROS
        rosSocket = new RosSocket(new RosSharp.RosBridgeClient.Protocols.WebSocketNetProtocol("ws://localhost:9090"));
    }
    
    void PublishMessage()
    {
        // Publish a message to a ROS topic
        rosSocket.Publish("unity_data", new RosSharp.StdMsgs.String { data = "Hello from Unity!" });
    }
}
```

## Validation and Debugging

### Gazebo Validation
- Compare simulation results with real robot data
- Validate physics parameters (mass, friction, etc.)
- Check sensor accuracy against real sensors
- Test edge cases and failure scenarios

### Unity Validation
- Verify physics parameters match real robot
- Test sensor outputs against real sensors
- Validate kinematic models
- Check for performance bottlenecks

## Performance Considerations

### Gazebo Performance
- Use simpler collision meshes for dynamic objects
- Reduce sensor update rates when possible
- Limit the number of active physics objects
- Use GPU-based rendering when available

### Unity Performance
- Optimize draw calls and batching
- Use appropriate LOD systems
- Profile physics calculations
- Consider fixed timestep for physics consistency

import ChatInterface from '@site/src/components/chatbot/ChatInterface';

<ChatInterface chapterId="gazebo-unity-practical-examples" />