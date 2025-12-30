---
title: ROS2 Practical Examples
sidebar_position: 4
---

# ROS2 Practical Examples

## Overview
This chapter provides practical examples of ROS2 applications in real-world robotics scenarios.

## Learning Objectives
After completing this chapter, you will:
- Implement a complete ROS2 robot control system
- Create a multi-node ROS2 application
- Integrate sensors and actuators using ROS2
- Deploy a ROS2 application to a physical robot

## TurtleBot3 Example

The TurtleBot3 is a popular educational robot platform that works well with ROS2. Let's look at a practical example of controlling a TurtleBot3.

### Basic Navigation

Here's an example of a simple navigation node that moves the robot forward:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.navigate)
        
    def navigate(self):
        msg = Twist()
        msg.linear.x = 0.2  # Move forward at 0.2 m/s
        msg.angular.z = 0.0  # No rotation
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Sensor Integration

ROS2 makes it easy to integrate various sensors:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
    def scan_callback(self, msg):
        # Check for obstacles in front of the robot
        front_scan = msg.ranges[len(msg.ranges)//2]  # Front reading
        
        cmd = Twist()
        if front_scan > 1.0:  # No obstacle within 1 meter
            cmd.linear.x = 0.2
        else:  # Obstacle detected, turn
            cmd.angular.z = 0.5
            
        self.publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Multi-Robot Systems

ROS2 supports multi-robot systems through DDS (Data Distribution Service):

- Each robot runs in its own ROS domain
- Communication between robots happens through DDS
- Coordination can be achieved through shared topics/services

## Simulation Integration

ROS2 works seamlessly with simulation environments like Gazebo:

- Robot models can be loaded into Gazebo
- Sensors and actuators are simulated
- ROS2 nodes can control the simulated robot
- Results can be validated before deployment to real hardware

import ChatInterface from '@site/src/components/chatbot/ChatInterface';

<ChatInterface chapterId="ros2-practical-examples" />