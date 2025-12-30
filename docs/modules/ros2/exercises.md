---
title: ROS2 Exercises
sidebar_position: 5
---

# ROS2 Exercises

## Overview
This chapter provides hands-on exercises to reinforce the concepts learned in the ROS2 module.

## Exercise 1: Simple Publisher and Subscriber

Create a publisher node that publishes a counter value and a subscriber node that prints the received values.

### Steps:
1. Create a new ROS2 package called `my_robot_exercises`
2. Create a publisher node that publishes an integer counter every second
3. Create a subscriber node that subscribes to the counter and prints the value
4. Test the nodes using `ros2 run`

### Solution Outline:
```python
# publisher_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class CounterPublisher(Node):
    def __init__(self):
        super().__init__('counter_publisher')
        self.publisher = self.create_publisher(Int32, 'counter', 10)
        self.counter = 0
        self.timer = self.create_timer(1.0, self.publish_counter)
        
    def publish_counter(self):
        msg = Int32()
        msg.data = self.counter
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = CounterPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Exercise 2: Service Server and Client

Create a service that calculates the distance between two points.

### Steps:
1. Define a custom service message for point coordinates
2. Create a service server that calculates Euclidean distance
3. Create a service client that sends two points and receives the distance
4. Test the service using `ros2 service`

## Exercise 3: Action Server for Robot Movement

Create an action server that moves a robot to a specified goal position.

### Steps:
1. Define a custom action message for navigation
2. Create an action server that simulates moving to a goal
3. Provide feedback during the movement
4. Create an action client that sends navigation goals
5. Test the action using `ros2 action`

## Exercise 4: Parameter Server

Create a node that uses parameters to configure its behavior.

### Steps:
1. Create a node that declares parameters for robot speed and turning radius
2. Use the parameters to control the robot's movement
3. Change parameters at runtime using `ros2 param`
4. Implement parameter validation callbacks

## Exercise 5: Multi-node System

Create a complete system with multiple nodes that work together.

### Steps:
1. Create a sensor node that publishes simulated sensor data
2. Create a processing node that processes the sensor data
3. Create a decision node that makes decisions based on processed data
4. Create an actuator node that executes commands
5. Use launch files to start all nodes together
6. Test the complete system

## Challenge Exercise: TurtleBot3 Navigation

Implement a complete navigation system for a TurtleBot3 that:
- Uses laser scan data to detect obstacles
- Plans a path to a goal position
- Avoids obstacles while navigating
- Reports its status and position

import ChatInterface from '@site/src/components/chatbot/ChatInterface';

<ChatInterface chapterId="ros2-exercises" />