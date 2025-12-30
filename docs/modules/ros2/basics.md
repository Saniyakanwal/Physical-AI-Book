---
title: ROS2 Basics
sidebar_position: 2
---

# ROS2 Basics

## Overview
This chapter covers the fundamental concepts of Robot Operating System 2 (ROS2).

## Learning Objectives
After completing this chapter, you will:
- Understand the architecture of ROS2
- Know how to create and run basic ROS2 nodes
- Understand topics, services, and actions in ROS2
- Learn about ROS2 launch files and parameters

## What is ROS2?

ROS2 is the next generation of the Robot Operating System, designed to address the limitations of ROS1 and provide a more robust, scalable, and production-ready framework for robotics development.

### Key Differences from ROS1
- Improved real-time support
- Better security features
- Quality of Service (QoS) policies
- Multi-robot systems support
- Lifecycle management

## Core Concepts

### Nodes
Nodes are the fundamental building blocks of a ROS2 system. Each node is a process that performs computation.

### Topics and Messages
Topics are named buses over which nodes exchange messages. Messages are the data packets sent over topics.

### Services
Services provide a request/reply communication pattern between nodes.

### Actions
Actions are a more complex communication pattern for long-running tasks with feedback.

## Practical Example

Here's a simple ROS2 publisher node:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

import ChatInterface from '@site/src/components/chatbot/ChatInterface';
import PersonalizationToggle from '@site/src/components/personalization/PersonalizationToggle';

<PersonalizationToggle />
<ChatInterface chapterId="ros2-basics" />