---
title: ROS 2 Fundamentals
sidebar_position: 1
---

# ROS 2 Fundamentals

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. Unlike traditional operating systems, ROS 2 is middleware that provides services designed specifically for robotic applications: hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.

## Learning Outcomes

By the end of this chapter, you will:
- Understand the architecture of ROS 2
- Create and run ROS 2 nodes using Python
- Implement publishers and subscribers for topics
- Design services for request-response communication
- Build action servers and clients for long-running tasks
- Use rclpy for Python-based ROS 2 development

## ROS 2 Architecture Overview

ROS 2 uses a distributed computing model where different processes (nodes) communicate through messages. The communication is managed by DDS (Data Distribution Service), which provides reliable, real-time messaging.

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Node A    │    │   Node B    │    │   Node C    │
│             │    │             │    │             │
│ Publisher   │    │ Subscriber  │    │ Service     │
│ /sensor_data│───▶│ /sensor_data│    │ Server      │
│             │    │             │    │ /move_base  │
└─────────────┘    └─────────────┘    └─────────────┘
                          ▲                   │
                          │                   │
                    ┌─────┴─────┐    ┌────────▼────────┐
                    │   DDS     │    │   Service       │
                    │Middleware │    │ Client          │
                    │           │    │ /move_base      │
                    └───────────┘    └─────────────────┘
```

## Nodes

Nodes are the fundamental components of any ROS program. Each node performs a specific function and communicates with other nodes through topics, services, or actions.

### Creating a Simple Node in Python

```python
import rclpy
from rclpy.node import Node

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

## Topics and Message Passing

Topics enable asynchronous, many-to-many communication. Publishers send messages to topics, and subscribers receive messages from topics.

### Topic Communication Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):

    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

class Listener(Node):

    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main():
    rclpy.init()

    talker = Talker()
    listener = Listener()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(talker)
    executor.add_node(listener)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services

Services provide synchronous, request-response communication. A client sends a request to a service server, which processes the request and returns a response.

### Service Server Example

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning: {response.sum}')
        return response

def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client Example

```python
import sys
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = int(sys.argv[1]) if len(sys.argv) > 1 else 41
        self.req.b = int(sys.argv[2]) if len(sys.argv) > 2 else 1
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request()
    minimal_client.get_logger().info(
        f'Result of add_two_ints: {response.sum}')

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions

Actions handle long-running tasks with feedback and goal management. They are ideal for navigation, manipulation, and calibration tasks.

### Action Server Example

```python
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from turtlesim.action import RotateAbsolute
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute


class RotateActionServer(Node):

    def __init__(self):
        super().__init__('rotate_action_server')

        self._action_server = ActionServer(
            self,
            RotateAbsolute,
            'rotate_absolute',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        self._srv = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')
        while not self._srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self._pose_sub = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.pose_callback,
            QoSProfile(depth=1))

        self.current_pose = None

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def pose_callback(self, msg):
        self.current_pose = msg

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = RotateAbsolute.Feedback()

        # Simulate rotation
        req = TeleportAbsolute.Request()
        req.x = self.current_pose.x
        req.y = self.current_pose.y
        req.theta = goal_handle.request.theta

        future = self._srv.call_async(req)

        while not future.done():
            feedback_msg.remaining_angle = abs(goal_handle.request.theta - self.current_pose.theta)
            goal_handle.publish_feedback(feedback_msg)

        result = RotateAbsolute.Result()
        result.delta = goal_handle.request.theta - self.current_pose.theta
        goal_handle.succeed()

        return result

def main():
    rclpy.init()
    rotate_action_server = RotateActionServer()

    executor = MultiThreadedExecutor()
    rclpy.spin(rotate_action_server, executor=executor)

    rotate_action_server.destroy()
    rclpy.shutdown()
```

## Parameters

Parameters provide a way to configure nodes dynamically. They can be set at launch time or changed during runtime.

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):

    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with defaults
        self.declare_parameter('motor_speed', 1.0)
        self.declare_parameter('timeout', 5)
        self.declare_parameter('debug_mode', False)

        # Access parameter values
        self.speed = self.get_parameter('motor_speed').value
        self.timeout = self.get_parameter('timeout').value
        self.debug = self.get_parameter('debug_mode').value

        self.get_logger().info(f'Initialized with speed: {self.speed}, timeout: {self.timeout}')

def main():
    rclpy.init()
    node = ParameterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch Files

Launch files allow you to start multiple nodes simultaneously with configurations.

```xml
<launch>
  <!-- Load parameters from YAML file -->
  <param name="use_sim_time" value="true" />

  <!-- Start the talker node -->
  <node pkg="demo_nodes_py" exec="talker" name="talker" output="screen">
    <param name="frequency" value="2.0" />
  </node>

  <!-- Start the listener node -->
  <node pkg="demo_nodes_py" exec="listener" name="listener" output="screen" />

  <!-- Start a service server -->
  <node pkg="demo_nodes_py" exec="add_two_ints_server" name="add_two_ints_server" />
</launch>
```

## Best Practices

1. **Node Design**: Keep nodes focused on single responsibilities
2. **Message Types**: Use standard message types when possible
3. **Naming Conventions**: Follow ROS naming conventions (`snake_case`)
4. **Error Handling**: Implement robust error handling and logging
5. **Resource Management**: Cleanly shut down resources when nodes terminate
6. **Testing**: Write unit tests for nodes and components

## Quiz

1. What is the primary difference between topics and services in ROS 2?
   A) Topics are faster than services
   B) Topics are asynchronous, services are synchronous
   C) Topics use more memory than services
   D) Services are deprecated in ROS 2

2. Which component manages communication between ROS 2 nodes?
   A) Node Manager
   B) Master
   C) DDS Middleware
   D) Parameter Server

3. What type of communication pattern is best for long-running tasks that provide feedback?
   A) Topics
   B) Services
   C) Actions
   D) Parameters

Answers: 1-B, 2-C, 3-C

## Summary

ROS 2 provides the communication infrastructure necessary for developing complex robotic applications. Understanding nodes, topics, services, and actions is fundamental to building robust, modular robot software. The rclpy library makes it easy to develop Python-based ROS 2 applications that interface with the broader ROS ecosystem.

Next, we'll explore how to simulate robots using Gazebo, building on these communication patterns to create virtual environments for testing and development.