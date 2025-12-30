---
title: Gazebo and Unity Basics
sidebar_position: 2
---

# Gazebo and Unity Basics

## Overview
This chapter introduces the fundamental concepts of both Gazebo and Unity as simulation environments for robotics.

## Learning Objectives
After completing this chapter, you will:
- Understand the basic architecture of Gazebo and Unity
- Know how to set up simulation environments in both platforms
- Learn to create basic robot models in both environments
- Understand the physics engines used in each platform

## Gazebo Basics

### Architecture
Gazebo consists of:
- **Server**: Runs the simulation engine
- **Client**: Provides visualization and user interaction
- **Plugins**: Extend functionality (sensors, controllers, etc.)

### Basic Simulation Setup

To start a basic Gazebo simulation:

```bash
# Launch Gazebo with an empty world
gazebo

# Launch with a specific world file
gazebo my_world.world
```

### Creating a Simple Robot Model

A basic robot model in Gazebo uses the SDF (Simulation Description Format):

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="my_robot">
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
      </visual>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
    </link>
  </model>
</sdf>
```

## Unity Basics

### Architecture
Unity consists of:
- **Editor**: Development environment
- **Runtime**: Execution environment
- **Packages**: Extend functionality (including robotics packages)

### Setting up Unity for Robotics

1. Install Unity Hub and Unity Editor
2. Install the Unity Robotics Package
3. Set up ROS/ROS2 communication using ROS# or similar

### Basic Robot in Unity

Creating a simple robot in Unity involves:
- Creating GameObjects for robot parts
- Adding colliders and rigidbodies for physics
- Configuring joints to connect parts
- Adding sensors using Unity's sensor components

## Physics Engines

### Gazebo Physics
- **ODE**: Open Dynamics Engine (default)
- **Bullet**: Good balance of speed and accuracy
- **Simbody**: Multibody dynamics
- **DART**: Dynamic Animation and Robotics Toolkit

### Unity Physics
- **NVIDIA PhysX**: Industry-standard physics engine
- Optimized for real-time applications
- Supports complex interactions and constraints

## Comparison

| Aspect | Gazebo | Unity |
|--------|--------|-------|
| Physics Accuracy | High | Good |
| Graphics Quality | Basic | High |
| ROS Integration | Native | Requires Bridge |
| Learning Curve | Moderate | Steep |
| Use Case | Research/Development | Visualization/VR |

import ChatInterface from '@site/src/components/chatbot/ChatInterface';

<ChatInterface chapterId="gazebo-unity-basics" />