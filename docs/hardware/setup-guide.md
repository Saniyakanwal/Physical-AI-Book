---
title: Hardware Setup Guide
sidebar_position: 3
---

# Hardware Setup Guide

## Overview
This guide provides instructions for setting up the hardware required for practical exercises in this course.

## Required Components

### Basic Setup
- Computer with Ubuntu 20.04 or 22.04 (or Windows with WSL2)
- At least 16GB RAM (32GB recommended)
- Multi-core processor (Intel i7 or equivalent AMD processor)
- Dedicated GPU with CUDA support (for NVIDIA Isaac platform)

### Recommended Robot Platform
- TurtleBot3 (or similar educational robot)
- Components:
  - Main controller (Raspberry Pi 4 or Jetson Nano)
  - Chassis and wheels
  - IMU sensor
  - Camera
  - LIDAR (e.g., LDS-01 or similar)

## Software Installation

### ROS2 Installation
```bash
# Add ROS2 GPG key and repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros-iron/raspberry-pi-os/ros-iron-raspberry-pi-os.gpg | sudo gpg --dearmor -o /usr/share/keyrings/ros-iron-raspberry-pi-os.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-iron-raspberry-pi-os.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Iron
sudo apt update
sudo apt install -y ros-iron-desktop
sudo apt install -y python3-colcon-common-extensions
```

### NVIDIA Isaac Setup
1. Install NVIDIA drivers (if using CUDA)
2. Install Isaac ROS packages:
```bash
sudo apt update
sudo apt install ros-iron-isaac-ros-*  # Install all Isaac ROS packages
```

## Hardware Assembly

### TurtleBot3 Assembly
1. Follow the official TurtleBot3 assembly guide
2. Connect all sensors to the main controller
3. Verify all connections before powering on
4. Test basic movement before proceeding

## Troubleshooting

### Common Issues
- **Robot not responding**: Check all cable connections
- **LIDAR not detected**: Verify power and data connections
- **Camera not working**: Check USB connections and permissions

### Verification Steps
1. Power on the robot
2. Connect to the robot's network
3. Test basic movement commands
4. Verify sensor data is being published

import ChatInterface from '@site/src/components/chatbot/ChatInterface';
import TranslationToggle from '@site/src/components/translation/TranslationToggle';

<TranslationToggle />
<ChatInterface chapterId="hardware-setup" />