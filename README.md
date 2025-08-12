[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)]() [![Unity](https://img.shields.io/badge/Unity-6.0-green)]() [![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-orange)]()

# Cognibot

> Traditional robots struggle with dynamic, unstructured environments because they separate perception ("what do I see?") from action ("what should I do?"). Cognitive science suggests biological agents succeed because they use shared representations—the same neural codes for seeing and acting. This project explores whether robots can achieve similar adaptability.

---

## Table of Contents

- [Overview](#overview)
- [Vision](#vision)
- [Tech Stack](#tech-stack)
- [Installation](#installation)
- [Usage](#usage)
- [Current Implementation Status](#current-implementation-status)
- [Research Goals](#research-goals)
- [Contributing](#contributing)

---

## Overview

**Cognibot** is a cognitive robotics project exploring **Theory of Event Coding** in embodied artificial agents. Unlike traditional robotics architectures that separate perception, planning, and action into distinct modules, this project investigates how shared representations can unify perception and action planning. 

The project implements the **Theory of Event Coding (TEC)** and **HiTEC** computational model principles in a simulated differential drive robot using **Unity** for physics simulation and **ROS 2** for cognitive architecture implementation.

---

## Vision

This project will (eventually) attempt to explore the following concepts:
- **Shared Perception-Action Representations**: Features that simultaneously represent perceptual information and action possibilities
- **Ideomotor Learning**: How robots can learn action-effect associations through experience
- **Active Perception**: How robots can use sensorimotor interactions to actively explore and learn about their environment
- **Task driven modulation**: How current goals influence event coding
- **Reward driven modulation**: How rewards and internal drives can modulate event coding
- **Semantic Grounding**: How can meaning be generated through these sensorimotor associations?
- **Attention driven modulation**: How can attention mechanisms influence event coding?
- **Hierarchical Feature Development**: How can robots learn complex features through hierarchical representations?
- **Multi-modal Integration**: How can robots integrate multiple sensor modalities (e.g., vision, LIDAR) into a unified coding framework?
- **Metacognitive Monitoring**: How can robots monitor their own cognitive processes and adapt their behavior accordingly?

---

## Tech Stack

- **Simulation**: Unity 6.0 (6000.0.54f1) LTS with ROS TCP Connector, URDF Importer 
- **Middleware**: ROS 2 Jazzy on Ubuntu 24.04
- **Languages**: C# (Unity), Python (ROS 2), XML (URDF/Launch files)

---

## Installation

### Prerequisites

- Unity 6.0 LTS (or compatible) with ROS TCP Connector package (See: [ROS TCP Connector](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/README.md) )
- ROS 2 Jazzy (installed on Ubuntu 24.04)


### Clone Repository

```bash
git clone https://github.com/rasika-lokhande/cognibot.git 
```

### Build ROS 2 Workspace

```bash
cd cognibot/ros2_ws
colcon build 
source install/setup.bash
```

---

## Usage

### In Unity

1. Open the `cognibot_sim` Unity project
2. Configure ROS Connection GameObject
    - Set **ROS IP** to your ROS 2 machine's IP address



### Running simulation

1. Press Play in Unity to start the simulation

2. Launch the ROS 2 nodes to connect Unity with the cognitive architecture
```bash
# Terminal 1: Launch the cognitive architecture
cd cognibot/ros2_ws
source install/setup.bash
ros2 launch cognibot_bringup cognibot_unity.launch.xml 
# This launches the ROS 2 nodes, connects to Unity, and opens Rviz for visualization
```
3. Monitor feature activations 
```bash
# Terminal 2: Monitor feature activations
ros2 topic echo /feature_codes
```

---

## Project Structure

```
cognibot/
├── ros2_ws/                                    # ROS2 workspace
│   ├── src/
│   │   ├── cognibot_interfaces/                # Custom message definitions for feature codes
│   │   ├── common_coding/                      # Common coding implementation
│   │   │   ├── src/
│   │   │   │   ├── lidar_processor.py          # Spatial feature extraction and activation
│   │   │   │   └── action_executor.py          # Execute motor commands based on feature codes
│   │   ├── cognibot_description/               # URDF, robot description
│   │   └── cognibot_bringup/                   # Launch files and configurations
├── cognibot_sim/                               # Unity simulation environment
│   ├── Assets/
│   │   ├── Scripts/
│   │   │   ├── LidarSensor.cs                  # Configurable LIDAR implementation
│   │   │   └── AGVController.cs                 # Robot movement controller
└── README.md
```

---


## Current Capabilities
- **Obstacle Avoidance**: A simple mobile robot navigates cluttered environments using spatial feature codes
- **Feature Activation**: Real-time logs of which perceptual features drive behavior
- **Sensorimotor Integration**: Direct coupling between LIDAR perception and motor actions


### ✅ Completed
- [x] ROS 2-Unity integration: Real-time sensor/actuator communication
- [x] Basic URDF model: Robot description for Unity simulation
- [x] Basic robot controller: Differential drive robot with Unity physics
- [x] LIDAR sensor simulation: Configurable LIDAR with Unity
- [x] Feature code generation: Spatial feature extraction from LIDAR data
- [x] Feature activation: Real-time feature activation logging
- [x] Motor command execution: Basic motor commands based on feature codes
- [x] ROS 2 message definitions: Custom messages for feature codes
- [x] Basic cognitive architecture: ROS 2 nodes for feature processing and action execution
- [x] Reactive navigation: Obstacle avoidance using feature activation

### 🔄 In Progress
- [ ] HiTEC neural dynamics: Competition, lateral inhibition, activation equations
- [ ] Task context system: Goal-directed attention and feature modulation

### 📋 Next steps
- [ ] Motor babbling phase: Random exploration for action-effect discovery
- [ ] Ideomotor learning: Weight adaptation through action-effect associations
---


## References

**Key papers informing this implementation:**
- Haazebroek, P., van Dantzig, S., & Hommel, B. (2011). A computational model of perception and action for cognitive robotics. *Cognitive Processing*, 12(4), 355-365.
- Hommel, B., Müsseler, J., Aschersleben, G., & Prinz, W. (2001). The theory of event coding (TEC): A framework for perception and action planning. *Behavioral and Brain Sciences*, 24(5), 849-878.

---

## Contributing

This is an active cognitive robotics experimental project. Contributions and collaborations are welcome!

**How to Contribute:**
- Open issues for theoretical discussions or implementation questions
- Submit pull requests with experimental improvements
- Share alternative implementation approaches
- **Most importantly, critique the implementation and suggest improvements!**

**Collaboration:**
If you're interested in collaborative work on cognitive architectures in robotics, please reach out through GitHub issues or email.

---

*"The same neural codes that help you perceive a coffee cup's handle also prepare your hand to grasp it. Can we build robots that work the same way?"*
