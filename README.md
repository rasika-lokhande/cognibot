# Cognibot

> Exploring Common Coding Theory in Embodied Robotics with ROS 2 and Unity

---

## Table of Contents

- [Overview](#overview)
- [Vision](#vision)
- [Current Research Focus](#current-research-focus)
- [Features](#features)
- [Tech Stack](#tech-stack)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Research Goals](#research-goals)
- [Contributing](#contributing)

---

## Overview

**Cognibot** is a cognitive robotics research project exploring **Common Coding Theory** in embodied artificial agents. Unlike traditional robotics architectures that separate perception, planning, and action into distinct modules, this project investigates how shared spatial representations can unify perception and action planning.

The project implements the **Theory of Event Coding (TEC)** and **HiTEC** computational model principles in a simulated differential drive robot using **Unity** for physics simulation and **ROS 2** for cognitive architecture implementation.

---

## Vision

The Cognibot project explores a fundamental question in cognitive robotics:

> **"Can robots achieve more natural, adaptive behavior by using shared representations for perception and action, rather than traditional sense-plan-act pipelines?"**

This research investigates:
- **Common Coding Theory**: Spatial features that simultaneously represent perceptual information and action possibilities
- **Ideomotor Learning**: How robots can learn action-effect associations through experience
- **Embodied Spatial Cognition**: Action-oriented perception in real-time robotic systems
- **Perception-Action Coupling**: Integration of spatial awareness with movement planning

---

## Current Research Focus

### Common Coding Implementation

The project currently demonstrates **common coding principles** through:

- **Spatial Feature Codes**: Unified representations for spatial relationships (e.g., "ForwardClear", "LeftOpen", "RightOpen")
- **Shared Perception-Action Representations**: The same neural codes that process spatial perception also bias action selection
- **Competitive Feature Dynamics**: Attention-like mechanisms where spatial features compete for activation
- **Task-Driven Modulation**: How current goals influence spatial perception and action planning

### Key Insights Being Explored

- How spatial perception can be inherently action-oriented rather than descriptive
- Whether shared representations lead to more fluid, natural robotic behavior
- The role of attention and competition in spatial cognition
- Scalability of learned vs. hardcoded feature representations

---

## Features

- ✅ Unity-based 3D simulation with configurable LIDAR sensor
- ✅ ROS 2 communication bridge for real-time control and sensing
- ✅ **Common Coding Architecture**: Spatial feature processing with shared perception-action representations
- ✅ **Configurable LIDAR**: Adjustable scan range and resolution for spatial perception experiments
- ✅ **Feature Code Processing**: Real-time spatial relationship detection and activation
- ✅ **Competitive Dynamics**: Lateral inhibition between spatial features
- ✅ **Task Context Integration**: Goal-driven attention weighting of spatial features
- 🔄 **Ideomotor Learning**: Adaptive action-effect association learning (in development)

---

## Tech Stack

- **Simulation**: Unity 6.0 (6000.0.54f1) LTS with ROS TCP Connector
- **Middleware**: ROS 2 Jazzy on Ubuntu 24.04
- **Cognitive Architecture**: Python-based common coding implementation
- **Languages**: C# (Unity), Python (ROS 2), XML (URDF/Launch files)
- **Research Framework**: Based on Theory of Event Coding (TEC) and HiTEC computational model

---

## Installation

### Prerequisites

- Unity 6.0 LTS (or compatible)
- ROS 2 Jazzy (installed on Ubuntu 24.04)
- Python packages: numpy, rclpy

### Clone Repository

```bash
git clone https://github.com/rasika-lokhande/cognibot.git --recurse-submodules
cd cognibot
```

### Build ROS 2 Workspace

```bash
cd ros2_ws
colcon build --packages-select cognibot_interfaces cognibot_common_coding
source install/setup.bash
```

---

## Usage

### Running the Common Coding Experiment

```bash
# Terminal 1: Launch the cognitive architecture
ros2 launch cognibot_bringup cognibot_common_coding.launch.xml

# Terminal 2: Start the LIDAR feature processor
ros2 run cognibot_common_coding lidar_processor

# Terminal 3: Monitor spatial feature activations
ros2 topic echo /feature_codes

# Terminal 4: Optional - Monitor LIDAR data
ros2 topic echo /scan
```

### In Unity

1. Open the `cognibot_sim` Unity project
2. Configure LIDAR parameters in the Inspector:
   - **Start Angle**: -90° (standard 180° LIDAR)
   - **End Angle**: +90°
   - **Number of Rays**: 180
3. Press **Play** to start simulation
4. Observe real-time spatial feature activation based on robot's environment

### Experimental Scenarios

- **Corridor Navigation**: Test how the robot perceives and responds to narrow passages
- **Opening Detection**: Observe activation of "LeftOpen" and "RightOpen" features
- **Obstacle Avoidance**: Monitor "ForwardClear" feature in cluttered environments
- **Task Context Effects**: Compare feature activations under different goal contexts

---

## Project Structure

```
cognibot/
├── ros2_ws/                                    # ROS2 workspace
│   ├── src/
│   │   ├── cognibot_interfaces/                # Custom message definitions for feature codes
│   │   ├── cognibot_common_coding/             # Common coding implementation
│   │   │   ├── src/
│   │   │   │   ├── lidar_processor.py          # Spatial feature extraction and activation
│   │   │   │   ├── task_manager.py             # Task context and attention modulation
│   │   │   │   └── common_coding_controller.py # Integrated perception-action system
│   │   ├── cognibot_description/               # URDF, robot description
│   │   └── cognibot_bringup/                   # Launch files and configurations
├── cognibot_sim/                               # Unity simulation environment
│   ├── Assets/
│   │   ├── Scripts/
│   │   │   ├── LidarSensor.cs                  # Configurable LIDAR implementation
│   │   │   └── DifferentialDriveController.cs # Robot movement controller
│   │   └── Scenes/
│   │       └── CommonCodingTestEnvironment.unity # Test environment for experiments
├── docs/                                       # Research documentation
│   ├── common_coding_theory.md                 # Theoretical background
│   ├── experiment_results.md                   # Experimental findings
│   └── cognitive_architecture_design.md        # Architecture documentation
└── README.md
```

---

## Research Goals

### Current Phase: Common Coding Foundation (🔄 In Progress)

* ✅ Implement spatial feature codes with shared perception-action representations
* ✅ Demonstrate competitive dynamics between spatial features
* ✅ Configurable LIDAR sensor for spatial perception experiments
* 🔄 Task-driven attention and feature weighting
* 🔄 Ideomotor learning of action-effect associations

### Next Phase: Advanced Common Coding

* Hierarchical spatial feature development
* Learned vs. hardcoded feature emergence
* Multi-modal sensor integration (vision + LIDAR)
* Temporal sequence learning and prediction
* Complex environment navigation using common coding principles

### Future Phase: Cognitive Architecture Integration

* Memory integration with spatial feature codes
* Goal-driven task switching and attention
* Internal simulation using learned spatial representations
* Meta-cognitive monitoring of spatial perception-action coupling

---

## Research Context

This project is inspired by:

- **Theory of Event Coding (TEC)** by Hommel et al. (2001)
- **HiTEC computational model** by Haazebroek et al. (2011)
- **Ideomotor Theory** and action-effect learning principles
- **Embodied Cognition** research in cognitive science
- **Active Perception** and enactive approaches to robotics

### Publications and References

Key papers informing this implementation:
- Haazebroek, P., van Dantzig, S., & Hommel, B. (2011). A computational model of perception and action for cognitive robotics. *Cognitive Processing*, 12(4), 355-365.
- Hommel, B., Müsseler, J., Aschersleben, G., & Prinz, W. (2001). The theory of event coding (TEC): A framework for perception and action planning. *Behavioral and Brain Sciences*, 24(5), 849-878.

---

## Contributing

This is an active cognitive robotics research project. Contributions and collaborations are welcome!

**Research Interests:**
- Common coding theory implementation
- Spatial cognition and robotics
- Embodied artificial intelligence
- Perception-action coupling

**How to Contribute:**
- Open issues for theoretical discussions or implementation questions
- Submit pull requests with experimental improvements
- Share experimental results or alternative implementation approaches
- Suggest new test scenarios for common coding validation

**Research Collaboration:**
If you're interested in collaborative research on common coding or embodied cognition in robotics, please reach out through GitHub issues or email.

---

*"The same neural codes that help you perceive a coffee cup's handle also prepare your hand to grasp it. Can we build robots that work the same way?"*
