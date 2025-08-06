
# Cognibot

> Building a cognitive robot with ROS 2 and Unity

---

## Table of Contents

- [Overview](#overview)
- [Vision](#vision)
- [Features](#features)
- [Tech Stack](#tech-stack)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Development Goals](#development-goals)
- [Contributing](#contributing)

---

## Overview

**Cognibot** is an experimental robotics project for developing and testing cognitive architectures in simulated embodied agents. The goal is to incrementally build a cognitive robot that can perceive, reason, act, and adapt in a physically grounded environment.

The simulation environment is implemented in **Unity** for physics-rich interaction, while **ROS 2** handles the robot's modular control system, sensor integration, and middleware communication. The long-term aim is to enable **situated cognition**—embedding perception, memory, action, and decision-making within a real-time, embodied context.

---

## Vision

The Cognibot project explores one central question:

> **"What does it mean to give cognition to a robot that exists in the world?"**

This repository lays the foundation for:
- A modular, extensible cognitive architecture for robots
- Embodied interaction in a physically grounded 3D environment
- Continuous integration of higher-level cognitive capabilities (like planning, learning, internal simulation, and goal-driven behavior)

The robot is expected to evolve from simple reactive behaviors to more sophisticated, memory- and inference-driven autonomy.

---

## Features

- ✅ Unity-based 3D simulation with physics-enabled environments
- ✅ ROS 2 communication bridge for control and sensing

---

## Tech Stack

- **Simulation**: Unity 6.0 (6000.0.54f1) LTS with ROS TCP Connector and URDF Importer packages.
- **Middleware**: ROS 2 Jazzy on Ubuntu 24.04
- **Languages**: C#, Python, XML (URDF/Xacro)

---

## Installation

### Prerequisites

- Unity 6.0 LTS (or compatible)
- ROS 2 Jazzy (installed on Ubuntu 24.04)

### Clone Repository

```bash
git clone https://github.com/rasika-lokhande/cognibot.git --recurse-submodules
cd cognibot
```

---

## Usage

### In ROS 2

```bash
# Build and Source your ROS 2 workspace
cd cognibot # or path to the cognibot directory
colcon build
source install/setup.bash

# Launch 
ros2 launch cognibot_bringup  cognibot_unity.launch.xml
```

```bash
# Send velocity commands (to test)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### In Unity

1. Open the `cognibot_sim` Unity project in Unity Hub
2. Ensure the `ROSConnection` component is configured to match your ROS IP/port
3. Press **Play** to simulate
4. Use ROS to control the robot; Unity will simulate motion and physics

---

## Project Structure

```
cognibot/
├── ros2_ws/                            # ROS2 workspace
│   ├── src/
│   │   └── cognibot_description/       # URDF, xacro, display launch files
│   │   └── cognibot_bringup/           # Launch and config for ROS control
|   ├── .gitignore                      # gitignore file specific to ROS2 workspace
├── cognibot_sim/                       # Unity project folder
│   ├── Assets/                         # Unity Project Assets
|   ├── .gitignore                      # gitignore file specific to Unity project
├── docs/                               # Notes, design, cognitive architecture sketches
└── README.md
```

---

## Development Goals

The project follows an incremental roadmap:

### Phase 1: Reactive Robot (✔️ In Progress)

* Unity ↔ ROS bridge
* Differential drive base
* Basic sensor inputs (odom, IMU)
* Teleoperation and simple behaviors

### Phase 2: Architecture Integration

* Implement modular perception, memory, and action layers
* Encode beliefs, goals, and internal states
* Unity ↔ ROS ↔ Architecture message flow

### Phase 3: Prospective Cognition

* Goal-driven planning and execution
* Internal simulation of possible actions
* Interaction with dynamic environments

---

## Contributing

This is a personal research project but contributions, suggestions, or discussions are welcome! Please:

* Open an issue for bugs, feature ideas, or questions
* Submit a pull request with clear descriptions and testing instructions

---

