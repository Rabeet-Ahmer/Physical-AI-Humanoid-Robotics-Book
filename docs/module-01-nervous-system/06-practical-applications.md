---
id: practical-applications
title: Practical Applications
sidebar_label: Practical Applications
sidebar_position: 6
description: Real-world applications of ROS 2 in Humanoid Robotics
tags: [ros2, applications, humanoid]
---

# Practical Applications

## Industry Use Cases

### 1. Tesla Optimus
While Tesla uses a custom stack, the architecture mirrors ROS 2 concepts: distributed modules for vision, planning, and control communicating over a high-speed bus.

### 2. NASA Valkyrie
Used ROS 1/2 for high-level task planning and teleoperation. The modularity allowed researchers from different universities to contribute code for specific limbs without needing the full robot hardware.

### 3. Agile Mobile Robots (Digit)
Uses ROS 2 for navigation and high-level decision making, interfacing with a real-time OS (RTOS) for low-level balance control.

## Research Applications
*   **Sim-to-Real**: Training RL policies in Isaac Sim (which has a ROS 2 bridge) and deploying them to a physical robot seamlessly because the message interfaces are identical.
*   **Swarm Robotics**: Coordinating multiple humanoids using ROS 2's native discovery protocol—robots automatically "see" each other on the network.