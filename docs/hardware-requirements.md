---
id: hardware-requirements
sidebar_position: 3
title: Hardware Requirements
---

# Hardware Requirements

To successfully complete this course, you will need access to specific hardware for simulation and physical deployment.

## 1. Digital Twin Workstation (Required)
A powerful workstation is essential for running physics simulations (Isaac Sim/MuJoCo) and training models.

*   **GPU**: NVIDIA RTX 4070 Ti Super (16GB VRAM) or better.
    *   *Minimum*: RTX 3080 (10GB+ VRAM).
    *   *Recommended*: RTX 4090 (24GB VRAM).
*   **CPU**: Intel Core i9-14900K or AMD Ryzen 9 7950X.
*   **RAM**: 64GB DDR5 (Minimum 32GB).
*   **Storage**: 2TB NVMe SSD (Gen 4/5).
*   **OS**: Ubuntu 22.04 LTS (Dual boot recommended).

## 2. Physical AI Edge Kit (Edge Deployment)
For deploying models to the robot.

*   **Compute Module**: NVIDIA Jetson Orin Nano (8GB) or AGX Orin (for VLA).
*   **Camera**: Intel RealSense D435i or D405 (Depth + RGB).
*   **Power Supply**: 19V DC Power supply or Battery pack (XT60).

## 3. Robot Lab Options
Choose one of the following setups for the physical labs.

### Option A: The Proxy (Low Cost)
*   **Arm**: WidowX 250s (Interbotix) or equivalent 6-DOF arm.
*   **Gripper**: Parallel jaw gripper.
*   **Teleop**: Leader arm (WidowX or similar) or VR Controller (Quest 3).

### Option B: The Miniature (Medium Cost)
*   **Robot**: K-Scale Open Source Humanoid (miniature).
*   **Servos**: Dynamixel XL330/430 series.

### Option C: Premium (Research Lab)
*   **Robot**: Unitree H1 or Fourier GR-1 (Full size).
*   **Teleop**: ALOHA Stationary setup with ViperX 300s arms.