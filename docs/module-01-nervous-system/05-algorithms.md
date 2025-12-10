---
id: algorithms
title: Algorithms & Models
sidebar_label: Algorithms
sidebar_position: 5
description: ROS 2 Algorithms and Models
tags: [ros2, algorithms, models]
---

# Algorithms & Models

## The Control Loop
The fundamental algorithm in robotics is the Control Loop, running inside a ROS 2 node.

**Algorithm: Standard Feedback Loop**
1.  **Initialize** node and publishers/subscribers.
2.  **Loop** at a fixed frequency (e.g., 100Hz):
    a.  **Read** sensor state ($S_t$).
    b.  **Calculate** error ($E_t = Goal - S_t$).
    c.  **Compute** control output ($U_t$) using a control law (e.g., PID).
    d.  **Publish** $U_t$ to motor topic.
    e.  **Sleep** to maintain frequency.

```python title="feedback_loop.py"
def control_loop(self):
    # 1. Read Sensor
    current_pos = self.sensor.read()
    
    # 2. Calculate Error
    error = self.target_pos - current_pos
    
    # 3. Compute Control (P-Controller)
    kp = 1.5
    control_out = kp * error
    
    # 4. Publish
    msg = Float64()
    msg.data = control_out
    self.publisher_.publish(msg)
```

## Asynchronous Handling
ROS 2 nodes are asynchronous by default.
*   **Callbacks**: Functions that trigger *only* when a message arrives.
*   **Executors**: The scheduler that manages the queue of callbacks. For humanoids, we often use **MultiThreadedExecutor** to allow parallel processing of vision and proprioception data.

## State Estimation (Fusion)
How do we know the robot is standing up?
*   **Algorithm**: Extended Kalman Filter (EKF).
*   **Inputs**: IMU (Acceleration/Gyro), Leg Odometry (Joint positions).
*   **Process**: Fuses noisy sensor data into a single, robust estimate of the robot's orientation and position relative to the world.
*   **ROS 2 Tool**: `robot_localization` package.