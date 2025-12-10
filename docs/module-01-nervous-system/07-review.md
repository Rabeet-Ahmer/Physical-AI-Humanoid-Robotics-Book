---
id: review
title: Review & Checkpoints
sidebar_label: Review
sidebar_position: 7
description: Module 1 Review and Checkpoints
tags: [ros2, review, summary]
---

# Review & Checkpoints

## Key Takeaways
*   **ROS 2** is the middleware that connects the robot's "brain" to its "body".
*   **Nodes** are the units of computation; **Topics** are the streams of data.
*   **URDF** is the file format defining the robot's physical structure.
*   **rclpy** is the Python library we use to interface with ROS 2.

## Conceptual Checkpoints
1.  **The Graph**: Can you draw the graph for a simple robot with a camera and a motor? (Camera Node -> `/image` -> Processing Node -> `/cmd_vel` -> Motor Node).
2.  **Transforms**: Why do we need `tf2`? (To convert data from the sensor frame to the base frame).
3.  **QoS**: When would you use "Best Effort" reliability? (For high-frequency sensor data like video streams where dropping a frame is better than lagging).

## Next Steps
In Module 2, we will take these concepts and build a **Digital Twin** simulation, putting our URDF and ROS nodes to the test in a physics engine.