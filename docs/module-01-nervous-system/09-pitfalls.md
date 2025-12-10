---
id: pitfalls
title: Common Pitfalls
sidebar_label: Pitfalls
sidebar_position: 9
description: Common pitfalls and design trade-offs in ROS 2
tags: [ros2, pitfalls, trade-offs]
---

# Common Pitfalls & Design Trade-offs

## 1. Network Storms (DDS)
**Pitfall**: Publishing high-resolution images or point clouds at high frequency (e.g., 60Hz) over Wi-Fi.
**Result**: The network creates a bottleneck, causing latency spikes in critical control loops. The robot stutters or falls.
**Fix**: Use "Shared Memory" transport for intra-process communication (Zero Copy) or reduce data rates/resolution.

## 2. Callback Blocking
**Pitfall**: Doing heavy computation (e.g., `time.sleep(5)` or heavy ML inference) inside a subscriber callback.
**Result**: The node stops processing new messages. If this node controls balance, the robot falls.
**Fix**: Offload heavy tasks to a separate thread or use Asynchronous Actions.

## 3. Wrong Frame IDs
**Pitfall**: Publishing sensor data with the wrong `frame_id` in the header.
**Result**: The robot thinks obstacles are behind it instead of in front, leading to collisions.
**Fix**: Visualize TF frames in Rviz constantly to verify alignment.

## 4. Over-Engineering
**Pitfall**: Creating a separate node for every tiny function (e.g., "Add One Node").
**Result**: Excessive overhead from serialization/deserialization of messages.
**Fix**: Group tightly coupled logic into a single node or use Nodelets/Components.