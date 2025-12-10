---
id: mini-project
title: Mini Project
sidebar_label: Mini Project
sidebar_position: 8
description: Module 1 Mini Project - ROS 2 Implementation
tags: [ros2, project, hands-on]
---

# Mini Project: The "Blinking" Robot

## Task Description
Create a ROS 2 system that simulates a robot "blinking" its LED eyes.
1.  Create a node named `eye_controller`.
2.  Publish a boolean message to topic `/eyes/blink_cmd` at 1Hz (blink every second).
3.  Create a second subscriber node named `face_display`.
4.  When it receives `True`, print "O_O" (Open) to the console.
5.  When it receives `False`, print "-_-" (Closed).

## Tools Required
-   ROS 2 (Humble or Jazzy)
-   Python 3.10+
-   Terminal

## Expected Output
Your terminal should show alternating logs every second:

```bash
[INFO] [face_display]: O_O
[INFO] [face_display]: -_-
[INFO] [face_display]: O_O
```

## Solution Walkthrough

<details>
<summary>Click to see the solution code</summary>

### 1. The Eye Controller (Publisher)

```python title="eye_controller.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class EyeController(Node):
    def __init__(self):
        super().__init__('eye_controller')
        self.publisher_ = self.create_publisher(Bool, '/eyes/blink_cmd', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.is_open = True

    def timer_callback(self):
        msg = Bool()
        msg.data = self.is_open
        self.publisher_.publish(msg)
        self.is_open = not self.is_open # Toggle state

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(EyeController())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. The Face Display (Subscriber)

```python title="face_display.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class FaceDisplay(Node):
    def __init__(self):
        super().__init__('face_display')
        self.subscription = self.create_subscription(
            Bool, 
            '/eyes/blink_cmd', 
            self.listener_callback, 
            10)

    def listener_callback(self, msg):
        if msg.data:
            self.get_logger().info('O_O')
        else:
            self.get_logger().info('-_-')

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(FaceDisplay())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
</details>