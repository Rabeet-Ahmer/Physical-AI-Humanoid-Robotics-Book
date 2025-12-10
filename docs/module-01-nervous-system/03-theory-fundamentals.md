---
id: theory-fundamentals
title: Theory & Fundamentals
sidebar_label: Theory & Fundamentals
sidebar_position: 3
description: Mathematical foundations and theory of ROS 2
tags: [ros2, math, theory, pub-sub]
---

# Theory & Fundamentals

## Pub/Sub Algebra
The core of ROS 2 is the Publish/Subscribe pattern. It decouples data producers from consumers.

Let $N$ be the set of all nodes in the system.
Let $T$ be the set of all topics.

A node $n_i \in N$ can publish to a topic $t_j \in T$:
$$
P(n_i, t_j) \rightarrow \text{messages}
$$

A node $n_k \in N$ can subscribe to topic $t_j$:
$$ 
S(n_k, t_j) \leftarrow \text{messages}
$$ 

Key property: $n_i$ does not know $n_k$ exists. They only know $t_j$.

## Coordinate Frames (TF2)
Robots exist in physical space. We define this space using Coordinate Frames.
*   **World Frame ($W$)**: Fixed global origin (e.g., the floor).
*   **Base Frame ($B$)**: The robot's center (e.g., pelvis).
*   **End-Effector Frame ($E$)**: The robot's hand.

We represent the position and orientation of the hand relative to the base as a transformation matrix $T_{BE}$:

$$ 
T_{BE} = \begin{bmatrix} 
 R_{3 \times 3} & p_{3 \times 1} \ 
 0_{1 \times 3} & 1 
\end{bmatrix}
$$ 

Where $R$ is the rotation matrix and $p$ is the position vector.

ROS 2 uses the **tf2** library to manage these transforms over time. It allows us to ask questions like: "If the camera sees an object at $(x, y, z)$ in the *Camera Frame*, where is it in the *Base Frame*?"

$$ 
P_{Base} = T_{Base \leftarrow Camera} \cdot P_{Camera}
$$ 


## Minimal Node Structure

Here is the skeleton of a Python ROS 2 node using `rclpy`.

```python title="minimal_node.py"
import rclpy

from rclpy.node import Node



class HumanoidBrain(Node):

    def __init__(self):

        super().__init__('humanoid_brain')

        self.get_logger().info('Brain Node Started')
        

        # logic goes here


def main(args=None):

    rclpy.init(args=args)

    node = HumanoidBrain()

    rclpy.spin(node) # Blocks and keeps node alive

    node.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':

    main()
```