# Research: Module 2: The Digital Twin

**Status**: In Progress
**Date**: 2025-12-10

## Unknowns & Clarifications

### 1. Docusaurus v3+ Best Practices
- **Goal**: Verify latest syntax for Admonitions, MDX components, and LaTeX math rendering.
- **Decision**: Use standard Docusaurus v3 admonitions (`:::note`, etc.) and KaTeX for math (`$$`).
- **Rationale**: Constitution Principle IV mandates strict Docusaurus compatibility.

### 2. ROS 2 & Gazebo Versions
- **Goal**: Confirm the recommended ROS 2 and Gazebo combination for a 2025 textbook.
- **Decision**: Target ROS 2 Jazz (if stable) or Iron/Humble with Gazebo Harmonic.
- **Rationale**: Need to balance stability with recency (Constitution Principle II).

### 3. Unity for Robotics
- **Goal**: Identify standard packages for ROS-Unity integration.
- **Decision**: Focus on `Unity Robotics Hub` packages (URDF Importer, ROS-TCP-Connector).
- **Rationale**: Industry standard for Unity-ROS communication.

### 4. Sensor Simulation Best Practices
- **Goal**: precise configuration for LiDAR/Camera in Gazebo vs Unity.
- **Decision**: Use Gazebo `gpu_ray` for LiDAR (performance) and Unity Sensors package for high-fidelity visual data.
- **Rationale**: Performance vs. Realism trade-offs (Constitution Principle III).

## Technology Choices

| Technology | Choice | Rationale |
| :--- | :--- | :--- |
| **Documentation** | Docusaurus v3 | Existing project standard, supports MDX/React. |
| **Physics Sim** | Gazebo Harmonic | Latest LTS-aligned, high physics fidelity. |
| **Rendering** | Unity 6 / 2023 LTS | High-fidelity visuals, strong ROS integration. |
| **ROS Version** | ROS 2 Jazzy | Latest LTS (May 2024 release), ensures longevity. |
