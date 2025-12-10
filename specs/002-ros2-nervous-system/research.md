# Research: Module 1 - The Robotic Nervous System

**Feature**: `002-ros2-nervous-system`
**Date**: 2025-12-10

## Technical Decisions

### 1. ROS 2 Distribution
**Decision**: Use **ROS 2 Jazzy Jalisco**.
**Rationale**:
-   Released May 2024 with EOL May 2029 (covers the textbook's lifespan better than Humble's 2027 EOL).
-   Targeting humanoid robotics in 2025/2026 requires the latest features (Ubuntu 24.04 support, performance improvements).
-   "Physical AI" implies modern stacks; clinging to older LTS might limit integration with newer AI tools.
**Alternatives**:
-   *Humble Hawksbill*: More stable now, but effectively legacy by the time students finish the course.
-   *Rolling*: Too unstable for a textbook.

### 2. Code Presentation
**Decision**: Static, syntax-highlighted code blocks with titles and line highlighting.
**Rationale**:
-   Interactive ROS 2 (simulated in browser) is technically prohibitive without a heavy backend (e.g., The Construct).
-   Docusaurus native support for `python` blocks with `title="..."` and `{1-3}` highlighting is robust and accessible.
-   "Copy" button is built-in.
**Best Practices**:
-   Use `import rclpy` explicitly in every snippet.
-   Include `package.xml` and `setup.py` snippets where relevant to show the full build context.

### 3. Diagramming
**Decision**: Mermaid.js for architecture/flow; SVG/Images for complex visualizations (e.g., Rviz screenshots).
**Rationale**:
-   Mermaid is native to Docusaurus (via plugin).
-   Text-based diagrams are version-controllable and easy to edit.
-   Complex 3D visualizations (URDF meshes) cannot be rendered in Mermaid; will need placeholders or static images.

## Unknowns Resolved
-   **ROS 2 Version**: Jazzy confirmed.
-   **Dependencies**: `rclpy` is the primary focus; `rclcpp` mentioned for context but not the primary teaching language (Physical AI = Python-centric).
