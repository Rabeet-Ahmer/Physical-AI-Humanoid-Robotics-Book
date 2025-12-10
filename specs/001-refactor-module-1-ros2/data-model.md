# Data Model: Refactor Module 1 ROS 2

**Date**: 2025-12-11

## Entities

### 1. Module 1 Documentation (Root Entity)
- **Description**: Represents the entire collection of documentation files for "The Robotic Nervous System (ROS 2)" module within Docusaurus. Its structure is designed to strictly emulate Module 2.
- **Attributes**:
    - `root_path`: `/docs/module-01-nervous-system/` (String) - The base directory for Module 1.
    - `_category_.json`: (JSON) - Docusaurus category configuration for the module root, defining its label and position in the sidebar.

### 2. Content Section (Markdown File)
- **Description**: Individual Markdown files that contain the actual educational content for specific topics within Module 1. These will be organized hierarchically under categories.
- **Attributes**:
    - `file_name`: (String) - Adheres to Module 2's file naming conventions (e.g., `XX-topic-name.md`).
    - `path`: (String) - Relative path from `Module 1 Documentation` root.
    - `content`: (Markdown String) - The actual text, code, and other elements of the documentation.
    - `front_matter`: (YAML/JSON) - Docusaurus-specific metadata within the Markdown file (e.g., `title`, `slug`, `sidebar_label`).

### 3. Docusaurus Category (Directory)
- **Description**: Directories within Module 1 that group related Content Sections or nested Categories, mirroring Module 2's hierarchical organization. Each category will have an associated `_category_.json` file.
- **Attributes**:
    - `directory_name`: (String) - The name of the directory (e.g., `concepts`).
    - `path`: (String) - Relative path from `Module 1 Documentation` root.
    - `_category_.json`: (JSON) - Configuration for the category, defining its label, position, and potentially link behavior in the sidebar.

## Relationships

### 1. Module 1 Documentation to Docusaurus Categories / Content Sections
- **Type**: Contains
- **Description**: The `Module 1 Documentation` entity contains one or more `Docusaurus Category` entities and/or `Content Section` entities directly under its root.

### 2. Docusaurus Category to Nested Docusaurus Categories / Content Sections
- **Type**: Contains
- **Description**: A `Docusaurus Category` can contain nested `Docusaurus Category` entities and/or `Content Section` entities, forming the hierarchical structure.

## Content Distribution (Example based on user input)

The user-provided Module 1 content will be distributed into the following initial `Content Sections` (Markdown files), adhering to Module 2's likely file naming conventions and category structure:

- **Module 1 Overview/Introduction**:
    - `01-overview.md` (or similar, depending on Module 2's pattern)
    - Content: "Module 1: The Robotic Nervous System (ROS 2) Focus: Middleware for robot control."

- **ROS 2 Core Concepts**:
    - `02-ros2-nodes-topics-services.md` (or similar, within a 'concepts' category if Module 2 has one)
    - Content: "ROS 2 Nodes, Topics, and Services."

- **Python Bridging**:
    - `03-python-agents-rclpy.md` (or similar, within an 'applications' or 'programming' category)
    - Content: "Bridging Python Agents to ROS controllers using rclpy."

- **Robot Description Formats**:
    - `04-urdf-humanoids.md` (or similar, within a 'robot-modeling' or 'fundamentals' category)
    - Content: "Understanding URDF (Unified Robot Description Format) for humanoids."
