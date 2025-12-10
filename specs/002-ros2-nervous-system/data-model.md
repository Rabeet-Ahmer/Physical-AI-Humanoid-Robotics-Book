# Data Model: Module 1 Content

**Feature**: `002-ros2-nervous-system`

## File System Structure

The "database" is the file system.

```text
docs/module-01-nervous-system/
├── 01-overview.md
├── 02-system-intuition.md
├── 03-theory-fundamentals.md
├── 04-architecture.md
├── 05-algorithms.md
├── 06-practical-applications.md
├── 07-review.md
├── 08-mini-project.md
└── 09-pitfalls.md
```

## Frontmatter Schema

Every MDX file MUST adhere to this YAML frontmatter schema:

```yaml
---
id: <string>           # Unique slug (e.g., "theory-fundamentals")
title: <string>        # H1 Title (e.g., "Theory & Fundamentals")
sidebar_label: <string> # Sidebar navigation text (e.g., "Theory")
sidebar_position: <int> # Ordering (1-9)
description: <string>  # Meta description for SEO (max 160 chars)
tags: <array<string>>  # e.g., [ros2, middleware, math]
---
```

## Page-Specific Models

### 01-overview.md
-   **Structure**:
    -   `# Module 1: The Robotic Nervous System`
    -   `## Concept Overview`
    -   `## Learning Objectives` (Bloom's Taxonomy)
    -   `## Prerequisites`

### 03-theory-fundamentals.md
-   **Structure**:
    -   `## Mathematical Foundations`
    -   `### Pub/Sub Algebra` (LaTeX)
    -   `### Coordinate Frames (TF2)`

### 08-mini-project.md
-   **Structure**:
    -   `## Task Description`
    -   `## Tools Required`
    -   `## Expected Output`
    -   `## Solution Walkthrough` (Collapsible)
