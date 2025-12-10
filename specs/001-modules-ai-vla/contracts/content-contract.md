# Content Contract: Topic Markdown Files

**Branch**: `001-modules-ai-vla` | **Date**: 2025-12-10 | **Plan**: [specs/001-modules-ai-vla/plan.md](specs/001-modules-ai-vla/plan.md)

This document specifies the required structure and content for each Markdown/MDX file representing a topic within Module 3 and Module 4 of the textbook. Adherence to this contract ensures consistency, pedagogical clarity, and Docusaurus compatibility.

---

### File Naming Convention

-   Files MUST be named in a sequential `NN-topic-name.md` or `NN-topic-name.mdx` format (e.g., `01-course-introduction.md`).
-   The `NN` prefix dictates the order in the Docusaurus sidebar.

### Frontmatter

Each Markdown/MDX file MUST begin with Docusaurus frontmatter, including at least:

```yaml
---
title: [Topic Title]
description: [Brief description of the topic]
slug: /[module-slug]/[topic-slug]
tags: [tag1, tag2]
---
```

### Mandatory Sub-sections (Headers)

Each topic file MUST include the following level 2 headers (`##`), in the exact order specified, without skipping any:

1.  `## Concept Overview`
    -   Define the topic clearly.
    -   Explain why it matters in Physical AI.
    -   Connect it to humanoid robotics.

2.  `## System-Level Intuition`
    -   Big-picture explanation.
    -   How this fits into a humanoid system.
    -   Analogies (biological + engineering).

3.  `## Theory & Fundamentals`
    -   Mathematical foundations (LaTeX where needed, e.g., `$E=mc^2$`).
    -   Models and frameworks.
    -   Key equations and assumptions.

4.  `## Architecture & Components`
    -   Explain: Subsystems, Pipelines, Data flow.
    -   Hardware–software interaction.

5.  `## Diagrams (MANDATORY)`
    -   MUST contain at least one Mermaid diagram.
    -   Example:
        ```mermaid
        graph TD
            A[Start] --> B{Process};
            B --> C{Decision};
            C -- Yes --> D[End];
            C -- No --> B;
        ```

6.  `## Algorithms & Models`
    -   Step-by-step explanation.
    -   Include pseudocode where relevant.

7.  `## Code Examples (MANDATORY)`
    -   MUST provide real code (Python / ROS / PyTorch / Mujoco / Isaac Sim).
    -   Code blocks MUST specify language (e.g., `` ```python ``).

8.  `## Practical Applications`
    -   Real humanoid robots.
    -   Industry use-cases.
    -   Research applications.

9.  `## Common Pitfalls & Design Trade-offs`
    -   Engineering challenges.
    -   Performance vs realism.
    -   Hardware constraints.

10. `## Mini Project / Lab`
    -   Task description.
    -   Expected output.
    -   Tools required.

11. `## Review & Checkpoints`
    -   Bullet-point recap.
    -   Conceptual checkpoints.

12. `## Further Reading`
    -   Paper, Books, Open-source projects.

### Content Guidelines

-   **Formatting Consistency**: Preserve overall formatting consistency, including heading levels, admonitions, and code block styles.
-   **LaTeX**: Use `$LaTeX$` for inline math and `$$LaTeX$$` for display math.
-   **Citations**: Use APA 7th Edition format for all academic claims.
-   **Executable Code**: All code snippets should be valid and executable, with appropriate environment/dependency notes.
-   **Admonitions**: Use Docusaurus admonitions (`:::note`, `:::tip`, `:::warning`, `:::danger`) where appropriate.

### Versioning

Changes to this content contract will follow the project's Constitution amendment process.
