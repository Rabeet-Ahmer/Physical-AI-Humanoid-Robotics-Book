# Data Model: AI Robot Brain & Vision-Language-Action Modules

**Branch**: `001-modules-ai-vla` | **Date**: 2025-12-10 | **Plan**: [specs/001-modules-ai-vla/plan.md](specs/001-modules-ai-vla/plan.md)

This data model describes the logical structure of the content for the AI Robot Brain and Vision-Language-Action modules within the textbook.

---

### Entity: Module

Represents a top-level organizational unit of the textbook, grouping related topics.

**Attributes**:
-   **ID**: Unique identifier (e.g., `module-03-ai-brain`).
-   **Title**: Display name of the module (e.g., "Module 3: The AI-Robot Brain (NVIDIA Isaac™)").
-   **Description**: A brief overview of the module's focus.
-   **Order**: Numeric sequence for display in the sidebar.
-   **Topics**: A collection of `Topic` entities belonging to this module.

**Relationships**:
-   Has many `Topic`s.

---

### Entity: Topic

Represents a specific learning concept or area within a `Module`. Each topic follows a consistent structural template.

**Attributes**:
-   **ID**: Unique identifier (e.g., `02-nvidia-isaac-sim`).
-   **Title**: Display name of the topic (e.g., "NVIDIA Isaac Sim: Photorealistic simulation and synthetic data generation").
-   **FilePath**: Relative path to the Markdown/MDX file containing the topic's content (e.g., `docs/module-03-ai-brain/02-nvidia-isaac-sim.md`).
-   **Order**: Numeric sequence for display within its parent module's sidebar.
-   **SubSections**: A collection of predefined content sections that make up the topic.

**Relationships**:
-   Belongs to one `Module`.
-   Contains many `SubSection`s.
-   May contain `MermaidDiagram`s and `CodeExample`s within its `SubSection`s.
-   May contain one `MiniProjectLab`.

---

### Entity: SubSection

Represents a standardized content block within a `Topic`.

**Attributes**:
-   **Type**: Predefined type of section (e.g., "Concept Overview", "System-Level Intuition", "Theory & Fundamentals", "Diagrams", "Code Examples", "Mini Project / Lab", "Review & Checkpoints", "Further Reading").
-   **Content**: The actual Markdown/MDX text, potentially including embedded diagrams, code blocks, and LaTeX.
-   **Mandatory**: Boolean indicating if this subsection is required for all topics (e.g., "Diagrams", "Code Examples").

**Relationships**:
-   Belongs to one `Topic`.

---

### Entity: MermaidDiagram

Represents a diagram embedded within a `SubSection` using Mermaid syntax.

**Attributes**:
-   **Syntax**: The Mermaid textual definition of the diagram.
-   **Description**: A textual explanation of the diagram's purpose or content.
-   **Location**: The specific `SubSection` where the diagram is embedded.

**Relationships**:
-   Is contained within a `SubSection`.

---

### Entity: CodeExample

Represents a block of executable code embedded within a `SubSection`.

**Attributes**:
-   **Language**: Programming language (e.g., Python, ROS, PyTorch, Mujoco, Isaac Sim).
-   **Code**: The actual source code.
-   **Context**: Explanation of the code's purpose, how it relates to the theory, and its practical application.
-   **Dependencies**: Required libraries or packages.
-   **Location**: The specific `SubSection` where the code is embedded.

**Relationships**:
-   Is contained within a `SubSection`.

---

### Entity: MiniProjectLab

Represents a hands-on exercise designed to reinforce learning for a specific `Topic`.

**Attributes**:
-   **TaskDescription**: Detailed instructions for the student.
-   **ExpectedOutput**: Description of what the student should achieve or observe.
-   **ToolsRequired**: List of software/hardware tools necessary to complete the project.
-   **LearningObjectivesTested**: Reference to specific learning objectives addressed by the project.
-   **Location**: The specific `SubSection` where the mini-project is described.

**Relationships**:
-   Is contained within a `SubSection` (specifically the "Mini Project / Lab" subsection).
-   Belongs to one `Topic`.
