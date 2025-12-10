# Data Model: Module 2: The Digital Twin

## Entities

### 1. Topic (Markdown File)
Represents a single lesson or chapter within the module.

| Field | Type | Description |
| :--- | :--- | :--- |
| `id` | string | Unique identifier (slug), e.g., `physics-simulation-gazebo` |
| `title` | string | Display title, e.g., "Physics Simulation in Gazebo" |
| `sidebar_position` | integer | Ordering in the navigation sidebar |
| `description` | string | Brief summary for SEO and previews |
| `tags` | array<string> | Keywords for search and grouping |

### 2. Code Example
Represents an executable code snippet embedded in a topic.

| Field | Type | Description |
| :--- | :--- | :--- |
| `language` | string | Programming language (python, cpp, xml, etc.) |
| `title` | string | Filename or descriptive title |
| `content` | text | The actual code body |
| `github_source` | url | (Optional) Link to the full source file in the repo |

### 3. Diagram
Represents a visual explanation using Mermaid or an image.

| Field | Type | Description |
| :--- | :--- | :--- |
| `type` | enum | `mermaid`, `image` |
| `caption` | string | Descriptive caption for accessibility |
| `source` | text/url | Mermaid code or image path |

## Relationships

- **Module** (Folder) *contains* 1..N **Topics** (Files)
- **Topic** *contains* 0..N **Code Examples**
- **Topic** *contains* 0..N **Diagrams**
