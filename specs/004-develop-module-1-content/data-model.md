# Data Model: Develop Module 1 Content

**Date**: 2025-12-11

## Entities

### 1. Module 1 Content (Root Entity)
- **Description**: The comprehensive educational material for "The Robotic Nervous System (ROS 2)" module. This includes detailed textual explanations, visual aids, code examples, and citations.
- **Attributes**:
    - `markdown_file`: (Path) - The `.md` file (e.g., `01-overview.md`) where the content resides.
    - `title`: (String) - The title of the content section, corresponding to Docusaurus front matter.
    - `slug`: (String) - The URL-friendly identifier for the content section.
    - `sidebar_label`: (String) - The label displayed in the Docusaurus sidebar.
    - `word_count`: (Integer) - The number of words in the section (SC-001).

### 2. Diagram
- **Description**: Visual representations used to explain complex concepts within Module 1 content.
- **Attributes**:
    - `type`: (Enum: Mermaid.js, SVG, Image) - The format of the diagram.
    - `source_code`: (String) - For Mermaid diagrams, the Mermaid DSL code.
    - `file_path`: (Path) - For SVG/Image, the path to the graphic file.
    - `caption`: (String) - A descriptive title for the diagram.
    - `associated_concept`: (String) - The complex concept the diagram illustrates (SC-002).

### 3. Code Snippet
- **Description**: Executable Python 3.10+ code examples demonstrating practical applications.
- **Attributes**:
    - `language`: (String, always "python") - The programming language.
    - `version`: (String, always "3.10+") - The Python version.
    - `code`: (String) - The actual code snippet.
    - `title`: (String) - Optional title for the code block.
    - `comments`: (String) - Explanations for the code's logic and choices.
    - `dependencies`: (List of Strings) - Libraries required to run the code (e.g., `numpy`, `rclpy`).
    - `reproducibility_steps`: (String) - Instructions to ensure consistent execution, including seed setting (Constitution Principle III).

### 4. Citation
- **Description**: Academic references for factual claims within the content.
- **Attributes**:
    - `format`: (String, always "APA 7th Edition") - The citation style.
    - `text`: (String) - The full citation entry (e.g., in a "References" section).
    - `inline_reference`: (String) - The inline reference in the content (e.g., `(Author, Year)`).
    - `source_type`: (Enum: Journal, Conference, Textbook, Whitepaper, Pre-print) - Classification of the source (Constitution Principle II).
    - `url`: (URL) - Link to the source, if available.

### 5. Web Search Result
- **Description**: Information retrieved from external web sources to ensure content is up-to-date and accurate. This entity is primarily for internal tracking during the content development process.
- **Attributes**:
    - `search_query`: (String) - The query used for the search.
    - `url`: (URL) - The URL of the retrieved source.
    - `date_accessed`: (Date) - When the information was retrieved.
    - `relevance_score`: (Integer) - Internal metric for how relevant the source is (1-5).
    - `summary`: (String) - Brief summary of the key findings from the source.

## Relationships

### 1. Module 1 Content to Diagram
- **Type**: Contains
- **Description**: A `Module 1 Content` section can contain one or more `Diagram` entities.

### 2. Module 1 Content to Code Snippet
- **Type**: Contains
- **Description**: A `Module 1 Content` section can contain one or more `Code Snippet` entities.

### 3. Module 1 Content to Citation
- **Type**: Contains
- **Description**: A `Module 1 Content` section can contain one or more `Citation` entities, both inline and in a references section.

### 4. Module 1 Content to Web Search Result
- **Type**: Derived From
- **Description**: `Module 1 Content` is derived from information obtained via `Web Search Result` entities. This is a process-level relationship, not a direct content inclusion.
