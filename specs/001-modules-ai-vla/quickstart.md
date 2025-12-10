# Quickstart Guide: Contributing to AI Robot Brain & Vision-Language-Action Modules

**Branch**: `001-modules-ai-vla` | **Date**: 2025-12-10 | **Plan**: [specs/001-modules-ai-vla/plan.md](specs/001-modules-ai-vla/plan.md)

This guide provides a quick overview for contributors looking to add content to Module 3: The AI-Robot Brain (NVIDIA Isaac™) and Module 4: Vision-Language-Action (VLA) of the textbook.

---

## 1. Get Started

1.  **Clone the Repository**:
    ```bash
    git clone [repository_url]
    cd Hackathon1.0
    ```
2.  **Checkout the Feature Branch**:
    ```bash
    git checkout 001-modules-ai-vla
    ```
3.  **Install Dependencies**: Ensure you have Node.js and npm installed.
    ```bash
    npm install
    ```
4.  **Start Docusaurus Development Server (Optional, but Recommended)**:
    This allows you to preview your changes locally as you write.
    ```bash
    npm run start
    ```
    Your browser should open to `http://localhost:3000`.

## 2. Locate Module Directories

-   **Module 3 (AI Robot Brain)**: Content files go into `docs/module-03-ai-brain/`
-   **Module 4 (Vision-Language-Action)**: Content files go into `docs/module-04-vla/`

If these directories do not exist, create them. Ensure they contain an `_category_.json` file for sidebar integration.

## 3. Create New Topic Files

For each new topic within a module:

1.  **Create a Markdown File**: Inside the appropriate module directory, create a new `.md` or `.mdx` file following the naming convention `NN-topic-name.md` (e.g., `01-course-introduction.md`, `02-nvidia-isaac-sim.md`). The `NN` ensures correct ordering in the sidebar.
2.  **Add Frontmatter**: Start your file with Docusaurus frontmatter:
    ```yaml
    ---
    title: Your Topic Title
    description: A brief description of what this topic covers.
    slug: /module-[module-number]/your-topic-slug
    tags: [robotics, ai, perception]
    ---
    ```
3.  **Follow the Content Contract**: Strictly adhere to the section structure and guidelines defined in `specs/001-modules-ai-vla/contracts/content-contract.md`. Ensure all mandatory headers are present and filled.

## 4. Key Content Requirements

-   **Diagrams**: Use Mermaid syntax for all diagrams within the `## Diagrams (MANDATORY)` section.
-   **Code Examples**: Provide real, executable code in the `## Code Examples (MANDATORY)` section. Specify the language for syntax highlighting (e.g., `` ```python ``).
-   **Mathematics**: Use LaTeX for all mathematical expressions (e.g., `$inline math$` or `$$display math$$`).
-   **Citations**: Include APA 7th Edition citations for all academic claims.
-   **Technical Verification**: When referencing specific APIs, configurations, or library features, ensure to verify their details using the Context7 MCP tool during the writing process (as per the project Constitution).

## 5. Review and Submit

1.  **Review your content locally**: Use `npm run start` to check rendering, formatting, and sidebar integration.
2.  **Linting/Formatting**: Ensure your Markdown adheres to project standards.
3.  **Commit Your Changes**:
    ```bash
    git add .
    git commit -m "feat(moduleX): Add new topic - [Topic Name]"
    ```
4.  **Push and Create a Pull Request**:
    ```bash
    git push origin 001-modules-ai-vla
    ```
    Create a Pull Request against the `main` branch, linking back to this feature spec.

By following these guidelines, you'll ensure your contributions are high-quality and integrate seamlessly into the textbook.
