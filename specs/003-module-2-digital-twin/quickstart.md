# Quickstart: Module 2 Content Generation

## Prerequisites

- Node.js 18+ (for Docusaurus)
- Python 3.10+ (for verifying code examples)
- VS Code with Markdown All in One & Mermaid extension

## Generation Steps

1.  **Create Directory**: Ensure `docs/module-02-digital-twin/` exists.
2.  **Generate Files**: Create the markdown files as per the `data-model.md` and `plan.md`.
    - `01-course-introduction.md`
    - `02-physics-simulation-gazebo.md`
    - `03-high-fidelity-rendering-unity.md`
    - `04-simulating-sensors.md`
3.  **Fill Content**: Use the templates and requirements from `spec.md` to populate each file.
    - Ensure Frontmatter matches `contracts/frontmatter-schema.yaml`.
    - Validate Mermaid diagrams render correctly.
    - Verify code snippets are syntactically correct.

## Verification

1.  **Start Docusaurus**:
    ```bash
    npm start
    ```
2.  **Navigate**: Open `http://localhost:3000/docs/module-02-digital-twin/course-introduction` (URL may vary based on `slug` configuration).
3.  **Review**: Check formatting, sidebar ordering, and diagram rendering.

## Code Quality

- Run `npm run typecheck` (if configured) or standard markdown linters.
- Ensure all LaTeX math (`$$ ... $$`) renders without errors.
