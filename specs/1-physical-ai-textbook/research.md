# Research & Decisions: Physical AI Textbook

**Feature**: Physical AI Textbook (Docusaurus)
**Date**: 2025-12-10

## 1. Framework Selection
*   **Decision**: Docusaurus v3 (Classic Theme)
*   **Rationale**: Explicit user requirement (FR-001). Excellent support for technical documentation, versioning, and static performance.
*   **Alternatives**: MkDocs, Jekyll (Rejected due to user constraint).

## 2. Mathematical Typesetting
*   **Decision**: `remark-math` + `rehype-katex`
*   **Rationale**: Standard MDX ecosystem tools supported by Docusaurus for rendering LaTeX equations ($E=mc^2$) required by Constitution Principle IV (Math Rendering).
*   **Implementation**:
    *   Install packages.
    *   Configure `docusaurus.config.js` to use these in `presets`.
    *   Include KaTeX CSS in `src/css/custom.css`.

## 3. Diagramming
*   **Decision**: `@docusaurus/theme-mermaid`
*   **Rationale**: First-party support in Docusaurus v3. Meets Constitution Principle I (Visual Aids) for rendering kinematics/control loops directly from text.

## 4. Project Structure
*   **Decision**: Standard Docusaurus `classic` template.
*   **Rationale**: Provides `docs/`, `blog/`, `src/pages/` out of the box.
*   **Mapping**:
    *   `docs/` -> Modules (Hierarchical content).
    *   `src/pages/index.js` -> Landing page (FR-006).
    *   `docs/hardware-requirements.md` -> Hardware guide.

## 5. Unknowns Resolved
*   **Compatibility**: Docusaurus v3 requires Node.js 18+.
*   **Python**: Confirmed that Python requirement is *only* for code blocks within the content, not for the build process itself.
