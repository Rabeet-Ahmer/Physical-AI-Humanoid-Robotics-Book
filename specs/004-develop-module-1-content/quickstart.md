# Quickstart: Develop Module 1 Content

**Date**: 2025-12-11

This guide provides instructions to quickly set up, build, and verify the developed content for Module 1 within the Docusaurus site.

## Prerequisites

- Node.js (v18 or higher)
- npm (v8 or higher)
- Git

## Setup

1.  **Clone the repository (if you haven't already):**
    ```bash
    git clone [repository-url]
    cd [repository-name]
    ```
2.  **Checkout the feature branch:**
    ```bash
    git checkout 004-develop-module-1-content
    ```
3.  **Install Docusaurus dependencies:**
    ```bash
    npm install
    ```

## Build and Run Docusaurus Site

1.  **Start the Docusaurus development server:**
    ```bash
    npm run start
    ```
    This will open the site in your default browser, usually at `http://localhost:3000`.

## Verification

After the Docusaurus site is running, perform the following verification steps:

1.  **Content Completeness and Detail Check:**
    - Navigate to "Module 1: The Robotic Nervous System (ROS 2)" in the Docusaurus sidebar.
    - Go through each content section (Overview, ROS 2 Nodes/Topics/Services, Python Agents with rclpy, URDF for humanoids).
    - Visually confirm that each section contains detailed textual explanations, meeting the minimum word count (SC-001).

2.  **Pedagogical Structure Alignment Check:**
    - Within each content section, verify that it adheres to the pedagogical structure inspired by `module-02-digital-twin/01-course-introduction.md` (FR-010). Look for sections like Learning Objectives, Concept Overview, Theory, Code Examples, etc.

3.  **Diagrams Verification:**
    - Identify complex concepts within the text.
    - Confirm that relevant diagrams (Mermaid.js or high-quality SVG) are present and render correctly (FR-002, SC-002).

4.  **Code Snippets Verification:**
    - Locate all code snippets within Module 1.
    - Ensure they are formatted correctly (language highlighted).
    - If feasible, copy and run the Python 3.10+ code snippets in a local environment to verify executability and consistent results (FR-003, SC-003). Check for adherence to Constitution Principle III (commentary, reproducibility).

5.  **Citations Verification:**
    - Identify factual claims within the text.
    - Verify that academic citations in APA 7th Edition format are present for all such claims (FR-004, SC-004).

6.  **Up-to-Date Information Check:**
    - Briefly scan key technical details (e.g., ROS 2 versions, commands).
    - Confirm that the information appears current based on the latest knowledge in the field (FR-005, SC-006).

7.  **Docusaurus Formatting and Build Check:**
    - In your terminal, stop the Docusaurus development server (Ctrl+C).
    - Run the Docusaurus build command: `npm run build`
    - Check the console output for any warnings or errors related to Markdown, LaTeX, or admonition formatting (FR-006, SC-005).
    - Run the Docusaurus link validation tool: `npm run docusaurus check` (to be run after build, this will also detect errors in content itself)

## Next Steps

- Once verified, the developed content is ready for review and potential merging.
- If issues are found, report them and address them on the `004-develop-module-1-content` branch.
