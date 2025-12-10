# Quickstart: Refactor Module 1 ROS 2

**Date**: 2025-12-11

This guide provides instructions to quickly set up, build, and verify the refactored Module 1 documentation within the Docusaurus site.

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
    git checkout 001-refactor-module-1-ros2
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

1.  **Visual Structure Consistency Check:**
    - Navigate to `http://localhost:3000`.
    - In the sidebar, expand "Module 1: The Robotic Nervous System (ROS 2)".
    - Visually compare the structure (top-level categories, nesting, order of items) of Module 1 with "Module 2: Digital Twin". Ensure Module 1 strictly emulates Module 2's structure as defined in the specification.

2.  **Internal Link Functionality Check:**
    - Browse through various pages within the refactored Module 1.
    - Click on any internal links (links pointing to other pages within Module 1 or the Docusaurus site).
    - Ensure all links resolve correctly and navigate to the intended pages.

3.  **Automated Link Validation:**
    - In your terminal, stop the Docusaurus development server (Ctrl+C).
    - Run the Docusaurus link validation tool:
    ```bash
    npm run docusaurus check
    ```
    - Verify that there are no broken internal or external links reported for Module 1 or the entire site.

4.  **Docusaurus Metadata Check:**
    - While the `docusaurus check` command helps, also manually inspect `_category_.json` files and Markdown front matter within `docs/module-01-nervous-system/` for correct YAML/JSON syntax and valid Docusaurus metadata fields, especially if any metadata errors were previously observed or addressed.

## Next Steps

- Once verified, the feature is ready for review and potential merging.
- If issues are found, report them and address them on the `001-refactor-module-1-ros2` branch.