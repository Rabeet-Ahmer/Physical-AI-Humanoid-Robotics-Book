# Quickstart: Create Textbook Introduction

**Feature**: Create Textbook Introduction
**Date**: 2025-12-12

This guide provides instructions to verify the "Course Introduction" content update.

## Prerequisites

- Node.js (v18+)
- npm

## Setup

1.  **Clone repository**: `git clone [repo]`
2.  **Checkout branch**: `git checkout 005-create-textbook-intro`
3.  **Install dependencies**: `npm install`

## Verification Steps

1.  **Run Docusaurus**:
    ```bash
    npm run start
    ```
2.  **Open Browser**: Navigate to `http://localhost:3000/docs/intro`.
3.  **Visual Check**:
    - [ ] **Definition**: Verify "Physical AI" is defined and contrasted with "Generative AI".
    - [ ] **Diagram**: Verify the "Brain-Body-Nervous System" Mermaid diagram renders correctly.
    - [ ] **Modules**: Verify all 4 modules are listed in the correct order.
    - [ ] **Prerequisites**: Verify specific versions are listed (Ubuntu 22.04, ROS 2 Humble).
    - [ ] **Cleanliness**: Verify no placeholder text (e.g., "E=mc^2") remains.

## Troubleshooting

- **Diagram not rendering**: Ensure the `mermaid` code block is properly fenced with ```mermaid.
- **Broken layout**: Check browser console for Docusaurus hydration errors.
