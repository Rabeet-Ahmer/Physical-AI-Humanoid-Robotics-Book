# Research Report: Update Textbook Logo

**Feature**: Update Textbook Logo
**Date**: 2025-12-12
**Status**: Completed

## 1. Logo Concept

**Unknown**: Best SVG design for "Physical AI".
**Findings**:
- **Concept**: A simple book icon with "AI" text or a circuit/brain motif overlaying a book.
- **Style**: Minimalist, flat design to scale well on navbar and footer.
- **Colors**: Should match the primary theme color (`#2e8555` or similar green) or be neutral (white/gray) for adaptability.

## 2. Technical Implementation

**Unknown**: How to handle SVGs in Docusaurus.
**Findings**:
- Docusaurus uses `static/img/logo.svg`.
- Ideally, the SVG should define `fill="currentColor"` or specific classes to adapt to light/dark mode, or be provided as a colored SVG that works on both backgrounds (e.g., a bright accent color).
- Replacing the file is sufficient; no config change needed if the name remains `logo.svg`.

## 3. Decisions & Rationale

- **Decision**: Generate a simple SVG code representing an open book with "AI" text.
  - **Rationale**: Direct code generation ensures we have a valid, clean SVG without needing external image generation tools or downloads.
- **Decision**: Keep filename `logo.svg`.
  - **Rationale**: Avoids touching `docusaurus.config.ts`, reducing risk of broken links.

## 4. Implementation Strategy

- Create the SVG file content directly.
- Overwrite `static/img/logo.svg`.
