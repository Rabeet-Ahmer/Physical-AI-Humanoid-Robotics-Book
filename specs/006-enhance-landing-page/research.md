# Research Report: Enhance Landing Page

**Feature**: Enhance Landing Page
**Date**: 2025-12-12
**Status**: Completed

## 1. Landing Page Design Patterns (Academic)

**Unknown**: Best practices for academic textbook landing pages in Docusaurus.
**Findings**:
- **Hero Section**: Must include Title, Author(s), Concise Tagline, and primary CTA ("Read the Book"). A high-quality conceptual image (e.g., robot/brain) is standard.
- **Value Proposition**: A "Why this book?" section is crucial. For Physical AI, this means highlighting the "Simulation-First" and "Spec-Driven" approaches.
- **Curriculum Overview**: Users need to see the Table of Contents or Module breakdown immediately to assess relevance.
- **Interactive Elements**: Emphasize code and simulation capabilities.

## 2. Visual Style: "Academic & Clean"

**Unknown**: How to implement the requested "Academic & Clean" style (Serif headings, minimal clutter).
**Findings**:
- **Typography**:
  - **Headings**: Use a serif font (e.g., *Merriweather*, *Lora*, or *Playfair Display*) to evoke a textbook feel.
  - **Body**: Keep a clean sans-serif (system default or *Inter*) for readability.
- **Implementation**:
  - Import font via `@import` in `src/css/custom.css`.
  - Override Docusaurus CSS variables: `--ifm-heading-font-family`.
- **Color Palette**:
  - Background: White (`#ffffff`) or very light gray (`#f8f9fa`).
  - Text: Dark gray (`#212529`) rather than pure black for softness.
  - Accents: Muted blue or academic red, avoiding neon/hyper-vibrant tech colors unless used sparingly for diagrams.

## 3. Implementation Strategy

- **Hero Component**: Create a dedicated `HomepageHeader` component to separate hero logic from the main page.
- **Features Component**: Refactor `HomepageFeatures` to support the specific 3-column grid (ROS 2, Digital Twin, VLA).
- **CSS Overrides**:
  - Add Google Fonts import to `src/css/custom.css`.
  - Set `--ifm-heading-font-family: 'Merriweather', serif;`.
  - Increase whitespace/padding for a "breathing" layout.

## 4. Decisions & Rationale

- **Decision**: Use **Merriweather** for headings.
  - **Rationale**: It is a highly readable serif font designed for screens, fitting the "Academic" requirement perfectly.
- **Decision**: Split the landing page into distinct components (`HomepageHeader`, `HomepageCurriculum`, `HomepageFeatures`).
  - **Rationale**: Improves maintainability and allows for modular updates.

## 5. Technical Pre-requisites

- **Google Fonts**: Need to import 'Merriweather'.
- **Icons**: Use standard SVGs or Docusaurus-compatible icon libraries (e.g., FontAwesome or simple SVGs) for feature cards.
