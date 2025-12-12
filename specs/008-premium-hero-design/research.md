# Research Report: Premium Hero Section Design

**Feature**: Premium Hero Section Design
**Date**: 2025-12-12
**Status**: Completed

## 1. Animated Background (Glowing Particles / Light Bloom)

**Unknown**: How to implement "subtle glowing particles or soft light bloom in the background" for a Docusaurus React component efficiently.
**Findings**:
- **Technology**: `react-tsparticles` (React wrapper for `tsparticles`) is a suitable and performant library for particle animations. It offers extensive customization.
- **Implementation**:
  - Integrate `react-tsparticles` into a new `BackgroundAnimation` React component.
  - Configure particle options (density, color, size, movement, interactions).
  - Use CSS (`box-shadow`, `filter: drop-shadow()`) on particles or container for enhanced glowing/bloom effects. `tsparticles` itself has `shadow` and `twinkle` options.
- **Performance**: Ensure particles are optimized to maintain a high Lighthouse Performance score.
- **Decision**: Use `react-tsparticles` for particle animation and CSS for light bloom/glowing effects.

## 2. Glass-morphism CSS Effects

**Unknown**: How to implement glass-morphism for badges and buttons in Docusaurus.
**Findings**:
- **Core CSS Properties**:
  - `backdrop-filter: blur(Xpx)`: Key for the frosted glass effect. Requires `-webkit-backdrop-filter` for Safari.
  - `background: rgba(R, G, B, alpha)`: Semi-transparent background.
  - `border: 1px solid rgba(...)`: Subtle, semi-transparent border.
  - `box-shadow`: For depth.
- **Implementation**:
  - Create a reusable CSS class (e.g., `.glass-effect`) in `src/css/custom.css`.
  - Apply this class to relevant elements (badges, buttons).
  - Ensure dark mode compatibility by adjusting `rgba` values and border colors within `[data-theme='dark']` scope.
- **Decision**: Implement a `.glass-effect` CSS class in `custom.css` and apply it to the feature badges and secondary CTA button.

## 3. Custom Font Integration

**Unknown**: Consolidate best practices for integrating multiple custom fonts into Docusaurus.
**Findings**:
- **Method**: Use `@import url(...)` in `src/css/custom.css` for Google Fonts.
- **Targeting**:
  - Global headings (`h1-h6`): Default font is set via `--ifm-heading-font-family`.
  - Specific components/sections (e.g., `.navbar__title`, `.homepage-main h1-h6`): Apply font directly via CSS class selector.
- **Fonts to integrate**: 'Akronim' (global headings), 'Foldit' (navbar title), 'Story Script' (landing page headings), 'Teko' (textbook content headings).
- **Decision**: All required fonts are already imported in `custom.css` and applied as per previous requests. This research confirms the approach.

## 4. Decisions & Rationale

- **Decision**: Create a new `BackgroundAnimation` component utilizing `react-tsparticles` for the hero background.
  - **Rationale**: Provides a configurable, performant solution for particle effects with React integration.
- **Decision**: Centralize glass-morphism styles in `custom.css` for reusability.
  - **Rationale**: Ensures consistency and maintainability of the visual effect.

## 5. Technical Pre-requisites

- **`react-tsparticles`**: Installation required.
- **Image Assets**: The hero book cover image needs to be downloaded to `static/img/hero-book-cover.jpg`.
- **Icon Assets**: Emojis will be used directly in JSX/HTML.
