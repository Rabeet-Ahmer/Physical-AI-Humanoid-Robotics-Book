# Research Report: Bento Layouts & Animations for Other Sections

**Feature**: Bento Layouts & Animations for Other Sections
**Date**: 2025-12-13
**Status**: Completed

## 1. Bento Grid CSS Layout Patterns

**Unknown**: Best practices for implementing bento grid layouts in Docusaurus/React.
**Findings**:
- **CSS Grid (Recommended)**: Ideal for 2D layouts with varying-sized cells. Features like `grid-template-areas`, `grid-row-start`/`end`, `grid-column-start`/`end`, and `span` are powerful for irregular grids.
- **Flexbox**: Useful for arranging content *within* individual bento grid cells, or for simpler 1D layouts.
- **React Integration**: Create reusable `BentoGridContainer` and `BentoGridItem` components. The `BentoGridItem` can take `rowSpan` and `colSpan` props to dynamically adjust its size using inline styles or CSS classes.
- **Docusaurus Styling**: Use global `src/css/custom.css` for core grid definitions and CSS Modules for component-specific styles. Leverage Docusaurus CSS variables for theming.
- **Responsiveness**: Mobile-first approach with `@media` queries. `grid-template-columns: repeat(auto-fit, minmax(WIDTH, 1fr))` for fluid columns.
**Decision**: Use CSS Grid for the primary bento layout, with Flexbox for internal content alignment within cells. Create `BentoGridContainer` and `BentoGridItem` React components.

## 2. Non-Generic Animation Techniques

**Unknown**: How to incorporate unique, non-generic animations (on-scroll reveals, parallax, hover effects) with a focus on performance for Docusaurus/React.
**Findings**:
- **On-Scroll Reveals**:
  - **Intersection Observer API**: The most performant and recommended method to detect when elements enter/exit the viewport. Triggers animations only when visible.
  - **Libraries**: `react-intersection-observer` is a React hook that simplifies using the Intersection Observer API.
- **Parallax**:
  - **JavaScript-based Libraries**: Offer more control and smoother effects than pure CSS parallax. Examples include `react-spring` (for physics-based animations) or custom `requestAnimationFrame` implementations tied to scroll position.
- **Hover Effects**:
  - **CSS Transitions/Transforms**: Performant for simple effects (e.g., `transform: scale()`, `transform: translateY()`, `box-shadow`).
  - **Framer Motion / React Spring**: React-specific libraries offer declarative APIs for complex, physics-based hover animations.
- **Performance Best Practices**:
  - **GPU Acceleration**: Animate `transform` and `opacity` only. Avoid animating properties that trigger layout/paint.
  - **`will-change`**: Use sparingly to hint browser about upcoming animations.
  - **Debouncing/Throttling**: Essential for scroll and resize events.
- **Decision**: Use `react-intersection-observer` for on-scroll reveal animations. Implement hover effects using CSS transitions/transforms. For parallax, start with CSS transforms and consider `react-spring` if more complex physics are needed. Encapsulate animation logic in a new `AnimatedContent` component.

## 3. Custom Font Integration (Consolidated)

**Unknown**: Ensure all previously requested custom fonts are correctly integrated and applied.
**Findings**:
- All fonts ('Akronim', 'Foldit', 'Story Script', 'Teko') are now correctly imported in `src/css/custom.css`.
- Application via CSS selectors (`h1-h6`, `.navbar__title`, `.homepage-main h1-h6`, `.docs-wrapper h1-h6`) ensures specific targeting.
- **Decision**: No further research needed; the current setup is sufficient and follows best practices.

## 4. Decisions & Rationale

- **Decision**: Create a `BentoGridContainer` and `BentoGridItem` components for reusable bento layout.
  - **Rationale**: Promotes modularity and simplifies applying the grid structure across different sections.
- **Decision**: Implement an `AnimatedContent` wrapper component for on-scroll reveal animations.
  - **Rationale**: Centralizes animation logic and leverages `react-intersection-observer` for performance.
- **Decision**: Utilize CSS for most animations (transforms, opacity, transitions) and `react-tsparticles` for background particles (from Hero design).
  - **Rationale**: Prioritizes performance by favoring GPU-accelerated CSS properties.

## 5. Technical Pre-requisites

- **`react-intersection-observer`**: New dependency to install.
