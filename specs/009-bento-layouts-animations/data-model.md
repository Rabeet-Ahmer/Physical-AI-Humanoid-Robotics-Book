# Data Model: Bento Layouts & Animations

**Feature**: Bento Layouts & Animations for Other Sections
**Status**: N/A (UI Component)

## Summary

This feature involves redesigning existing UI components and introducing new ones for layout and animation. It does not introduce any new data entities or require persistence beyond static assets.

## Components & Props

### 1. HomepageFeatures (Reworked)
- **Props**: None (will fetch data internally or from hardcoded list).
- **Structure**: Will render `BentoGridContainer` containing `BentoGridItem`s.

### 2. HomepageCurriculum (Reworked)
- **Props**: None (will fetch data internally or from hardcoded list).
- **Structure**: Will render `BentoGridContainer` containing `BentoGridItem`s.

### 3. BentoGridContainer (New Component)
- **Props**:
  - `children`: ReactNode (content of the grid).
  - `className`: string (additional CSS classes).

### 4. BentoGridItem (New Component)
- **Props**:
  - `children`: ReactNode (content of the grid item).
  - `className`: string (additional CSS classes).
  - `colSpan`: number (how many columns the item spans, for CSS Grid).
  - `rowSpan`: number (how many rows the item spans, for CSS Grid).
  - `variant`: string (e.g., 'tall', 'wide', 'default' to apply specific styles).

### 5. AnimatedContent (New Component)
- **Props**:
  - `children`: ReactNode (content to animate).
  - `animationType`: string (e.g., 'fade-in-up', 'parallax', 'hover-tilt').
  - `delay`: number (animation delay in ms).
  - `threshold`: number (Intersection Observer threshold).

## Assets

- **Icons**: Emojis (✨, 🤝, 🎯, 🎓) for badges, as decided in previous spec.
- **Background Images/Textures**: Potentially for specific bento cells, but not explicitly requested.

## Animations
- **On-scroll reveal**: Using `react-intersection-observer` (via `AnimatedContent`).
- **Hover effects**: CSS transitions on `transform`, `box-shadow`.
- **Parallax**: Possibly CSS background-position or JS-based `transform: translateY` on scroll.
