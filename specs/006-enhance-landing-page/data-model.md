# Data Model: Landing Page

**Feature**: Enhance Landing Page
**Status**: N/A (Static Content)

## Summary

This feature primarily involves UI components and static content. No persistent data model is required.

## Components & Props

### 1. FeatureItem
- **Props**:
  - `title`: string
  - `Svg`: React.ComponentType<ComponentProps<'svg'>>
  - `description`: JSX.Element

### 2. ModuleItem (Curriculum)
- **Props**:
  - `title`: string
  - `link`: string (url)
  - `description`: string
