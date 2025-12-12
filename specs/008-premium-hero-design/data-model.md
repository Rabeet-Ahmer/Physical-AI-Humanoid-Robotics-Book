# Data Model: Premium Hero Section Design

**Feature**: Premium Hero Section Design
**Status**: N/A (UI Component)

## Summary

This feature primarily involves the design and implementation of a React UI component (`HomepageHeader`) and its supporting sub-components. It does not introduce any new data entities or require persistence beyond static assets.

## Components & Props

### 1. HomepageHeader (Reworked)
- **Props**:
  - `title`: string (from siteConfig)
  - `tagline`: string (from siteConfig)

### 2. BackgroundAnimation (New Component)
- **Props**:
  - `options`: object (configuration for `react-tsparticles`)

### 3. FeatureBadge (New Reusable Component)
- **Props**:
  - `text`: string
  - `icon`: string (emoji or SVG path)

### 4. CTAButton (Reused/Updated Styling)
- **Props**:
  - `text`: string
  - `link`: string
  - `styleType`: 'primary' | 'secondary'
  - `icon`: string (emoji or SVG path, optional)

## Assets

- **Hero Image**: `static/img/hero-book-cover.jpg` (URL: `https://stockcake.com/i/futuristic-robot-portrait_1029060_1100434`)
- **Icons**: Emojis (✨, 🤝, 🎯, 🎓) for badges and secondary CTA.
