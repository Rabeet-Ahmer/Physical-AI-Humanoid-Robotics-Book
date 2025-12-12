# Quickstart: Premium Hero Section Design

**Feature**: Premium Hero Section Design
**Date**: 2025-12-12

This guide provides instructions to quickly set up and verify the new Hero section on the Docusaurus landing page.

## Prerequisites

- Node.js (v18+)
- npm
- Git

## Setup

1.  **Clone the repository (if not already):** `git clone [repository-url]`
2.  **Checkout the feature branch:** `git checkout 008-premium-hero-design`
3.  **Install Docusaurus dependencies:** `npm install`
4.  **Install `react-tsparticles`**: `npm install react-tsparticles tsparticles`

## Build and Run Docusaurus Site

1.  **Start the Docusaurus development server:**
    ```bash
    npm run start
    ```
    This will open the site in your default browser, usually at `http://localhost:3000`.

## Verification Checklist

### Desktop View (Responsive Check: Full Width)
- [ ] **Two-Column Layout**: Verify the Hero section displays as two columns (Left: Image, Right: Text/CTAs).
- [ ] **Hero Image**: Confirm the image `static/img/hero-book-cover.jpg` is displayed, is large, rectangular, has rounded corners, and a subtle shadow. The image should be `https://stockcake.com/i/futuristic-robot-portrait_1029060_1100434`.
- [ ] **Badge**: Confirm "PHYSICAL AI BOOK" badge is present, rounded, semi-transparent gray, and in small-caps.
- [ ] **Main Title**: Verify "AI Native Robotics" is displayed, large, bold, clean, modern sans-serif.
- [ ] **Subtitle**: Verify "Learn Physical AI & Humanoid Robotics the modern way" is present, medium-large, and slightly muted gray.
- [ ] **Feature Badges**: Confirm three horizontally aligned rounded pill buttons with specified text and emojis (✨, 🤝, 🎯). They should have dark gray background, soft shadow, light inner border.
- [ ] **CTA Buttons**: Verify two side-by-side buttons:
    - Primary: "Start Reading →" (white background, dark text, large padding, rounded full).
    - Secondary: "Explore Project" (dark translucent background, soft border, graduation-cap emoji 🎓).
- [ ] **Atmosphere**: Look for subtle glowing particles or light bloom in the background. Headings should be high-contrast white.
- [ ] **Overall Mood**: Assess if it conveys a "premium, futuristic AI education" vibe.

### Mobile View (Responsive Check: Narrow Width)
- [ ] **Stacked Layout**: Resize browser to a mobile width. Verify the Hero section switches to a stacked layout (Image on top, Text below).
- [ ] **Centered Elements**: Confirm all elements (image, text, buttons) are centered on mobile.
- [ ] **Readability**: Ensure all text is readable and buttons are easily tappable.

### Technical Checks
- [ ] **Performance**: After `npm run build`, run Google Lighthouse and check Performance score (>90%).
- [ ] **Accessibility**: Run Google Lighthouse and check Accessibility score (>90%).
- [ ] **Build Check**: Run `npm run build` to ensure no build errors.

## Next Steps

- Once verified, the new Hero section is ready for review and potential merging.
- If issues are found, report them and address them on the `008-premium-hero-design` branch.
