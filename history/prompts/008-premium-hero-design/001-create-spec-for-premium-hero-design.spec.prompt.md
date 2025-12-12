---
id: 001
title: Create Spec for Premium Hero Design
stage: spec
date: 2025-12-12
surface: agent
model: gemini-2.0-flash-exp
feature: 008-premium-hero-design
branch: 008-premium-hero-design
user: uset
command: /sp.specify
labels: ["spec", "hero-section", "design", "ui", "landing-page"]
links:
  spec: specs/008-premium-hero-design/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/008-premium-hero-design/spec.md
  - specs/008-premium-hero-design/checklists/requirements.md
tests: []
---

## Prompt

Create the **hero section** with the following layout, styling, and content. The goal is to replicate a polished, premium, modern design similar to high‑end tech education landing pages. ### **Overall Layout** * Two‑column hero layout. * **Left column:** A large, centered course/book cover image with a futuristic AI‑themed visual aesthetic. * **Right column:** Title, subtitle, description tags, and call‑to‑action buttons. * Section should feel spacious, balanced, and visually premium. **Atmosphere** * Subtle glowing particles or soft light bloom in the background. * Clean, high‑contrast white headings. ### **Left Column (Hero Image)** * A large rectangular book/course cover image. * For image search online * Image should have rounded corners and a subtle shadow. ### **Right Column (Text Content)** #### **Top Label Badge** * Rounded pill badge with text: * *"PHYSICAL AI BOOK"*. * Semi‑transparent gray background. * Small-caps typography. #### **Main Title** * Large bold heading: * **"AI Native Robotics"** * Clean, bold, modern sans serif typography. #### **Subtitle** * Medium-large text: * *"Learn Physical AI & Humanoid Robotics the modern way"* * Slightly muted gray tone. ### **Feature Badges Row** Create three horizontally aligned feature badges: * **Open Source** (with small sparkle emoji or icon) * **Co‑Learning with AI** (handshake emoji or icon) * **Spec‑Driven Development** (target/bulls‑eye icon) Styling: * Rounded pill buttons. * Dark gray background. * Soft shadow. * Light inner border. ### **Call‑to‑Action Buttons** Two large buttons below the feature row: 1. **Primary CTA (white button):** * Text: *"Start Reading →"* * White background, dark text, large padding, rounded full. 2. **Secondary CTA (dark glass button):** * Text: *"Explore Project"* * Dark translucent background with soft border. * Small graduation‑cap emoji/icon. Buttons should appear side‑by‑side with space between. ### **Responsive Design Requirements** * On mobile, switch to a stacked layout (image on top, text below). * All elements centered on mobile. ### **Overall Mood & Style** * Premium educational platform vibe. * Futuristic AI aesthetics. * Clean typography, balanced spacing, soft shadows. * Modern glass‑morphism pills and buttons. --- Use this exact structure when generating the hero section layout for the Docusaurus site.

## Response snapshot

Created feature branch 008-premium-hero-design.
Generated specification for the Premium Hero Section Design.
Identified two clarification needs: Hero image source and specific icon sources.
Checklist indicates failing status due to these clarifications.

## Outcome

- ✅ Impact: Detailed requirements for a new hero section captured.
- 🧪 Tests: Checklist shows failing status due to pending clarifications.
- 📁 Files: Spec and Checklist created.
- 🔁 Next prompts: /sp.clarify
- 🧠 Reflection: The user provided a highly detailed request, which allowed for a very granular spec. Clarifications are for external assets.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
