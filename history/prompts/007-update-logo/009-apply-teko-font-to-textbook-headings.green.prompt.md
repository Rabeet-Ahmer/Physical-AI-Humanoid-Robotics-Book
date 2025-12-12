---
id: 009
title: Apply Teko Font to Textbook Headings
stage: green
date: 2025-12-12
surface: agent
model: gemini-2.0-flash-exp
feature: 007-update-logo
branch: 007-update-logo
user: uset
command: (user input for font style)
labels: ["configuration", "styling", "font"]
links:
  spec: specs/007-update-logo/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - src/pages/index.tsx
  - src/css/custom.css
tests:
  - npm run build
---

## Prompt

Use this font for the main content's heading in the textbook:
`@import url('https://fonts.googleapis.com/css2?family=Teko:wght@300..700&display=swap');`

## Response snapshot

Updated `src/pages/index.tsx` to add `homepage-main` class.
Modified `src/css/custom.css` to:
- Import all requested fonts (Akronim, Foldit, Story Script, Teko).
- Apply 'Akronim' to global headings.
- Apply 'Foldit' to the `.navbar__title`.
- Apply 'Story Script' to headings within `.homepage-main` (landing page).
- Apply 'Teko' to headings within textbook content (`.docs-wrapper h1-h6`).
Successfully built the site after changes.

## Outcome

- ✅ Impact: Text headings across the site now use specified fonts as per granular rules.
- 🧪 Tests: `npm run build` passed. Visual verification pending user confirmation.
- 📁 Files: `src/pages/index.tsx`, `src/css/custom.css` modified.
- 🔁 Next prompts: User to confirm visual changes, then merge.
- 🧠 Reflection: Successfully implemented complex font styling rules targeting different sections of the Docusaurus site.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
