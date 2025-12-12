---
id: 008
title: Apply Specific Fonts to Landing Page Headings and Navbar Title
stage: green
date: 2025-12-12
surface: agent
model: gemini-2.0-flash-exp
feature: 007-update-logo
branch: 007-update-logo
user: uset
command: (user input for font styles)
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

Use this font style only for headings on landing page:
`@import url('https://fonts.googleapis.com/css2?family=Story+Script&display=swap');`

## Response snapshot

Updated `src/pages/index.tsx` to add `homepage-main` class to the main element.
Modified `src/css/custom.css` to:
- Import 'Akronim', 'Foldit', and 'Story Script' fonts.
- Apply 'Akronim' to global headings.
- Apply 'Foldit' to the `.navbar__title`.
- Apply 'Story Script' to headings within `.homepage-main` (landing page).
Successfully built the site after changes.

## Outcome

- ✅ Impact: Landing page headings, global headings, and navbar title now use specified fonts.
- 🧪 Tests: `npm run build` passed. Visual verification pending user confirmation.
- 📁 Files: `src/pages/index.tsx`, `src/css/custom.css` modified.
- 🔁 Next prompts: User to confirm visual changes, then merge.
- 🧠 Reflection: Successfully applied granular font styling based on user's specific request for targeting.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
