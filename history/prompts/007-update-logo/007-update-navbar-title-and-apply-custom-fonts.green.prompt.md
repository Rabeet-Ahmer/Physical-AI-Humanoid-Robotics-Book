---
id: 007
title: Update Navbar Title and Apply Custom Fonts
stage: green
date: 2025-12-12
surface: agent
model: gemini-2.0-flash-exp
feature: 007-update-logo
branch: 007-update-logo
user: uset
command: (user input for font style and title)
labels: ["configuration", "styling", "font"]
links:
  spec: specs/007-update-logo/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - docusaurus.config.ts
  - src/css/custom.css
tests:
  - npm run build
---

## Prompt

Use this font style to write "Physical AI Book" ahead of the logo:
`@import url('https://fonts.googleapis.com/css2?family=Akronim&family=Foldit:wght@100..900&display=swap');`

## Response snapshot

Updated `docusaurus.config.ts` to set navbar title to "Physical AI Book".
Modified `src/css/custom.css` to import 'Akronim' and 'Foldit' fonts.
Applied 'Akronim' to general headings and 'Foldit' to the `.navbar__title`.
Successfully built the site after changes.

## Outcome

- ✅ Impact: Navbar title updated with requested text and custom font applied. Global headings also use 'Akronim'.
- 🧪 Tests: `npm run build` passed. Visual verification (navbar title, logo, heading fonts) pending user confirmation.
- 📁 Files: `docusaurus.config.ts`, `src/css/custom.css` modified.
- 🔁 Next prompts: User to confirm visual changes, then merge.
- 🧠 Reflection: Clarified font application for multiple families in the request.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
