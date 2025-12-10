---
id: 001
title: initial-constitution-setup
stage: constitution
date: 2025-12-10
surface: agent
model: gemini-2.0-flash-exp
feature: none
branch: main
user: user
command: /sp.constitution
labels: ["governance", "setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - null
---

## Prompt

/sp.constitution We are making a textbook on "Physical AI & Humanoid Robotics" which will act as a complete well defined course. The technical documentation for the tech stacks (i.e Docusaurus) which will be used in the creation of the textbook should be fetched using Context7 MCP.

## Core Principles:
- Pedalogical Clarity & Structure. (All modules content MUST be clear, concise, and logically structured to facilitate student learning. Prerequisites MUST be explicitly stated, concepts introduced progressively, and assumptions for pedagogical purposes clearly articulated. Terminology and notation MUST remain consistent throughout the textbook.)
- Academic Rigor And Sourcing. (All technical claims MUST be supported by evidence from authoritative, peer-reviewed academic sources published within the past 10 years. Citations MUST adhere to APA format.)
- Practical Application And Code examples. (The module MUST include practical examples and exercises that demonstrate key concepts and reinforce learning. Students MUST be able to write Python code after completing the modules.)
- Docusaurus Compatibility and Formatting. (The module content MUST be written in Markdown format, strictly compatible with the Docusaurus platform, ensuring proper rendering of code blocks, citations, and cross-references.)
- Content Scope and Focus. (All modules must strictly adhere to its defined scope, content must be between 3000-5000 words, out of scope content must be avoided)
- Development and Documentation standards. (All content creation must align with the principles of this constitution)
- Learning and assessment objectives. (The textbook aims to enable students to explain concepts, learning will be enforced through examples and exercises)

## Response snapshot

Updated constitution with Physical AI & Humanoid Robotics principles and set version 1.0.0.

## Outcome

- ✅ Impact: Established project governance and core principles.
- 🧪 Tests: N/A
- 📁 Files: Updated constitution.md
- 🔁 Next prompts: Plan or Spec creation
- 🧠 Reflection: Initial setup successful.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
