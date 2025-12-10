# Implementation Plan: Physical AI Textbook

**Branch**: `1-physical-ai-textbook` | **Date**: 2025-12-10 | **Spec**: [link](spec.md)
**Input**: Feature specification from `/specs/1-physical-ai-textbook/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

[Extract from feature spec: primary requirement + technical approach from research]

## Technical Context

**Language/Version**: Node.js 18+ (for Docusaurus), Python 3.10+ (for content code snippets).  
**Primary Dependencies**: Docusaurus v3+, React 18+, `@docusaurus/theme-mermaid`, `remark-math`, `rehype-katex`.  
**Storage**: Content stored as Markdown/MDX files in the repository.  
**Testing**: `npm run build` (Static Site Generation check), manual content verification.  
**Target Platform**: Static Web (hosted on Vercel/Netlify/GitHub Pages).  
**Project Type**: Web application (Static Site Generator).  
**Performance Goals**: Fast initial load (Static), functional offline navigation (Service Workers optional).  
**Constraints**: Must run on modern browsers. Content must be rigorous (Constitution).  
**Scale/Scope**: 4 Modules, ~15-20 core pages + landing + assets.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [x] **I. Pedagogical Clarity**: Modular structure defined in spec.
- [x] **II. Academic Rigor**: Spec implies rigorous content; implementation will enable citations via MDX if needed.
- [x] **III. Practical Application**: Python code blocks supported by Docusaurus.
- [x] **IV. Docusaurus Compatibility**: Feature is literally a Docusaurus site.
- [x] **VIII. Technical Verification**: Context7 used to verify Docusaurus plugins (Mermaid, Math).

## Project Structure

### Documentation (this feature)

```text
specs/1-physical-ai-textbook/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)
<!--
  ACTION REQUIRED: Replace the placeholder tree below with the concrete layout
  for this feature. Delete unused options and expand the chosen structure with
  real paths (e.g., apps/admin, packages/something). The delivered plan must
  not include Option labels.
-->

```text
# [REMOVE IF UNUSED] Option 1: Single project (DEFAULT)
src/
├── models/
├── services/
├── cli/
└── lib/

tests/
├── contract/
├── integration/
└── unit/

# [REMOVE IF UNUSED] Option 2: Web application (when "frontend" + "backend" detected)
backend/
├── src/
│   ├── models/
│   ├── services/
│   └── api/
└── tests/

frontend/
├── src/
│   ├── components/
│   ├── pages/
│   └── services/
└── tests/

# [REMOVE IF UNUSED] Option 3: Mobile + API (when "iOS/Android" detected)
api/
└── [same as backend above]

ios/ or android/
└── [platform-specific structure: feature modules, UI flows, platform tests]
```

**Structure Decision**: [Document the selected structure and reference the real
directories captured above]

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
