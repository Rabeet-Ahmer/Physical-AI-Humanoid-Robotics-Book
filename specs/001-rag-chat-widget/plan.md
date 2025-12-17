# Implementation Plan: Docusaurus RAG Chat Widget

**Branch**: `001-rag-chat-widget` | **Date**: 2025-12-17 | **Spec**: [specs/001-rag-chat-widget/spec.md](../001-rag-chat-widget/spec.md)
**Input**: Feature specification from `specs/001-rag-chat-widget/spec.md`

## Summary

The goal is to implement a frontend chat widget for the RAG chatbot within the Docusaurus site. This involves creating a persistent icon, a toggleable chat UI panel with user input and response display, and state persistence across page navigations. The UI must integrate seamlessly with the existing Docusaurus theme and adhere to accessibility standards.

## Technical Context

**Language/Version**: TypeScript/JavaScript (React 18+)
**Primary Dependencies**: React, Docusaurus, Axios/Fetch (for backend API calls)
**Storage**: Browser Local Storage (for chat state and history persistence)
**Testing**: Jest, React Testing Library
**Target Platform**: Web browser (Docusaurus site)
**Project Type**: Web application (Frontend component)
**Performance Goals**: No measurable negative impact (>5%) on Core Web Vitals (FID, LCP, CLS)
**Constraints**: 
- Seamless visual integration with Docusaurus theme.
- WCAG 2.1 AA accessibility compliance.
- Fixed, small width/height chat UI.
- Maintain state across page navigations.
- Utilize existing backend RAG API (`POST /agent`).

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [x] **Project Directory**: Frontend related, so within `src/` for Docusaurus.
- [x] **Python-First**: N/A for frontend development.
- [x] **Tech Stack Adherence**: Uses React/TypeScript (Docusaurus standard), `fetch` API for network. Browser Local Storage for persistence. All aligned with modern web standards and Docusaurus context.
- [x] **No Hallucinations**: Research completed and decisions made based on findings.
- [x] **Context7 Documentation**: Used for Docusaurus, React, and accessibility best practices research.
- [x] **RAG-Centric Architecture**: This feature is the client-side consumer of the RAG architecture.
- [x] **API Standards**: Consumes the existing `POST /agent` API, adhering to its contract.
- [x] **Code Quality Standards**: Type hints, documentation, clean code will be adhered to in frontend implementation.

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-chat-widget/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output (API contracts from backend)
└── tasks.md             # Phase 2 output (to be created)
```

### Source Code (repository root)

```text
src/
├── components/          # NEW: Chat widget components
│   ├── ChatWidget/
│   │   ├── index.tsx
│   │   ├── ChatIcon.tsx
│   │   ├── ChatPanel.tsx
│   │   └── ChatMessage.tsx
│   └── hooks/           # NEW: Custom React hooks for state management, API calls
├── theme/               # MODIFIED: Custom Docusaurus theme layout (e.g., Layout.js)
│   └── Layout.js        # MODIFIED: To render the ChatWidget
└── css/                 # NEW: Styling for the ChatWidget
```

**Structure Decision**: Option 2 (Web application - Frontend focus). New components will be placed in `src/components/ChatWidget` and integrated into the Docusaurus theme via `src/theme/Layout.js`.
