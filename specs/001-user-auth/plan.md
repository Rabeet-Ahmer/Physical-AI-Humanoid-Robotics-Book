# Implementation Plan: Add User Authentication

**Branch**: `001-user-auth` | **Date**: 2025-12-19 | **Spec**: [specs/001-user-auth/spec.md](../001-user-auth/spec.md)
**Input**: Feature specification from `/specs/001-user-auth/spec.md`

## Summary

Implement a comprehensive authentication system using **Better Auth** for the frontend (Docusaurus) and **Neon Serverless Postgres** as the persistence layer. To support Better Auth (which requires a Node.js runtime), we will create a lightweight **Auth Server** (Node.js). The **Python Backend** (FastAPI) will verify authentication sessions by querying the shared database to gate access to the Chatbot API.

## Technical Context

**Language/Version**: Python 3.13 (Backend), TypeScript 5.x (Frontend/Auth Server)
**Primary Dependencies**: 
- Backend: `fastapi`, `asyncpg` (for session verification)
- Auth Server: `better-auth`, `hono` (or express), `pg`
- Frontend: `better-auth`, `@better-auth/react`
**Storage**: Neon Serverless Postgres (Shared)
**Testing**: `pytest` (Backend), `vitest` (Auth Server if needed)
**Target Platform**: Web (Docusaurus + Auth Server + FastAPI)
**Project Type**: Microservices (Frontend + Auth Service + Backend API)
**Performance Goals**: Login < 1s, Session validation < 50ms
**Constraints**: 
- Must use Better Auth (Node.js).
- Must use Python for Chatbot Backend.
- Must keep Docusaurus structure.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [x] **Project Directory**: Auth Server in `auth-server/`. Backend in `backend/`.
- [x] **Python-First**: Core business logic (Chatbot) remains in Python. Auth is delegated to a specialized tool (Better Auth) as requested.
- [x] **Tech Stack**: Uses Neon, FastAPI.
- [x] **No Hallucinations**: Validated Better Auth requirements.
- [x] **Context7**: Used for docs.

## Project Structure

### Documentation (this feature)

```text
specs/001-user-auth/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
└── tasks.md             # Phase 2 output
```

### Source Code

```text
auth-server/           # NEW: Node.js service for Better Auth
├── src/
│   ├── index.ts       # Server entry point
│   └── auth.ts        # Better Auth configuration
├── package.json
└── tsconfig.json

backend/               # Python Backend
├── src/
│   ├── api/
│   │   ├── dependencies.py # Auth verification (reads DB)
│   │   └── routers/        # Chatbot router
│   └── models/
└── tests/

src/                   # Docusaurus Frontend
├── components/
│   └── Auth/          # Login/Signup UI
├── lib/
│   └── auth-client.ts # Better Auth Client
└── pages/
```

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Separate Auth Server (Node.js) | Better Auth requires Node.js runtime. Docusaurus is static; Backend is Python. | Re-implementing full Auth/OAuth/Session logic in Python (FastAPI) would violate the user's explicit request to use "Better Auth Framework". |
