# Tasks: Add User Authentication

**Branch**: `001-user-auth` | **Spec**: [specs/001-user-auth/spec.md](spec.md) | **Plan**: [specs/001-user-auth/plan.md](plan.md)

## Phase 1: Setup

*Goal: Initialize the Auth Server environment and configure shared database infrastructure.*

- [x] T001 Initialize Node.js Auth Server project in `auth-server/` with `npm init -y`
- [x] T002 Install Better Auth and Hono dependencies in `auth-server/package.json`
- [x] T003 Configure TypeScript (`auth-server/tsconfig.json`) and scripts (`package.json`) for Auth Server
- [x] T004 Create Auth Server entry point and Better Auth config in `auth-server/src/index.ts`
- [x] T005 [P] Install `asyncpg` dependency in `backend/pyproject.toml`
- [x] T006 Define shared `.env` configuration file in project root

## Phase 2: Foundational

*Goal: Establish database schema and backend verification logic.*

- [x] T007 Generate Better Auth database schema migration in `auth-server/prisma/schema.prisma` (or equivalent for Better Auth CLI)
- [x] T008 Apply database migration to Neon Postgres using Better Auth CLI
- [x] T009 [P] Create Python session verification model in `backend/src/models/auth.py`
- [x] T010 Implement session verification dependency in `backend/src/api/dependencies.py` to query Neon DB
- [x] T011 [P] Configure Docusaurus proxy in `docusaurus.config.ts` (or middleware) to route `/api/auth/*` to Auth Server

## Phase 3: User Story 1 - Sign Up with Email and Password

*Goal: Enable new users to register and verify their accounts.*

**Independent Test**: Register a new user via API and verify record in DB and email sent.

- [x] T012 [US1] Configure Email/Password provider in `auth-server/src/auth.ts`
- [x] T013 [P] [US1] Implement Better Auth Client initialization in `src/lib/auth-client.ts`
- [x] T014 [US1] Create Sign Up page component in `src/pages/sign-up.tsx`
- [x] T015 [US1] Implement Sign Up form logic with error handling in `src/components/Auth/SignUpForm.tsx`
- [x] T016 [P] [US1] Configure SMTP/Email provider in Auth Server for verification emails

## Phase 4: User Story 2 - Sign In with Email and Password

*Goal: Allow returning users to log in.*

**Independent Test**: Login with valid credentials succeeds; invalid fails.

- [x] T017 [US2] Create Sign In page component in `src/pages/sign-in.tsx`
- [x] T018 [US2] Implement Sign In form logic in `src/components/Auth/SignInForm.tsx`
- [x] T019 [US2] Add Logout button component in `src/components/Auth/LogoutButton.tsx`

## Phase 5: User Story 3 - Sign In with Google

*Goal: Enable OAuth login and account linking.*

**Independent Test**: OAuth flow completes, user created/linked.

- [x] T020 [US3] Configure Google OAuth provider in `auth-server/src/auth.ts`
- [x] T021 [US3] Add "Sign in with Google" button to `src/components/Auth/SignInForm.tsx`
- [x] T022 [US3] Add "Sign in with Google" button to `src/components/Auth/SignUpForm.tsx`
- [x] T023 [US3] Verify account linking behavior configuration in `auth-server/src/auth.ts`

## Phase 6: User Story 4 - Chatbot Access Control

*Goal: Gate Chatbot API access.*

**Independent Test**: Unauthenticated request to Chatbot API returns 401; Authenticated returns 200.

- [x] T024 [US4] Implement Protected Route wrapper in `src/components/ProtectedRoute.tsx`
- [x] T025 [US4] Apply Protected Route to Chatbot page in `src/pages/chatbot.tsx` (or equivalent)
- [x] T026 [P] [US4] Apply `verify_session` dependency to Chatbot router in `backend/src/api/routers/chatbot.py`
- [x] T027 [US4] Handle 401 Unauthorized responses in frontend Chatbot client

## Phase 7: Polish

*Goal: Finalize UI/UX and ensure security.*

- [x] T028 Audit environment variables usage across all services
- [x] T029 Clean up any temporary debug logs
- [x] T030 Update documentation/README with auth setup instructions

## Dependencies

- Phase 1 & 2 must be completed before Phase 3-6.
- Phase 3 (Sign Up) and Phase 4 (Sign In) can be developed in parallel but share UI components.
- Phase 5 (Google Auth) depends on Phase 1 (Auth Server) but is independent of Email/Pass logic.
- Phase 6 (Access Control) depends on Phase 2 (Backend Verification).

## Implementation Strategy

1.  **MVP**: Setup Auth Server + Backend Verification + Email Sign Up/In.
2.  **Enhancement**: Google OAuth.
3.  **Finalize**: Chatbot Gating.