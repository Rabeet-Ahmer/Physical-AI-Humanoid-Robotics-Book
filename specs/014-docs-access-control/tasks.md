---
description: "Task list for Docs Access Control"
---

# Tasks: Docs Access Control

**Input**: Design documents from `/specs/014-docs-access-control/`
**Prerequisites**: plan.md, spec.md, research.md
**Tests**: OPTIONAL - None requested.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel
- **[Story]**: [US1] (Restrict Unauth), [US2] (Allow Auth)

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization

- [x] T001 Verify `src/theme/Root.js` exists and is writable

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core logic preparation

- [x] T002 [US1] Define `isProtected` helper function logic in `src/theme/Root.js` (replacing allowlist)

## Phase 3: User Story 1 - Restrict Unauthenticated Access (Priority: P1)

**Goal**: Block access to /docs/ for guests.

- [x] T003 [US1] Implement `isProtected` check for `/docs/*` paths in `src/theme/Root.js`
- [x] T004 [US1] Update `useEffect` to redirect unauthenticated users to `/sign-up` in `src/theme/Root.js`
- [x] T005 [US1] Implement server-side rendering guard (placeholder) for protected routes in `src/theme/Root.js`

**Checkpoint**: Guests attempting to access /docs/ are redirected.

## Phase 4: User Story 2 - Allow Authenticated Access (Priority: P1)

**Goal**: Allow members to access content.

- [x] T006 [US2] Verify logic allows session-holding users to pass through in `src/theme/Root.js` (Code review/Adjustment)

**Checkpoint**: Members can access /docs/.

## Phase 5: Polish & Integration

**Purpose**: UX improvements.

- [x] T007 [P] Improve "Verifying Access..." loading state UI in `src/theme/Root.js`
- [x] T008 [P] Run quickstart validation steps

## Dependencies & Execution Order

- **US1** and **US2** are implemented together in `Root.js`.
- **Polish** can follow immediately.

## Implementation Strategy

1. **Refactor Root.js**: Replace allowlist with blocklist (`/docs`).
2. **Verify**: Check Guest access (Redirect).
3. **Verify**: Check Member access (Allow).
4. **Polish**: Improve UI.