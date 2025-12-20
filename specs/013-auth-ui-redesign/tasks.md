---
description: "Task list for Auth UI Redesign"
---

# Tasks: Auth UI Redesign

**Input**: Design documents from `/specs/013-auth-ui-redesign/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md
**Tests**: OPTIONAL - None requested.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel
- **[Story]**: [US1]..[US4] (SignIn Creds, SignIn Google, SignUp Creds, SignUp Google)

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic assets

- [x] T001 Create `src/components/Auth/GoogleIcon.tsx` for inline SVG asset
- [x] T002 Create `src/components/Auth/Auth.module.css` with CSS variables and container styles

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core UI components used by all stories

- [x] T003 Create `src/components/Auth/AuthLayout.tsx` wrapper component using `Auth.module.css`
- [x] T004 Define typography and button styles in `src/components/Auth/Auth.module.css`

**Checkpoint**: Shared layout and styles ready.

## Phase 3: User Story 1 - Sign In with Credentials (Priority: P1)

**Goal**: Users can sign in with Email/Password using the new design.

- [x] T005 [US1] Refactor `src/components/Auth/SignInForm.tsx` to use `AuthLayout` and `Auth.module.css`
- [x] T006 [US1] Implement Email/Password fields and Submit button in `src/components/Auth/SignInForm.tsx`
- [x] T007 [US1] Implement error handling display in `src/components/Auth/SignInForm.tsx`

**Checkpoint**: Clean Sign In form (Email/Pass) visible and functional.

## Phase 4: User Story 2 - Sign In with Google (Priority: P1)

**Goal**: Users can sign in with Google.

- [x] T008 [US2] Add "Sign in with Google" button to `src/components/Auth/SignInForm.tsx`
- [x] T009 [US2] Integrate `GoogleIcon` into the button in `src/components/Auth/SignInForm.tsx`
- [x] T010 [US2] Verify Google Auth flow integration in `src/components/Auth/SignInForm.tsx`

**Checkpoint**: Google Auth added to Sign In.

## Phase 5: User Story 3 - Sign Up with Credentials (Priority: P1)

**Goal**: Users can sign up with Name/Email/Password.

- [x] T011 [US3] Refactor `src/components/Auth/SignUpForm.tsx` to use `AuthLayout` and `Auth.module.css`
- [x] T012 [US3] Implement Name, Email, Password fields in `src/components/Auth/SignUpForm.tsx`
- [x] T013 [US3] Implement Submit button and error handling in `src/components/Auth/SignUpForm.tsx`

**Checkpoint**: Clean Sign Up form (Creds) visible and functional.

## Phase 6: User Story 4 - Sign Up with Google (Priority: P1)

**Goal**: Users can sign up with Google.

- [x] T014 [US4] Add "Sign up with Google" button to `src/components/Auth/SignUpForm.tsx`
- [x] T015 [US4] Verify Google Auth flow integration in `src/components/Auth/SignUpForm.tsx`

**Checkpoint**: Google Auth added to Sign Up.

## Phase 7: Polish & Integration

**Purpose**: Final page-level integration and responsiveness.

- [x] T016 [P] Update `src/pages/sign-in.tsx` to ensure correct container centering
- [x] T017 [P] Update `src/pages/sign-up.tsx` to ensure correct container centering
- [x] T018 Verify mobile responsiveness for all Auth components
- [x] T019 Verify error state visuals matches design

## Dependencies & Execution Order

- **Foundational (Phase 2)** blocks all User Stories.
- **US1 & US2** (Sign In) can run in parallel with **US3 & US4** (Sign Up) if different developers work on `SignInForm` and `SignUpForm`.
- **US2** depends on **US1** (modifying same file).
- **US4** depends on **US3** (modifying same file).

## Implementation Strategy

1. **Foundation**: Build `AuthLayout` and CSS.
2. **SignIn MVP**: Build US1 (Creds) -> Verify.
3. **SignIn Full**: Add US2 (Google) -> Verify.
4. **SignUp MVP**: Build US3 (Creds) -> Verify.
5. **SignUp Full**: Add US4 (Google) -> Verify.
