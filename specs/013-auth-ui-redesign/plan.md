# Implementation Plan - Auth UI Redesign

## Technical Context

The project is a **Docusaurus** website (React-based static site generator) integrated with **Better Auth** for authentication. The backend `auth-server` handles the auth logic and database connections. The current task is to redesign the existing `SignInForm` and `SignUpForm` components to match a modern, clean aesthetic (Figma reference), supporting Email/Password and Google authentication.

**Key Technologies:**
- **Frontend**: Docusaurus (React 19), CSS Modules
- **Auth**: `better-auth/react`
- **Icons**: Inline SVGs (for Google logo)

## Constitution Check

### Backend Policy (Python-First)
- **Status**: **Exception / Not Applicable**
- **Reasoning**: This feature is purely Frontend (React/Docusaurus). The existing `auth-server` is Node.js/TypeScript based. We are *not* modifying the backend logic, only the UI. The Constitution's "Python-First" applies to *backend* development. Since we are redesigning the UI for an existing TS backend, we proceed with TS/React for the frontend.

### Tech Stack Adherence
- **Status**: **Compliant**
- **Reasoning**: We are using the existing stack (React, Docusaurus). No new forbidden libraries are introduced.

### No Hallucinations
- **Status**: **Compliant**
- **Reasoning**: We verified the stack (`better-auth`, `docusaurus`) and will use standard CSS Modules.

## Phase 1: Design & Contracts

- [x] **Research**: Validated stack and styling approach. [research.md](./research.md)
- [x] **Data Model**: Defined User/Session entities. [data-model.md](./data-model.md)
- [x] **Contracts**: Documented Better Auth usage. [contracts/README.md](./contracts/README.md)
- [x] **Quickstart**: defined run steps. [quickstart.md](./quickstart.md)

## Phase 2: Implementation

### Step 1: Styles & Assets
- **Task**: Create CSS Module and assets.
- **Files**: `src/components/Auth/Auth.module.css`
- **Details**: Define classes for container, form groups, inputs, buttons (primary & social), and error messages.
- **Validation**: Check styles against design description (modern, clean).

### Step 2: Component Refactoring (Sign In)
- **Task**: Update `SignInForm.tsx` to use new styles and layout.
- **Files**: `src/components/Auth/SignInForm.tsx`
- **Details**: Replace inline styles with CSS classes. Implement the Google button with the SVG icon.
- **Validation**: Verify render and responsiveness.

### Step 3: Component Refactoring (Sign Up)
- **Task**: Update `SignUpForm.tsx` to use new styles and layout.
- **Files**: `src/components/Auth/SignUpForm.tsx`
- **Details**: Match the style of Sign In form. Ensure Name field is properly styled.
- **Validation**: Verify render and responsiveness.

### Step 4: Page Wrappers
- **Task**: Ensure `src/pages/sign-in.tsx` and `src/pages/sign-up.tsx` provide correct container context.
- **Files**: `src/pages/sign-in.tsx`, `src/pages/sign-up.tsx`
- **Details**: Adjust layout containers if necessary to center the auth cards.
- **Validation**: Full page review.

## Phase 3: Testing & Verification

### Manual Testing
- [ ] Start Auth Server and Frontend.
- [ ] Verify Sign In UI (Desktop & Mobile).
- [ ] Verify Sign Up UI (Desktop & Mobile).
- [ ] Test "Sign in with Google" button (visual feedback).
- [ ] Test Error states (invalid password) -> Verify error message styling.