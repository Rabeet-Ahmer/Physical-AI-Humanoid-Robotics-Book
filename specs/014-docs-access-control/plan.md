# Implementation Plan - Docs Access Control

## Technical Context
The project uses Docusaurus with `better-auth`. An existing `src/theme/Root.js` implements a global route guard using an "allowlist" strategy. The goal is to refactor this to strictly protect `/docs/` URLs while allowing public access to other areas.

## Constitution Check
- **Python-First**: Exception (Frontend logic).
- **Tech Stack**: Compliant (React/Docusaurus).
- **No Hallucinations**: Verified existing `Root.js`.

## Phase 1: Design
- [x] **Research**: Strategy defined (Blocklist `/docs`). [research.md](./research.md)
- [x] **Data Model**: UI state defined. [data-model.md](./data-model.md)
- [x] **Quickstart**: Test steps defined. [quickstart.md](./quickstart.md)

## Phase 2: Implementation

### Step 1: Refactor Root Component
- **Task**: Modify `src/theme/Root.js`.
- **Details**: 
    - Remove `publicRoutes` array.
    - Implement `isProtected = (path) => path.startsWith('/docs')`.
    - Update logic: If `isProtected` and `!session`, redirect.
    - Ensure SSG protection (return placeholder during build for protected routes).
- **Files**: `src/theme/Root.js`

### Step 2: Verify ProtectedRoute Component
- **Task**: Review `src/components/ProtectedRoute.tsx`.
- **Details**: If it's redundant (since `Root.js` handles global protection), mark it for deletion or keep it for specific component wrapping. Given the requirement is global middleware, `Root.js` is the primary implementation. We will leave `ProtectedRoute.tsx` as is or deprecate it if unused. For this plan, we focus on `Root.js`.

### Step 3: Polish
- **Task**: Ensure "Loading" state in `Root.js` looks good (maybe use a Spinner component).