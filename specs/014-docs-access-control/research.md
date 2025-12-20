# Research: Docs Access Control

## Technical Context
- **Framework**: Docusaurus v3 (React)
- **Auth Library**: `better-auth` (client: `src/lib/auth-client.ts`)
- **Existing Implementation**: `src/theme/Root.js` exists with an allowlist approach.

## Decisions

### 1. Middleware Location
- **Decision**: Modify existing `src/theme/Root.js`.
- **Rationale**: Docusaurus uses `Root` component to wrap the entire application. It's the standard place for global auth guards.

### 2. Access Control Strategy
- **Decision**: Switch from "Allowlist" (Public Routes) to "Blocklist" (Protected Routes: `/docs/*`).
- **Rationale**: 
    - The requirement specifically targets `/docs/` urls.
    - Acceptance criteria explicitly state other pages (like `/blog`) should be accessible.
    - Maintaining an exhaustive public allowlist in Docusaurus is difficult (plugins, assets, etc.).
    - Protecting `/docs` is the specific business goal.

### 3. Client-Side vs Server-Side
- **Decision**: Hybrid approach (already in `Root.js`).
    - **Server/SSG**: Render placeholder/redirect for protected routes to prevent content leakage in static HTML.
    - **Client**: `useEffect` redirect for unauthenticated users on protected routes.
- **Rationale**: Docusaurus is an SSG. We must ensure the static HTML for `/docs` doesn't contain the textbook content if we want strict security (though `Root` wraps `children`. If we return null/placeholder in `Root` for protected paths on server render, the sensitive content (children) won't be rendered into the HTML. This is correct.)

### 4. User Experience
- **Decision**: Redirect to `/sign-up` (as per prompt request "If any user wants to access these /docs urls they have to sign-up").
- **Rationale**: Explicit user instruction.

## Unknowns Resolved
- **Current State**: Found `src/theme/Root.js`. It needs refactoring to match the specific "Protect /docs" requirement instead of "Block everything else".
