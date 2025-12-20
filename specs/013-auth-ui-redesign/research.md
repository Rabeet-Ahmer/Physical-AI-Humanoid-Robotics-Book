# Research: Auth UI Redesign

## Technical Context
- **Frontend Framework**: Docusaurus (React)
- **Auth Library**: `better-auth`
- **Backend**: `auth-server` (Hono/Node.js)
- **Styling**: React inline styles (current) -> CSS Modules (target)

## Decisions

### 1. Styling Strategy
- **Decision**: Adopt CSS Modules (`*.module.css`) for authentication components.
- **Rationale**: Encapsulates styles, prevents global namespace pollution, and aligns with Docusaurus best practices.
- **Implementation**: 
  - Create `src/components/Auth/Auth.module.css` for shared container styles.
  - Create specific module files if components diverge significantly.

### 2. Layout & Structure
- **Decision**: Implement a `AuthLayout` component (or shared container structure) within the forms.
- **Rationale**: The target design (Figma) typically implies a consistent card or split-view layout. A shared wrapper ensures consistency between Sign In and Sign Up.

### 3. Icons
- **Decision**: Use inline SVG for the Google logo.
- **Rationale**: Avoids adding an entire icon library dependency for a single asset.

### 4. Form Validation
- **Decision**: Continue using HTML5 validation + React state for custom errors (as currently implemented).
- **Rationale**: Simple and effective for the current requirements without adding form libraries like `react-hook-form` yet.

## Risks & Mitigations
- **Port Conflict**: `auth-client.ts` points to `localhost:3000`. Docusaurus defaults to `3000`.
  - **Mitigation**: User must run Auth Server on 3000 and Docusaurus on a different port (e.g., 3001), or update `auth-client.ts`. For this UI task, I will assume the user handles the environment or I will verify visual states primarily.

## Unknowns Resolved
- **Stack**: Confirmed Docusaurus + Better Auth.
- **Scope**: Frontend UI redesign only.
