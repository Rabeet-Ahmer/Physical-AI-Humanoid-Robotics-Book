---
id: 029-remove-auth-integration
title: Remove Authentication Integration from Main
stage: refactor
date: 2025-12-24
surface: agent
model: gemini-2.0-flash-exp
feature: user-auth
branch: main
user: user
command: remove-auth
labels: [refactor, removal, rollback]
links: null
files:
  - src/theme/Layout/index.tsx
  - src/lib/auth-client.ts
  - src/pages/sign-in.tsx
  - src/pages/sign-up.tsx
tests:
  - Manual verification: Protected routes (/docs/*) should now be accessible without login.
  - Manual verification: /sign-in and /sign-up pages should 404.
---

# Prompt

Remove the auth from main branch because it is not working, we will implement this in a separate branch so that uaer experience doesn't compromise

# Response

I have completely removed the authentication integration from the frontend to restore unrestricted access to the documentation.

**Actions:**
1.  **Modified `src/theme/Layout/index.tsx`**: Removed `better-auth` checks, `ProtectedLayout` logic, and redirects. It now acts as a simple wrapper for the Chat Widget.
2.  **Deleted Files**:
    -   `src/lib/auth-client.ts` (Auth client config)
    -   `src/pages/sign-in.tsx` (Login page)
    -   `src/pages/sign-up.tsx` (Registration page)

**Note:**
-   The `auth-server` folder and `package.json` dependencies remain in the repository (to preserve your backend work), but the *frontend* is no longer connected to them.
-   You can now safely create a new branch to work on Auth without affecting the live site.
