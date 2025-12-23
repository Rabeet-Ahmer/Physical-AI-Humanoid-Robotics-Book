---
id: 023-restructure-auth-server-for-vercel
title: Restructure Auth Server for Vercel
stage: refactor
date: 2025-12-23
surface: agent
model: gemini-2.0-flash-exp
feature: user-auth
branch: main
user: user
command: restructure-vercel-api
labels: [refactor, vercel, structure, deployment]
links: null
files:
  - auth-server/api/index.ts
  - auth-server/src/index.ts
  - auth-server/vercel.json
  - auth-server/tsconfig.json
tests:
  - Manual verification: File moved to api/index.ts.
  - Manual verification: Imports updated.
  - Manual verification: tsconfig includes api/.
---

# Prompt

Ok then implement these

# Response

I have restructured the `auth-server` to follow Vercel's standard "Zero Config" conventions:

**Changes:**
1.  **Created `auth-server/api` folder**: This is the magic folder Vercel looks for.
2.  **Moved Entry Point**: Moved `src/index.ts` to `api/index.ts`.
3.  **Updated Import**: Changed the import in `api/index.ts` to find `../src/auth.js` correctly.
4.  **Deleted `vercel.json`**: It's no longer needed as the file structure now dictates the routing.
5.  **Updated `tsconfig.json`**: Included the new `api` folder in the TypeScript configuration.

**Next Steps for You:**
-   Push these changes to GitHub.
-   Vercel should automatically detect the new API route (`/api`) without any custom configuration.
