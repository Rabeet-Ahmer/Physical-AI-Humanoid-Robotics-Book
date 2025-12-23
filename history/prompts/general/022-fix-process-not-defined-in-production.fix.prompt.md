---
id: 022-fix-process-not-defined-in-production
title: Fix "process is not defined" in Production
stage: fix
date: 2025-12-23
surface: agent
model: gemini-2.0-flash-exp
feature: general
branch: main
user: user
command: fix-process-production
labels: [bugfix, production, browser, error]
links: null
files:
  - src/lib/auth-client.ts
  - src/utils/api.ts
tests:
  - Manual verification: Inspect code for safe `typeof process !== 'undefined'` checks.
  - Deployment verification: Error should disappear in browser console.
---

# Prompt

Again in the deployed site chat is not opening and in the console I am seeing this error:
`Uncaught ReferenceError: process is not defined` in `main.js`.

# Response

The error persisted because usages of `process.env.NODE_ENV` and `process.env.NEXT_PUBLIC...` in `src/lib/auth-client.ts` and `src/utils/api.ts` were not fully guarded against the `process` object being undefined in the production browser environment.

**Action:**
- Updated `src/lib/auth-client.ts` and `src/utils/api.ts` to use `typeof process !== 'undefined'` checks before accessing any properties on `process`.
- This ensures that if `process` is missing (as it is in standard browser builds), the code gracefully falls back to the defined default values instead of crashing.

**Deployment:**
- Push these changes to GitHub.
- Vercel will rebuild the frontend, and the error should be resolved.
