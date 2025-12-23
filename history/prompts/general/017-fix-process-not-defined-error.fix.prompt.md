---
id: 017-fix-process-not-defined-error
title: Fix "process is not defined" Error
stage: fix
date: 2025-12-23
surface: agent
model: gemini-2.0-flash-exp
feature: general
branch: main
user: user
command: fix-process-error
labels: [bugfix, javascript, browser, runtime-error]
links: null
files:
  - src/utils/api.ts
tests:
  - Manual verification: Browser console should no longer show "Uncaught ReferenceError: process is not defined".
---

# Prompt

On browser console I can find this error because of which the site is not rendered:
```
Uncaught ReferenceError: process is not defined
    at eval (api.ts:4:1)
    at ./src/utils/api.ts (main.js:2274:1)
...
```

# Response

The error `Uncaught ReferenceError: process is not defined` occurred because `process.env` was being accessed directly in the client-side code (`src/utils/api.ts`) without a polyfill or definition in the browser environment.

**Action:**
- Modified `src/utils/api.ts` to include a safe check `typeof process !== 'undefined'` before accessing `process.env`.
- If `process` is undefined, it falls back to the default URL (`http://127.0.0.1:8000/agent`).

This ensures the code runs without error in the browser while still allowing environment variable overrides if the build process injects them.
