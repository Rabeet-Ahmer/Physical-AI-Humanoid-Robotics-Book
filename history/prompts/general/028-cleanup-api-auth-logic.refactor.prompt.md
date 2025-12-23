---
id: 028-cleanup-api-auth-logic
title: Cleanup Authentication Logic in Chat API
stage: refactor
date: 2025-12-24
surface: agent
model: gemini-2.0-flash-exp
feature: general
branch: main
user: user
command: cleanup-api-auth
labels: [refactor, api, cleanup]
links: null
files:
  - src/utils/api.ts
tests:
  - Manual verification: The 401 redirect and credentials: 'include' have been removed.
---

# Prompt

I can see that in @src/utils/api.ts you has implpemented authentication but we moved athentication to the book so clean this

# Response

I have removed the authentication-specific logic from `src/utils/api.ts`. 

**Changes:**
- Removed `credentials: 'include'` from the `fetch` call.
- Removed the manual `response.status === 401` check and the redirect to `/sign-in`.

These are now handled by the Docusaurus layout/route protection logic, keeping the API utility focused strictly on communication with the RAG agent.
