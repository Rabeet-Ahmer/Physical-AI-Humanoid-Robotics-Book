---
id: 018-fix-auth-client-baseurl
title: Fix Auth Client BaseURL for Local Dev
stage: fix
date: 2025-12-23
surface: agent
model: gemini-2.0-flash-exp
feature: user-auth
branch: main
user: user
command: fix-auth-fetch-error
labels: [bugfix, auth, better-auth, network]
links: null
files:
  - src/lib/auth-client.ts
tests:
  - Manual verification: Sign-in requests should now target http://localhost:3001/api/auth in development.
---

# Prompt

When I am trying to sign-in it shows an error:
```
ERROR
Failed to fetch
TypeError: Failed to fetch
...
```

# Response

The "Failed to fetch" error occurred because the `authClient` in the frontend was hardcoded to use the production Vercel URL (`https://...`), even when running locally.

**Action:**
- Updated `src/lib/auth-client.ts` to dynamically set the `baseURL`:
  - **Development:** `http://localhost:3001/api/auth` (Matches your local `auth-server` port).
  - **Production:** `https://physical-ai-humanoid-robotics-book-coral-seven.vercel.app/api/auth`.

**Next Steps for You:**
1.  Ensure your **Auth Server** is running (`npm run dev` in `auth-server` folder).
2.  Refresh your Docusaurus frontend.
3.  Try signing in again.
