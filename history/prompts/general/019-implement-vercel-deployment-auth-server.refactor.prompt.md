---
id: 019-implement-vercel-deployment-auth-server
title: Implement Vercel Deployment for Auth Server
stage: refactor
date: 2025-12-23
surface: agent
model: gemini-2.0-flash-exp
feature: user-auth
branch: main
user: user
command: prepare-vercel-auth
labels: [deployment, vercel, auth-server, hono]
links: null
files:
  - auth-server/src/index.ts
  - auth-server/vercel.json
tests:
  - Manual verification: Auth Server still runs locally with `npm run dev`.
  - Deployment verification: Added `vercel.json` and exported app for serverless compatibility.
---

# Prompt

Ok then implement the code for vercel deployment, keep it in the same main branch

# Response

I have prepared the `auth-server` for Vercel deployment while maintaining local development compatibility.

**Changes:**
1.  **Modified `auth-server/src/index.ts`**:
    *   Added `export default app` so Vercel's Node.js runtime can handle the Hono application.
    *   Wrapped the `serve()` function (from `@hono/node-server`) in a check for `process.env.NODE_ENV !== "production"`. This prevents the server from trying to bind to a local port when running as a Vercel Serverless Function.
2.  **Created `auth-server/vercel.json`**:
    *   Configured rewrites to route all `/api/auth/*` requests to the entry point.
    *   Explicitly set the runtime to `vercel-node`.

**Deployment Instructions for Vercel:**
1.  Set the **Root Directory** to `auth-server` in Vercel project settings.
2.  Add your environment variables (`DATABASE_URL`, `BETTER_AUTH_SECRET`, etc.) in Vercel.
3.  Deploy!
