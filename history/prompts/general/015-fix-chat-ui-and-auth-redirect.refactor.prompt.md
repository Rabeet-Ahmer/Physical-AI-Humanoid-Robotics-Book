---
id: 015-fix-chat-ui-and-auth-redirect
title: Fix Chat UI and Auth Redirect
stage: refactor
date: 2025-12-23
surface: agent
model: gemini-2.0-flash-exp
feature: general
branch: main
user: user
command: fix-chat-and-auth
labels: [bugfix, ui, auth, chat-widget]
links: null
files:
  - src/components/ChatWidget/ChatPanel.module.css
  - src/components/ChatWidget/ChatIcon.module.css
  - src/theme/Layout/index.tsx
tests:
  - Manual verification of Chat UI opening (via z-index fix).
  - Manual verification of Auth Redirect (via LayoutWrapper logic update).
---

# Prompt

Hi there, I am testing the site but got some unwants. The AI chat ui is not opening (I am clicking the chat icon), and also if I go to a url for which authentication is required returns a "Protected Page Content" but I want to redirect the User to "Sign-in" page. Do all work within the same main branch.

# Response

I addressed two reported issues:

1.  **Chat UI not opening:** The `ChatIcon` and `ChatPanel` were likely being obscured by other elements due to insufficient z-index values.
    *   **Fix:** Increased `z-index` in `ChatIcon.module.css` and `ChatPanel.module.css` to `2147483647` (maximum 32-bit signed integer) to ensure they always appear on top.

2.  **Protected Page Redirect:** Users were seeing the server-side "Protected Content" fallback instead of being redirected to the Sign-in page.
    *   **Fix:** Updated `src/theme/Layout/index.tsx` (the global Layout wrapper) to:
        *   Improve the server-side fallback UI (added a "Sign In" button).
        *   Add a `useEffect` hook that explicitly calls `history.replace('/sign-in')` when a user is unauthenticated on a protected route (`/docs/*`), providing a robust backup to the declarative `<Redirect />`.
        *   Ensure the `isProtected` logic correctly identifies documentation pages.
