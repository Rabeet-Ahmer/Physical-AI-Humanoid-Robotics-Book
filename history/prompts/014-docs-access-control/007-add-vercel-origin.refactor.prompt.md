# PHR: 007-add-vercel-origin

- **ID:** 007
- **TITLE:** Add Vercel Origin to Auth Server
- **STAGE:** refactor
- **DATE_ISO:** 2025-12-20
- **SURFACE:** agent
- **MODEL:** gemini-2.0-flash
- **FEATURE:** 014-docs-access-control
- **BRANCH:** 014-docs-access-control
- **USER:** user
- **COMMAND:** update-origins
- **LABELS:** ["auth-server", "cors", "production", "vercel"]
- **LINKS:** SPEC/TICKET/ADR/PR (null)
- **FILES_YAML:** 
  - auth-server/src/auth.ts
  - auth-server/src/index.ts
  - src/utils/api.ts
- **TESTS_YAML:**
  - none
- **PROMPT_TEXT:** This is where my frontened is hosted "https://physical-ai-humanoid-robotics-book-coral-seven.vercel.app/", your task is to update the code in @auth-server\** and in @src\**, keep the changes minimal
- **RESPONSE_TEXT:** Updated auth-server/src/auth.ts and index.ts to explicitly trust the provided Vercel URL. Updated src/utils/api.ts to use NEXT_PUBLIC_RAG_API_URL environment variable.

## Outcome
Auth server configured to accept requests from the deployed frontend. Frontend prepared for production backend URL.
