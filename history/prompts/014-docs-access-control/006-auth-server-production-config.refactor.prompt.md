# PHR: 006-auth-server-production-config

- **ID:** 006
- **TITLE:** Auth Server Production Configuration
- **STAGE:** refactor
- **DATE_ISO:** 2025-12-20
- **SURFACE:** agent
- **MODEL:** gemini-2.0-flash
- **FEATURE:** 014-docs-access-control
- **BRANCH:** 014-docs-access-control
- **USER:** user
- **COMMAND:** production-config
- **LABELS:** ["auth-server", "production", "cors", "env"]
- **LINKS:** SPEC/TICKET/ADR/PR (null)
- **FILES_YAML:** 
  - auth-server/src/auth.ts
  - auth-server/src/index.ts
  - src/lib/auth-client.ts
- **TESTS_YAML:**
  - none
- **PROMPT_TEXT:** I have deployed my frontend on vercel, so in @auth-server\** where should I update the code.
- **RESPONSE_TEXT:** Updated auth-server (auth.ts and index.ts) and frontend (auth-client.ts) to use environment variables for URLs, enabling communication between the Vercel frontend and the production auth server.

## Outcome
Code updated to support production URLs via environment variables.
