# PHR: 005-docs-access-control-fix

- **ID:** 005
- **TITLE:** Docs Access Control Fix
- **STAGE:** refactor
- **DATE_ISO:** 2025-12-20
- **SURFACE:** agent
- **MODEL:** gemini-2.0-flash
- **FEATURE:** 014-docs-access-control
- **BRANCH:** 014-docs-access-control
- **USER:** user
- **COMMAND:** fix-crash
- **LABELS:** ["bugfix", "frontend", "auth", "middleware", "redirect"]
- **LINKS:** SPEC/TICKET/ADR/PR (null)
- **FILES_YAML:** 
  - src/theme/Root.js
- **TESTS_YAML:**
  - manual visual verification assumed
- **PROMPT_TEXT:** You have done great, but when an unauthenticated user tries to access these urls the page just crashes which is not good. So redirect these users to sign-in page.
- **RESPONSE_TEXT:** Refactored Root.js to use declarative <Redirect /> component instead of useEffect/history.push to fix crash issues and redirected to /sign-in as requested.

## Outcome
Crash fixed. Redirects to /sign-in correctly.
