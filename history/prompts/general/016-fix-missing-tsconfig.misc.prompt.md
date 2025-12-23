---
id: 016-fix-missing-tsconfig
title: Fix Missing tsconfig.json
stage: misc
date: 2025-12-23
surface: agent
model: gemini-2.0-flash-exp
feature: general
branch: main
user: user
command: fix-tsconfig
labels: [fix, typescript, config]
links: null
files:
  - tsconfig.json
tests:
  - Validated existence of src/lib/auth-client.ts.
  - Created standard Docusaurus tsconfig.json in root.
---

# Prompt

In @src/theme/Layout/index.tsx are module not found errors persists on lines 2, 3, 5, 6. Just resolve these errors

# Response

The "Module not found" errors for Docusaurus aliases (like `@theme-original`, `@docusaurus/router`) were caused by the absence of a `tsconfig.json` file in the project root. This file is required for TypeScript to understand the project structure and mapping of these virtual modules.

**Action:**
- Created `tsconfig.json` in the root directory, extending `@docusaurus/tsconfig/tsconfig.json`.

This should resolve the red squiggles and build errors related to module resolution.
