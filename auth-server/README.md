# Auth Server

This is the Node.js service running Better Auth.

## Setup

1. Install dependencies:
   ```bash
   npm install
   ```

2. Configure `.env` in the root of the repo.

3. Run migration (if not done):
   ```bash
   npx better-auth migrate
   ```

4. Start server:
   ```bash
   npm run dev
   ```

## API

The server runs on port 3000 and exposes `/api/auth/*` endpoints.
