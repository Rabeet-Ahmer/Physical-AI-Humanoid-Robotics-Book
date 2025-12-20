# Quickstart: User Authentication

## Prerequisites

- Node.js 18+ (for Auth Server)
- Python 3.10+ (for Backend)
- Neon Database URL

## Setup

1.  **Environment Variables**:
    Create `.env` in root:
    ```env
    DATABASE_URL=postgres://...
    BETTER_AUTH_SECRET=...
    BETTER_AUTH_URL=http://localhost:3000
    GOOGLE_CLIENT_ID=...
    GOOGLE_CLIENT_SECRET=...
    ```

2.  **Auth Server (Node.js)**:
    Navigate to `auth-server/` (to be created):
    ```bash
    cd auth-server
    npm install
    npm run migrate # Setup DB schema
    npm run dev     # Starts on port 3000
    ```

3.  **Backend (Python)**:
    Navigate to `backend/`:
    ```bash
    uv sync
    uv run fastapi dev main.py # Starts on port 8000
    ```

4.  **Frontend (Docusaurus)**:
    Navigate to root:
    ```bash
    npm install
    npm start # Starts on port 3001 (configured to proxy /api/auth to 3000)
    ```

## Verification

1.  Open `http://localhost:3001`.
2.  Click "Sign In".
3.  Register with Email/Password.
4.  Verify redirect to Home.
5.  Access Chatbot page -> Should work.
6.  Logout.
7.  Access Chatbot page -> Should block/redirect.
