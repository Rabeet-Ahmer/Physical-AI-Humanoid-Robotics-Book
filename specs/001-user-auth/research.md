# Research: Add User Authentication

**Feature**: `001-user-auth` | **Date**: 2025-12-19

## Decisions

### 1. Authentication Framework
**Decision**: Use **Better Auth** (TypeScript) for Frontend and Schema Management.
**Rationale**: Explicit user requirement. Provides robust, modern auth features (OAuth, Sessions, Account Linking) out of the box for the web frontend.
**Implication**: The Python backend cannot "run" Better Auth directly. Instead, it will share the database and perform read-only verification of sessions created by the Better Auth client/server (Node.js/Next.js context).

### 2. Backend Session Verification
**Decision**: Implement a custom dependency in FastAPI to verify Better Auth sessions by querying the shared Neon database.
**Rationale**: Better Auth (by default) uses database-backed sessions. Since the backend is Python, we cannot use the Better Auth server-side library directly. Direct database query is the most reliable way to validate the session token.
**Mechanism**:
1.  Extract `better-auth.session_token` from cookies.
2.  Query `session` table: `SELECT * FROM session WHERE token = $1 AND expires_at > NOW()`.
3.  If valid, retrieve `user` data and attach to request.

### 3. Database Schema
**Decision**: Use Better Auth's default schema (generated via its CLI).
**Tables**: `user`, `session`, `account`, `verification`.
**Rationale**: Standard schema handles all requirements (OAuth, Email/Pass, Linking).

### 4. Frontend Integration (Docusaurus)
**Decision**: Use `better-auth/react` client in Docusaurus.
**Rationale**: Docusaurus is React-based. The client library simplifies API calls (signIn, signUp) and state management.

## Unknowns Resolved

- **Python Verification**: Resolved. Will query DB directly.
- **Frontend Testing**: Will use manual verification for now as no automated frontend test suite exists. Backend tests will mock the DB to test the verification logic.
- **Database Connection**: Both environments (Node/Python) will use the same `DATABASE_URL`.

## Integration Patterns

### Backend (FastAPI)
```python
# Pseudo-code for dependency
async def get_current_user(request: Request, db: AsyncConnection):
    token = request.cookies.get("better-auth.session_token")
    if not token:
        return None
    session = await db.fetch_one("SELECT * FROM session WHERE token=$1 AND expires_at > NOW()", token)
    if not session:
        return None
    user = await db.fetch_one("SELECT * FROM \"user\" WHERE id=$1", session['userId'])
    return user
```

### Frontend (Better Auth Client)
```typescript
import { createAuthClient } from "better-auth/react"
export const authClient = createAuthClient({
    baseURL: "http://localhost:3000" // API Base URL (if different)
})
```
