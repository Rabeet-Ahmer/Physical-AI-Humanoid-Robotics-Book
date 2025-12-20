# Quickstart: Docs Access Control

## Testing Access Control

1. **Start the App**:
   ```bash
   npm start
   ```
2. **Visit as Guest**:
   - Navigate to `http://localhost:3000/docs/intro` (or any docs page).
   - Verify you are redirected to `/sign-up` (or `/sign-in`).
   - Navigate to `http://localhost:3000/` (Home).
   - Verify you can see the homepage.

3. **Visit as User**:
   - Sign in at `/sign-in`.
   - Navigate to `http://localhost:3000/docs/intro`.
   - Verify you can read the content.
