# Quickstart: Auth UI Redesign

## Running the Application

1. **Start the Auth Server**
   The backend service is required for authentication calls.
   ```bash
   cd auth-server
   npm install
   npm run dev
   # Ensure it runs on port 3000 or update src/lib/auth-client.ts
   ```

2. **Start the Docusaurus Frontend**
   ```bash
   # From root directory
   npm install
   npm start
   # Docusaurus usually starts on port 3000. If port 3000 is taken by auth-server, 
   # Docusaurus will ask to use another port (e.g., 3001). This is expected.
   ```

3. **Accessing the UI**
   - **Sign In Page**: [http://localhost:3000/sign-in](http://localhost:3000/sign-in) (or 3001)
   - **Sign Up Page**: [http://localhost:3000/sign-up](http://localhost:3000/sign-up) (or 3001)

## Testing the UI
1. Navigate to `/sign-in`.
2. Verify the layout matches the design (clean, card-based/split).
3. Try "Sign in with Google" (requires configured `.env` in `auth-server`).
4. Try invalid Email/Password to see error styling.
