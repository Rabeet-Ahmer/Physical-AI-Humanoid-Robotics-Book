# API Contracts

The API contracts are implicitly defined by the `better-auth` library client.

## Base URL
- Configured in `src/lib/auth-client.ts` (Default: `http://localhost:3000`)

## Endpoints Used

### Email Auth
- **Sign In**: `betterAuth.signIn.email({ email, password })`
- **Sign Up**: `betterAuth.signUp.email({ email, password, name })`

### Social Auth
- **Sign In**: `betterAuth.signIn.social({ provider: "google" })`

*See `better-auth` documentation for full API specification.*
