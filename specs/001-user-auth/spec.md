# Feature Specification: Add User Authentication

**Feature Branch**: `001-user-auth`  
**Created**: 2025-12-19  
**Status**: Draft  
**Input**: User description: "Now we want to implement sign-up/sign-in authentication functionality using **Better AUth** framework... [full description]"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Sign Up with Email and Password (Priority: P1)

As a new user, I want to create an account using my email and a username so that I can access the chatbot functionality, with an email verification sent in the background.

**Why this priority**: Core functionality for onboarding users who prefer standard credentials or do not use Google.

**Independent Test**: Can be tested by calling the registration API with a new email/username and verifying the user is created in the database, a session is started, and a verification email is dispatched.

**Acceptance Scenarios**:

1. **Given** a visitor with a unique email and username, **When** they submit the registration form, **Then** a new user record is created in Neon Postgres, the password is hashed, they are logged in immediately, and an email verification link is sent to their provided email address.
2. **Given** a visitor with an existing email or username, **When** they submit the registration form, **Then** they receive a clear error message indicating the conflict.

---

### User Story 2 - Sign In with Email and Password (Priority: P1)

As a returning user, I want to log in with my credentials so that I can resume using the chatbot.

**Why this priority**: Essential for returning users to access their account.

**Independent Test**: Verify login API accepts valid credentials and rejects invalid ones.

**Acceptance Scenarios**:

1. **Given** a registered user, **When** they submit valid credentials, **Then** they are authenticated and a session is established.
2. **Given** a user, **When** they submit invalid credentials, **Then** they receive an authentication error.

---

### User Story 3 - Sign In with Google (Priority: P1)

As a user, I want to sign in using my Google account so that I can access the system quickly without remembering another password.

**Why this priority**: Reduces friction for onboarding and login.

**Independent Test**: Mock OAuth callback to verify user creation/linking upon successful Google auth.

**Acceptance Scenarios**:

1. **Given** a new user, **When** they authenticate via Google, **Then** an account is created using their Google profile info and they are logged in.
2. **Given** an existing user (registered via email) who uses the same email for Google, **When** they authenticate via Google, **Then** the existing account is linked to the Google profile, and the user is logged in.

---

### User Story 4 - Chatbot Access Control (Priority: P1)

As a site owner, I want to restrict chatbot usage to logged-in users while keeping the book content public, to encourage registration.

**Why this priority**: Enforces the business rule regarding feature gating.

**Independent Test**: Attempt to access chatbot API/route without a session and verify denial. Attempt with session and verify access.

**Acceptance Scenarios**:

1. **Given** an unauthenticated visitor, **When** they attempt to access the chatbot, **Then** they are redirected to the login page or shown a prompt to authenticate.
2. **Given** an authenticated user, **When** they access the chatbot, **Then** they can interact with it freely.
3. **Given** an unauthenticated visitor, **When** they browse the book content, **Then** they have full access without being asked to log in.

---

### Edge Cases

- **Duplicate Account**: User tries to register with an email already used for Google Sign-In.
- **Session Expiry**: User session expires while using the chatbot -> should prompt for re-login without losing context if possible.
- **Database Downtime**: Handling of auth requests when Neon Postgres is unreachable.
- **Unverified Email**: User does not verify their email after signing up. Access to features is not blocked.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST use **Better Auth** framework for handling authentication logic.
- **FR-002**: System MUST persist user data in **Neon Serverless PostgreSQL**.
- **FR-003**: System MUST support user registration with Email, Username, and Password.
- **FR-004**: System MUST support OAuth authentication via **Google Sign-In**.
- **FR-005**: System MUST securely hash passwords before storage (never store plain text).
- **FR-006**: System MUST enforce uniqueness for 'email' and 'username' fields in the database.
- **FR-007**: System MUST provide API endpoints for Sign-up, Sign-in, Logout, and Session validation.
- **FR-008**: System MUST gate access to Chatbot features, requiring a valid authenticated session.
- **FR-009**: System MUST allow public access to Book content (documentation/reading material) without authentication.
- **FR-010**: System MUST handle OAuth account linking where the email matches an existing account, merging the Google profile with the existing user record.
- **FR-011**: System MUST use environment variables for sensitive configuration (DB URL, API keys, Secrets).
- **FR-012**: System MUST send an email verification link to users upon successful registration.
- **FR-013**: System MUST enforce a single active session per user. New logins invalidate existing sessions.
- **FR-014**: System MUST manage user sessions with a 30-day absolute and 7-day rolling expiry.
- **FR-015**: System MUST NOT implement multi-factor authentication (MFA) in this iteration.

### Key Entities

- **User**: Represents a registered account.
  - `id`: Unique Identifier
  - `username`: Unique display name
  - `email`: Unique contact/login email
  - `password_hash`: Securely hashed credential (nullable for OAuth users)
  - `provider`: Auth method (email/google)
  - `created_at`, `updated_at`: Timestamps

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can successfully register and login via Email/Password in under 1 minute.
- **SC-002**: Users can successfully login via Google in under 30 seconds.
- **SC-003**: 100% of chatbot access attempts by unauthenticated users are blocked/redirected.
- **SC-004**: 100% of book content access attempts by unauthenticated users are successful.
- **SC-005**: All user passwords are stored as hashes in the Neon database.

## Clarifications

### Session 2025-12-19

- Q: Should email verification be enforced? → A: Optional/Background: Users can access the chatbot immediately; verification email is sent but not enforced for access.
- Q: How many concurrent sessions are allowed per user? → A: Single Session per User: A user can only have one active session at a time; new logins invalidate old sessions.
- Q: What is the desired session expiry behavior? → A: Long-lived Session: Session expires after 30 days (absolute) or 7 days of inactivity (rolling).
- Q: What is the desired behavior for OAuth account linking when an email collision occurs? → A: Link Accounts: If a user attempts to sign in with Google using an email that already exists (e.g., from email/password signup), the Google account should be linked to the existing user profile.
- Q: What type of additional authentication factors (e.g., 2FA) should be implemented? → A: No additional authentication factors: Only password or social login is required.
