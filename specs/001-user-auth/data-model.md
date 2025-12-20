# Data Model: Add User Authentication

**Feature**: `001-user-auth`

## Entities

### User (`user`)
*Managed by Better Auth*
- `id`: Text (Primary Key)
- `name`: Text
- `email`: Text (Unique)
- `emailVerified`: Boolean
- `image`: Text (URL)
- `createdAt`: Timestamp
- `updatedAt`: Timestamp

### Session (`session`)
*Managed by Better Auth*
- `id`: Text (Primary Key)
- `expiresAt`: Timestamp
- `token`: Text (Unique)
- `createdAt`: Timestamp
- `updatedAt`: Timestamp
- `ipAddress`: Text (Optional)
- `userAgent`: Text (Optional)
- `userId`: Text (Foreign Key -> user.id)

### Account (`account`)
*Managed by Better Auth (for OAuth)*
- `id`: Text (Primary Key)
- `accountId`: Text
- `providerId`: Text
- `userId`: Text (Foreign Key -> user.id)
- `accessToken`: Text (Optional)
- `refreshToken`: Text (Optional)
- `idToken`: Text (Optional)
- `accessTokenExpiresAt`: Timestamp (Optional)
- `refreshTokenExpiresAt`: Timestamp (Optional)
- `scope`: Text (Optional)
- `password`: Text (Optional - for email/password if not using separate table, Better Auth config dependent)
- `createdAt`: Timestamp
- `updatedAt`: Timestamp

### Verification (`verification`)
*Managed by Better Auth (for Email verification)*
- `id`: Text (Primary Key)
- `identifier`: Text
- `value`: Text
- `expiresAt`: Timestamp
- `createdAt`: Timestamp
- `updatedAt`: Timestamp

## Relationships

- **User -> Session**: One-to-Many
- **User -> Account**: One-to-Many

## Schema SQL (Reference)

```sql
-- Better Auth will generate this, but for reference:
CREATE TABLE "user" (
    id text NOT NULL PRIMARY KEY,
    name text NOT NULL,
    email text NOT NULL UNIQUE,
    "emailVerified" boolean NOT NULL,
    image text,
    "createdAt" timestamp NOT NULL,
    "updatedAt" timestamp NOT NULL
);

CREATE TABLE "session" (
    id text NOT NULL PRIMARY KEY,
    "expiresAt" timestamp NOT NULL,
    token text NOT NULL UNIQUE,
    "createdAt" timestamp NOT NULL,
    "updatedAt" timestamp NOT NULL,
    "ipAddress" text,
    "userAgent" text,
    "userId" text NOT NULL REFERENCES "user"(id)
);

CREATE TABLE "account" (
    id text NOT NULL PRIMARY KEY,
    "accountId" text NOT NULL,
    "providerId" text NOT NULL,
    "userId" text NOT NULL REFERENCES "user"(id),
    "accessToken" text,
    "refreshToken" text,
    "idToken" text,
    "expiresAt" timestamp,
    "password" text
);

CREATE TABLE "verification" (
    id text NOT NULL PRIMARY KEY,
    identifier text NOT NULL,
    value text NOT NULL,
    "expiresAt" timestamp NOT NULL,
    "createdAt" timestamp,
    "updatedAt" timestamp
);
```
