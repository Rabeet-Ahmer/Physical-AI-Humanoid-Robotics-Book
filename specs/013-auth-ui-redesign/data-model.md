# Data Model: Auth UI Redesign

## Entities

### User (Better Auth Schema)
*Note: This schema is managed by the `auth-server` and `better-auth` library.*

| Field | Type | Description |
|-------|------|-------------|
| `id` | String | Unique identifier (UUID) |
| `name` | String | User's display name |
| `email` | String | User's email address (Unique) |
| `emailVerified` | Boolean | Whether email is verified |
| `image` | String | URL to profile image |
| `createdAt` | DateTime | Timestamp of creation |
| `updatedAt` | DateTime | Timestamp of last update |

### Account (Social Login)
*Links external providers to User.*

| Field | Type | Description |
|-------|------|-------------|
| `userId` | String | Reference to User |
| `providerId` | String | e.g., "google" |
| `accountId` | String | Provider's unique user ID |

## UI State Models

### SignInState
- `email`: string
- `password`: string
- `loading`: boolean
- `error`: string | null

### SignUpState
- `name`: string
- `email`: string
- `password`: string
- `loading`: boolean
- `error`: string | null
