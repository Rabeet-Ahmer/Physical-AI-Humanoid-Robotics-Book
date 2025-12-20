# API Contracts

No new backend APIs are required. This feature relies on the existing Auth Client.

## Client Interfaces

### Auth Client (`src/lib/auth-client.ts`)
- `useSession()`: Returns `{ data: Session, isPending: boolean, error: Error }`
