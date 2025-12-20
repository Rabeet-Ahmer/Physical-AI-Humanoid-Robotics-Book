# Data Model: Docs Access Control

## UI State

### AuthState
- **session**: UserSession | null
- **isPending**: boolean

### RouteState
- **currentPath**: string
- **isProtected**: boolean (derived: `currentPath.startsWith('/docs')`)
