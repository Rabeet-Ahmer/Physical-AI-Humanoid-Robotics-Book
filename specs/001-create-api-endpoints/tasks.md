# Tasks: Create API Endpoints for Frontend

**Feature**: Create API Endpoints for Frontend
**Status**: Pending
**Branch**: `001-create-api-endpoints`

## Dependencies

- **US1** depends on **Phase 1** & **Phase 2** completion.
- Refactoring `agent.py` (T003) is a blocker for `api.py` implementation (T005).

## Phase 1: Setup

**Goal**: Initialize the environment and dependencies for the new API service.

- [X] T001 Install FastAPI dependencies using `uv add "fastapi[standard]"` in `backend/`
- [X] T002 Create initial empty `backend/api.py` file to establish the entry point

## Phase 2: Foundational

**Goal**: Prepare the existing codebase for integration (Refactoring).

- [X] T003 Refactor `backend/agent.py` to expose a reusable `create_agent()` function or `agent` instance, ensuring the CLI loop remains functional under `if __name__ == "__main__":`

## Phase 3: User Story 1 - Expose Agent Functionality to Frontend

**Goal**: Implement the API endpoint to allow frontend interaction with the backend agent.
**Story**: [US1] Expose Agent Functionality to Frontend (Priority: P1)

**Independent Test**:
1. Start API: `uv run fastapi dev backend/api.py`
2. Send Request: `curl -X POST "http://127.0.0.1:8000/agent" -H "Content-Type: application/json" -d '{"user": "Test query"}'`
3. Verify Response: JSON object `{"assistant": "..."}`
4. Verify CLI still works: `uv run backend/agent.py`

**Tasks**:

- [X] T004 [US1] Define Pydantic models `UserQuery` and `AgentResponse` in `backend/api.py` matching the Data Model
- [X] T005 [US1] Implement `POST /agent` endpoint in `backend/api.py` that imports `agent` from `backend/agent.py`, uses `Runner.run()`, and returns the formatted response
- [X] T006 [US1] Implement error handling in `backend/api.py` for internal errors (HTTP 500) and timeouts (HTTP 504) as per spec Edge Cases

## Phase 4: Polish & Cross-Cutting Concerns

**Goal**: Ensure production readiness and compliance.

- [X] T007 Configure CORS in `backend/api.py` to allow requests from the frontend origin (default to `*` for development if unspecified)
- [X] T008 Verify `backend/api.py` implementation matches `specs/001-create-api-endpoints/contracts/openapi.yaml` structure
- [X] T009 Manual verification: Run the `curl` test from Quickstart and confirm 500ms latency target is locally reasonable

## Implementation Strategy

1. **Refactor First**: We must liberate the agent logic from the CLI loop in `agent.py` before we can import it.
2. **MVP API**: Build the endpoint with minimal error handling first to prove the connection.
3. **Hardening**: Add error handling and CORS once the happy path works.
