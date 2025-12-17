# Implementation Plan: Create API Endpoints for Frontend

**Branch**: `001-create-api-endpoints` | **Date**: 2025-12-17 | **Spec**: [specs/001-create-api-endpoints/spec.md](../001-create-api-endpoints/spec.md)
**Input**: Feature specification from `specs/001-create-api-endpoints/spec.md`

## Summary

The goal is to expose the backend AI agent functionality to the frontend via a new FastAPI service (`api.py`). This involves creating a `POST /agent` endpoint that accepts a user query and returns the agent's response, adhering to the "FastAPI[standards]" stack. To achieve this, the existing `agent.py` script will be refactored to make the `Agent` instance reusable/importable without triggering its CLI loop.

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**: `fastapi[standards]`, `uv` (package manager), `openai-agents` (implied by agent.py)
**Storage**: N/A for this specific feature (Agent interacts with existing Qdrant/RAG, but API doesn't own storage)
**Testing**: `pytest`
**Target Platform**: Linux server
**Project Type**: Web application (Backend Service)
**Performance Goals**: < 500ms p95 latency for API overhead (Agent processing time depends on LLM)
**Constraints**: 
- Strict adherence to `fastapi[standards]`
- No hallucinations (verify via Context7)
- Must refactor `agent.py` cleanly
- JSON Schema: `{"user": "..."}` -> `{"assistant": "..."}`

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [x] **Project Directory**: `backend/` used.
- [x] **Python-First**: Yes, Python 3.10+.
- [x] **Tech Stack**: `uv`, `FastAPI[standards]`, `OpenAI Agents SDK`.
- [x] **No Hallucinations**: Research done.
- [x] **Context7 Documentation**: Used for OpenAI Agents SDK & FastAPI integration patterns.
- [x] **RAG-Centric**: Exposing RAG agent.
- [x] **API Standards**: RESTful `POST /agent`, JSON responses, OpenAPI contract defined.

## Project Structure

### Documentation (this feature)

```text
specs/001-create-api-endpoints/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
│   └── openapi.yaml
└── tasks.md             # Phase 2 output (to be created)
```

### Source Code (repository root)

```text
backend/
├── api.py               # NEW: FastAPI application entry point
├── agent.py             # MODIFIED: Refactored to expose create_agent()
├── main.py              # EXISTING: RAG ingestion pipeline
├── retrieve.py          # EXISTING: Retrieval logic
└── pyproject.toml       # EXISTING: Dependencies
```

**Structure Decision**: Option 2 (Web application - Backend focus). We are adding `api.py` as a sibling to existing scripts in `backend/` to keep imports simple (`from agent import create_agent`).
