# Implementation Plan: OpenAI RAG Agent

**Branch**: `012-openai-rag-agent` | **Date**: 2025-12-17 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/012-openai-rag-agent/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement an agentic RAG chatbot in `backend/agent.py` using the `openai-agents` SDK. The agent will have a custom tool `search_knowledge_base` that wraps the existing retrieval logic from `retrieve.py` to query Qdrant. The agent will run in a CLI loop, answering user questions using the retrieved context.

## Technical Context

**Language/Version**: Python 3.13
**Primary Dependencies**: `openai-agents`, `qdrant-client`, `cohere`, `python-dotenv`
**Storage**: Qdrant Vector Database (Read via `retrieve.py`)
**Testing**: Manual verification via CLI interaction
**Target Platform**: Python environment (backend)
**Project Type**: Single script extension
**Performance Goals**: Agent response within 10 seconds (including retrieval)
**Constraints**: Must use `openai-agents` SDK and existing `retrieve.py` logic
**Scale/Scope**: Single Agent, One Tool

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [x] **Project Directory**: `backend/agent.py`.
- [x] **Python-First**: Yes.
- [x] **Tech Stack**: `openai-agents` (new approved dependency), Qdrant.
- [x] **No Hallucinations**: SDK usage verified via docs.
- [x] **RAG-Centric**: Core functionality is RAG.

## Project Structure

### Documentation (this feature)

```text
specs/012-openai-rag-agent/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # N/A
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── agent.py             # NEW: Agent implementation
├── retrieve.py          # Existing: Retrieval logic (imported by agent.py)
├── main.py              # Existing: Ingestion pipeline
└── pyproject.toml       # Dependencies
```

**Structure Decision**: A separate `agent.py` keeps the agent orchestration logic distinct from the retrieval mechanism (`retrieve.py`) and ingestion (`main.py`), promoting modularity.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| (None) | | |