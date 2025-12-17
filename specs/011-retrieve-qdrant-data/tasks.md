# Tasks: Retrieve Qdrant Data

**Feature**: `011-retrieve-qdrant-data`
**Spec**: [spec.md](spec.md)
**Plan**: [plan.md](plan.md)

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Ensure environment and dependencies are ready.

- [x] T001 Verify `.env` file in `backend/` contains `QDRANT_URL`, `QDRANT_API_KEY`, `COHERE_API_KEY`, and `QDRANT_COLLECTION_NAME`
- [x] T002 Verify `backend/pyproject.toml` contains `qdrant-client`, `cohere`, and `python-dotenv` dependencies

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Establish the file structure.

- [x] T003 Create `backend/retrieve.py` file with basic imports (`os`, `sys`, `qdrant_client`, `cohere`, `dotenv`) and logging setup

## Phase 3: User Story 1 - Semantic Search (Priority: P1)

**Goal**: Implement the full retrieval pipeline: query input -> embedding -> search -> output.
**Independent Test**: Run `uv run retrieve.py "test query"` and verify output.

- [x] T004 [US1] Implement argument parsing in `backend/retrieve.py` to accept the search query as a command-line argument
- [x] T005 [US1] Implement initialization of `QdrantClient` and `cohere.Client` in `backend/retrieve.py` using environment variables
- [x] T006 [US1] Implement `embed_query` function in `backend/retrieve.py` to generate embeddings using Cohere (`model="embed-english-v3.0"`, `input_type="search_query"`)
- [x] T007 [US1] Implement `search_qdrant` function in `backend/retrieve.py` to query the Qdrant collection with the generated embedding
- [x] T008 [US1] Implement `main` function in `backend/retrieve.py` to orchestrate the flow and print formatted results (Score, Source URL, Text)

## Final Phase: Polish & Cross-Cutting Concerns

**Purpose**: Robustness and final verification.

- [x] T009 Add error handling in `backend/retrieve.py` for missing environment variables, network errors, and empty search results
- [x] T010 Validate the implementation by running a real search query against the Qdrant database populated by `main.py`

## Dependencies & Execution Order

1.  **Phase 1**: Environment check (Start here).
2.  **Phase 2**: File creation (Depends on Phase 1).
3.  **Phase 3**: Core logic (Depends on Phase 2). Tasks T004-T008 should be implemented sequentially or logic grouped into the main function.
4.  **Final Phase**: Polish (Depends on Phase 3).

## Implementation Strategy

1.  **Setup**: Check environment.
2.  **Core Implementation**: Write the complete `retrieve.py` script (T003-T008) in one go or incrementally.
3.  **Verify**: Run the script with a known query.
