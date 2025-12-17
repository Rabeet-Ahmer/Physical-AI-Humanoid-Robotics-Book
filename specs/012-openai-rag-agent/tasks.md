# Tasks: OpenAI RAG Agent

**Feature**: `012-openai-rag-agent`
**Spec**: [spec.md](spec.md)
**Plan**: [plan.md](plan.md)

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Install required SDK and verify environment.

- [x] T001 Install `openai-agents` package using `uv add openai-agents` in `backend/` directory
- [x] T002 Verify `.env` file in `backend/` contains `OPENAI_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY`, and `COHERE_API_KEY`

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Ensure existing code is ready for integration.

- [x] T003 Verify `backend/retrieve.py` functions (`embed_query`, `search_qdrant`) are importable (ensure `if __name__ == "__main__":` block exists)

## Phase 3: User Story 1 - RAG Chatbot (Priority: P1)

**Goal**: Implement the agent that uses Qdrant as a tool to answer questions.
**Independent Test**: Run `uv run agent.py` and ask "What is the main topic of the website?".

- [x] T004 [US1] Create `backend/agent.py` with necessary imports from `agents` SDK, `dotenv`, and `retrieve` module
- [x] T005 [US1] Implement `search_knowledge_base` function in `backend/agent.py` decorated with `@function_tool` that invokes `retrieve.embed_query` and `retrieve.search_qdrant` and formats results
- [x] T006 [US1] Configure the Agent in `backend/agent.py` with name "RAG Assistant", model `gemini-2.0-flash`, instructions, and the retrieval tool
- [x] T007 [US1] Implement the asynchronous main loop in `backend/agent.py` using `Runner.run()` to process user input from stdin and print agent responses

## Final Phase: Polish & Cross-Cutting Concerns

**Purpose**: Verification and robustness.

- [x] T008 Add error handling in `backend/agent.py` for API failures or missing environment variables
- [x] T009 Validate the agent by running a conversation and verifying it retrieves correct context from Qdrant

## Dependencies & Execution Order

1.  **Phase 1**: Installation.
2.  **Phase 3**: Core Agent implementation.
3.  **Final Phase**: Validation.

## Implementation Strategy

1.  **Setup**: Install SDK.
2.  **Tooling**: Wrap the retrieval logic first.
3.  **Agent**: Connect the tool to the Agent and build the loop.
4.  **Verify**: Test the interaction.
