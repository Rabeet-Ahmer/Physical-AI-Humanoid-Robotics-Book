# Quickstart: OpenAI RAG Agent

**Feature**: `012-openai-rag-agent`

## Prerequisites

- **Environment**: Ensure `.env` file in `backend/` contains:
  - `OPENAI_API_KEY` (New requirement)
  - `QDRANT_URL`, `QDRANT_API_KEY`, `COHERE_API_KEY` (Existing)
- **Dependencies**: `openai-agents` installed via `uv`.

## Running the Agent

1.  Navigate to the `backend` directory:
    ```bash
    cd backend
    ```

2.  Run the agent script:
    ```bash
    uv run agent.py
    ```

3.  **Interaction Example**:
    ```text
    > What is the main topic of the website?
    [Agent searches Qdrant...]
    The main topic of the website is Physical AI and Humanoid Robotics...
    ```
