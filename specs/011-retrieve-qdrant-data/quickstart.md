# Quickstart: Retrieve Qdrant Data

**Feature**: `011-retrieve-qdrant-data`

## Prerequisites

- **Environment**: Ensure `.env` file in `backend/` contains:
  - `QDRANT_HOST` (or `QDRANT_URL`)
  - `QDRANT_API_KEY`
  - `COHERE_API_KEY`
  - `QDRANT_COLLECTION_NAME`
- **Dependencies**: `uv` installed.

## Running the Retrieval Script

1.  Navigate to the `backend` directory:
    ```bash
    cd backend
    ```

2.  Run the script using `uv`:
    ```bash
    uv run retrieve.py "What is the main topic of the website?"
    ```

3.  **Expected Output**:
    ```text
    Search Results for: "What is the main topic of the website?"
    --------------------------------------------------
    Score: 0.85
    Source: https://example.com/page1
    Text: ... content of the chunk ...
    --------------------------------------------------
    ...
    ```
