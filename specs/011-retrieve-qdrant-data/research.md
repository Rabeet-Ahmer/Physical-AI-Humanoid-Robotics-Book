# Research: Retrieve Qdrant Data

**Feature**: `011-retrieve-qdrant-data`
**Status**: Complete

## Key Decisions

### 1. Retrieval Method
- **Decision**: Use `qdrant_client.search()` for semantic search.
- **Rationale**: This is the standard method for finding nearest neighbors in Qdrant based on vector similarity. It supports filtering and payload retrieval, which meets our requirements.
- **Alternatives**: Scroll API (not suitable for ranked search), Point retrieval by ID (we don't have IDs, we have a query).

### 2. Embedding Consistency
- **Decision**: Use Cohere `embed-english-v3.0` with `input_type="search_query"`.
- **Rationale**: The ingestion pipeline (`main.py`) used `embed-english-v3.0` with `input_type="search_document"`. For retrieval to work effectively, the query must be embedded using the same model, but with the `search_query` input type as recommended by Cohere for RAG applications.
- **Implication**: We must reuse the `COHERE_API_KEY` from the environment.

### 3. Output Formatting
- **Decision**: Display `score`, `text`, and `source_url` for each result.
- **Rationale**: These fields provide the user with the relevance (score), the actual content (text), and the provenance (url) of the retrieved information.

## Open Questions

- None. The implementation follows the pattern established in `main.py`.
