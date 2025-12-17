# Research: OpenAI RAG Agent

**Feature**: `012-openai-rag-agent`
**Status**: Complete

## Key Decisions

### 1. SDK Selection
- **Decision**: Use `openai-agents` Python SDK.
- **Rationale**: User explicitly requested "OpenAI Agents SDK". Research confirms `openai-agents` is the package name for the official (experimental) SDK that supports multi-agent workflows and function calling patterns.
- **Dependency**: `openai-agents`.

### 2. Tool Integration
- **Decision**: Import `embed_query` and `search_qdrant` from the existing `retrieve.py` module.
- **Rationale**: `retrieve.py` already contains the verified logic for embedding with Cohere and searching Qdrant. Wrapping these into a single `@function_tool` decorated function avoids code duplication.
- **Implementation**:
  ```python
  from agents import function_tool
  from retrieve import embed_query, search_qdrant, ...

  @function_tool
  def search_knowledge_base(query: str) -> str:
      """Searches the internal knowledge base for relevant information."""
      # Logic to call embed and search, then format results
      ...
  ```

### 3. Agent Architecture
- **Decision**: Single Agent using `Runner.run()`.
- **Rationale**: The requirement is "simple for now". A single agent with one tool is the simplest implementation of the SDK.
- **Interface**: A simple CLI loop in `agent.py` that accepts user input and prints the agent's response.

## Open Questions

- **Module Name**: Does `pip install openai-agents` expose the module as `agents` or `openai_agents`? Documentation snippets suggest `from agents import ...`. I will assume `agents` but verify during implementation.

