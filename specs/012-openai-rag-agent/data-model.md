# Data Model: OpenAI RAG Agent

**Feature**: `012-openai-rag-agent`

## Agent Configuration

The agent will be configured with the following properties:

| Property | Value | Description |
|----------|-------|-------------|
| **Name** | `RAG Assistant` | The display name of the agent. |
| **Model** | `gpt-4o` | The OpenAI model to use (requires `OPENAI_API_KEY`). |
| **Instructions** | *"You are a helpful assistant. Use the search_knowledge_base tool to answer questions about the website."* | System prompt. |
| **Tools** | `[search_knowledge_base]` | The Qdrant retrieval tool. |

## Tool Interface

### `search_knowledge_base`

- **Input**: `query` (string) - The user's question or search term.
- **Output**: `string` - A formatted string containing the top retrieval results (Source URL + Content text) or "No results found".

## CLI Interface

The `agent.py` script will run an interactive loop.

**Usage:**
```bash
uv run agent.py
```

**Interaction:**
```text
User: What is this website about?
Agent: This website is about... [Answer based on retrieved content]
```
