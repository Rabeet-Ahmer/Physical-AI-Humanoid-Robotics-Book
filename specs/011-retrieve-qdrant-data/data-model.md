# Data Model: Retrieve Qdrant Data

**Feature**: `011-retrieve-qdrant-data`

## Qdrant Payload Structure

The `retrieve.py` script expects the following payload structure in the Qdrant collection (as populated by `main.py`):

| Field | Type | Description |
|-------|------|-------------|
| `text` | string | The text chunk content. |
| `source_url` | string | The URL where the text was extracted from. |
| `chunk_index` | integer | The index of the chunk within the source document. |

## CLI Interface

The `retrieve.py` script will be executed from the command line.

**Usage:**
```bash
python retrieve.py "<query_text>"
```

**Arguments:**
- `query_text`: The semantic search query string.

**Output:**
- List of matching results, each showing:
  - Similarity Score
  - Source URL
  - Text Content
