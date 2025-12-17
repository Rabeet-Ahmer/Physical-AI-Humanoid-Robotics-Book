# API Contract: Embedding Pipeline

## Overview
This contract defines the interface for the EmbeddingPipeline class that processes content from sitemaps and stores vector embeddings in Qdrant.

## Class Definition

### EmbeddingPipeline

#### Constructor
```python
def __init__(
    self,
    cohere_api_key: str,
    qdrant_url: str = "http://localhost:6333",
    collection_name: str,
    embedding_model: str = "embed-multilingual-v3.0",
    chunk_size: int = 1000,
    chunk_overlap: int = 200
)
```

**Parameters:**
- `cohere_api_key` (str): API key for Cohere embedding service
- `qdrant_url` (str): URL to Qdrant instance (default: "http://localhost:6333")
- `collection_name` (str): Name of the Qdrant collection to create/use
- `embedding_model` (str): Cohere model to use for embeddings (default: "embed-multilingual-v3.0")
- `chunk_size` (int): Size of text chunks in characters (default: 1000)
- `chunk_overlap` (int): Overlap between chunks in characters (default: 200)

#### Methods

##### get_all_url
```python
def get_all_url(self, sitemap_url: str) -> List[str]
```
**Description:** Parse sitemap.xml and return all URLs found within it
**Parameters:**
- `sitemap_url` (str): URL to the sitemap.xml file
**Returns:** List of URLs extracted from the sitemap
**Errors:** Raises exceptions for network issues or invalid sitemap format

##### extract_text_from_url
```python
def extract_text_from_url(self, url: str) -> str
```
**Description:** Extract clean text content from a given URL
**Parameters:**
- `url` (str): URL to extract text from
**Returns:** Clean text content from the webpage
**Errors:** Raises exceptions for network issues or content extraction problems

##### chunk_text
```python
def chunk_text(self, text: str, chunk_size: int = None, chunk_overlap: int = None) -> List[str]
```
**Description:** Split text into smaller chunks for embedding
**Parameters:**
- `text` (str): Text to be chunked
- `chunk_size` (int): Size of chunks (uses instance default if None)
- `chunk_overlap` (int): Overlap between chunks (uses instance default if None)
**Returns:** List of text chunks
**Errors:** None

##### embed
```python
def embed(self, texts: List[str]) -> List[List[float]]
```
**Description:** Generate vector embeddings for a list of texts
**Parameters:**
- `texts` (List[str]): List of text strings to embed
**Returns:** List of embedding vectors (each vector is a list of floats)
**Errors:** Raises exceptions for API issues or invalid input

##### create_collection
```python
def create_collection(self) -> None
```
**Description:** Create a collection in Qdrant for storing embeddings
**Parameters:** None
**Returns:** None
**Errors:** Raises exceptions for Qdrant connection or creation issues

##### save_to_qdrant
```python
def save_to_qdrant(self, texts: List[str], embeddings: List[List[float]], metadata: List[dict] = None) -> None
```
**Description:** Save text embeddings to Qdrant collection with optional metadata
**Parameters:**
- `texts` (List[str]): Original text chunks
- `embeddings` (List[List[float]]): Corresponding embedding vectors
- `metadata` (List[dict]): Optional metadata for each text chunk
**Returns:** None
**Errors:** Raises exceptions for Qdrant storage issues

##### run
```python
def run(self, sitemap_url: str) -> None
```
**Description:** Execute the complete pipeline from sitemap to Qdrant storage
**Parameters:**
- `sitemap_url` (str): URL to the sitemap.xml file
**Returns:** None
**Errors:** Raises exceptions for any stage of the pipeline