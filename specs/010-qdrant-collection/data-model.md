# Data Model: Qdrant Collection Pipeline

## Entities

### EmbeddingPipeline
- **Purpose**: Orchestrates the entire process from sitemap parsing to Qdrant storage
- **Attributes**:
  - `cohere_api_key` (str): API key for Cohere embedding service
  - `qdrant_client` (QdrantClient): Client for Qdrant vector database
  - `collection_name` (str): Name of the Qdrant collection
- **Methods**:
  - `get_all_url(sitemap_url: str) -> List[str]`: Parse sitemap and return all URLs
  - `extract_text_from_url(url: str) -> str`: Extract clean text from a single URL
  - `chunk_text(text: str, chunk_size: int = 1000, overlap: int = 200) -> List[str]`: Split text into chunks
  - `embed(texts: List[str]) -> List[List[float]]`: Generate embeddings for text chunks
  - `create_collection() -> None`: Create Qdrant collection for storing embeddings
  - `save_to_qdrant(texts: List[str], embeddings: List[List[float]], metadata: List[dict]) -> None`: Store embeddings in Qdrant

### Text Chunk
- **Purpose**: A segment of text content that has been processed and prepared for embedding
- **Attributes**:
  - `id` (str): Unique identifier for the chunk
  - `content` (str): The actual text content
  - `source_url` (str): URL where the text originated
  - `position` (int): Position of the chunk in the original document
  - `metadata` (dict): Additional information about the chunk

### Vector Embedding
- **Purpose**: Numerical representation of text content in vector space
- **Attributes**:
  - `id` (str): Unique identifier for the embedding
  - `vector` (List[float]): The actual embedding vector (e.g., 1024-dimensional for Cohere)
  - `text_chunk_id` (str): Reference to the source text chunk
  - `metadata` (dict): Associated metadata (source URL, position, etc.)

### Qdrant Collection
- **Purpose**: A container in Qdrant for storing vector embeddings with metadata
- **Attributes**:
  - `name` (str): The collection name
  - `vector_size` (int): Dimension of the vectors (e.g., 1024 for Cohere embeddings)
  - `distance_metric` (str): Distance metric for similarity search (e.g., cosine)
  - `points_count` (int): Number of vectors stored in the collection

## Relationships

```
EmbeddingPipeline 1 -- * Text Chunk
Text Chunk 1 -- 1 Vector Embedding
Vector Embedding 1 -- * Qdrant Collection
```

## Data Flow

1. `EmbeddingPipeline.get_all_url()` reads sitemap.xml and produces a list of URLs
2. `EmbeddingPipeline.extract_text_from_url()` processes each URL to extract clean text
3. `EmbeddingPipeline.chunk_text()` splits the text into `Text Chunk` entities
4. `EmbeddingPipeline.embed()` converts `Text Chunk` content to `Vector Embedding` entities
5. `EmbeddingPipeline.create_collection()` prepares the `Qdrant Collection`
6. `EmbeddingPipeline.save_to_qdrant()` stores `Vector Embedding` entities in the `Qdrant Collection`

## Validation Rules

- Each URL in sitemap must be a valid HTTP/HTTPS URL
- Text chunks must not exceed the embedding model's token limit
- Each text chunk must have a corresponding source URL in metadata
- Vector embeddings must have the correct dimensionality for the chosen model
- Collection must be created before attempting to store embeddings

## State Transitions

- `EmbeddingPipeline` starts in `initialized` state
- After `get_all_url()` completes, moves to `urls_fetched` state
- After `extract_text_from_url()` completes for all URLs, moves to `text_extracted` state
- After `chunk_text()` completes, moves to `text_chunked` state
- After `embed()` completes, moves to `embedded` state
- After `create_collection()` completes, moves to `collection_created` state
- After `save_to_qdrant()` completes, moves to `completed` state