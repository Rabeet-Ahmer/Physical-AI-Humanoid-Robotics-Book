# Quickstart: Qdrant Collection Pipeline

## Prerequisites

- Python 3.9+
- UV package manager
- Cohere API key
- Qdrant instance (local or remote)

## Setup

1. **Install dependencies**:
   ```bash
   uv venv  # Create virtual environment (optional but recommended)
   uv pip install qdrant-client cohere requests beautifulsoup4 langchain-community
   ```

2. **Set up Cohere API key**:
   ```bash
   export COHERE_API_KEY="your-cohere-api-key"
   ```

3. **Prepare Qdrant**:
   - For local development: `docker run -p 6333:6333 qdrant/qdrant`
   - Or use a remote Qdrant instance

## Basic Usage

### Running the Pipeline

```python
from main import EmbeddingPipeline

# Initialize the pipeline
pipeline = EmbeddingPipeline(
    cohere_api_key="your-cohere-api-key",
    qdrant_url="http://localhost:6333",  # or your Qdrant URL
    collection_name="my_collection"
)

# Run the full pipeline
pipeline.run(sitemap_url="https://example.com/sitemap.xml")
```

### Using Individual Components

```python
from main import EmbeddingPipeline

pipeline = EmbeddingPipeline(
    cohere_api_key="your-cohere-api-key",
    qdrant_url="http://localhost:6333",
    collection_name="my_collection"
)

# Step 1: Get URLs from sitemap
urls = pipeline.get_all_url("https://example.com/sitemap.xml")
print(f"Found {len(urls)} URLs")

# Step 2: Extract text from URLs
all_texts = []
for url in urls:
    text = pipeline.extract_text_from_url(url)
    all_texts.append({"url": url, "text": text})

# Step 3: Chunk the text
all_chunks = []
for item in all_texts:
    chunks = pipeline.chunk_text(item["text"])
    for i, chunk in enumerate(chunks):
        all_chunks.append({
            "content": chunk,
            "source_url": item["url"],
            "position": i
        })

# Step 4: Generate embeddings
texts = [chunk["content"] for chunk in all_chunks]
embeddings = pipeline.embed(texts)

# Step 5: Create collection and save to Qdrant
pipeline.create_collection()
metadata = [{"source_url": chunk["source_url"], "position": chunk["position"]} for chunk in all_chunks]
pipeline.save_to_qdrant(texts, embeddings, metadata)
```

## Configuration

### Environment Variables
- `COHERE_API_KEY`: Your Cohere API key
- `QDRANT_URL`: URL to your Qdrant instance (default: http://localhost:6333)
- `QDRANT_API_KEY`: API key if your Qdrant instance requires authentication

### Parameters
- `chunk_size`: Size of text chunks (default: 1000 characters)
- `chunk_overlap`: Overlap between chunks (default: 200 characters)
- `embedding_model`: Cohere model to use (default: embed-multilingual-v3.0)

## Example

```python
# Complete example
from main import EmbeddingPipeline

def main():
    pipeline = EmbeddingPipeline(
        cohere_api_key="your-cohere-api-key",
        qdrant_url="http://localhost:6333",
        collection_name="example_docs"
    )

    # Process a sitemap
    sitemap_url = "https://example.com/sitemap.xml"
    pipeline.run(sitemap_url)

    print("Pipeline completed successfully!")

if __name__ == "__main__":
    main()
```

## Troubleshooting

- **Sitemap parsing fails**: Verify the sitemap URL is accessible and properly formatted
- **Text extraction issues**: Some websites may block automated requests; check robots.txt
- **Embedding API errors**: Verify your Cohere API key and check rate limits
- **Qdrant connection issues**: Ensure Qdrant is running and accessible at the specified URL