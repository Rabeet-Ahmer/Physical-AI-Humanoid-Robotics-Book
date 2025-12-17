import os
import sys
import argparse
import logging
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from cohere import Client

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def embed_query(cohere_client: Client, query: str) -> list[float]:
    """
    Generates an embedding for the search query using Cohere.
    """
    try:
        response = cohere_client.embed(
            texts=[query],
            model="embed-multilingual-v3.0",  # Matches main.py reference
            input_type="search_query"
        )
        return response.embeddings[0]
    except Exception as e:
        logging.error(f"Error generating embedding: {e}")
        return []

def search_qdrant(qdrant_client: QdrantClient, collection_name: str, query_vector: list[float], limit: int = 5):
    """
    Searches the Qdrant collection with the query vector.
    """
    try:
        # Using query_points for qdrant-client compatibility
        results = qdrant_client.query_points(
            collection_name=collection_name,
            query=query_vector,
            limit=limit
        ).points
        return results
    except Exception as e:
        logging.error(f"Error searching Qdrant: {e}")
        return []

def main():
    parser = argparse.ArgumentParser(description="Retrieve relevant information from Qdrant based on a text query.")
    parser.add_argument("query", type=str, help="The search query text.")
    args = parser.parse_args()

    query_text = args.query
    if not query_text:
        logging.error("Query text cannot be empty.")
        return

    # Retrieve environment variables
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    cohere_api_key = os.getenv("COHERE_API_KEY")
    # Default to "rag_embedding" as per main.py reference, unless overridden
    collection_name = os.getenv("QDRANT_COLLECTION_NAME", "rag_embedding")

    # Validate environment variables
    if not all([qdrant_url, qdrant_api_key, cohere_api_key]):
        logging.error("Missing required environment variables. Please check your .env file.")
        return

    # Initialize clients
    try:
        qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        cohere_client = Client(cohere_api_key)
    except Exception as e:
        logging.error(f"Failed to initialize clients: {e}")
        return

    logging.info(f"Processing query: '{query_text}'")

    # Generate embedding
    query_vector = embed_query(cohere_client, query_text)
    if not query_vector:
        return

    # Search Qdrant
    results = search_qdrant(qdrant_client, collection_name, query_vector)
    
    if not results:
        print("\nNo results found.")
        return

    # Display results
    print(f"\nSearch Results for: \"{query_text}\"")
    print("-" * 50)
    for result in results:
        score = result.score
        payload = result.payload
        # Payload keys match main.py reference
        content = payload.get("content", "N/A")
        url = payload.get("url", "N/A")
        
        print(f"Score: {score:.4f}")
        print(f"Source: {url}")
        print(f"Content: {content[:200]}..." if len(content) > 200 else f"Content: {content}") # Truncate long text
        print("-" * 50)

if __name__ == "__main__":
    main()