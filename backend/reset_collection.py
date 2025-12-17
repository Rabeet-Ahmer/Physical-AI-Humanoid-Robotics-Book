import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient

load_dotenv()

qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")
collection_name = "rag_embedding" # Hardcoded to match main.py reference

if all([qdrant_url, qdrant_api_key]):
    client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
    if client.collection_exists(collection_name):
        print(f"Deleting collection {collection_name}...")
        client.delete_collection(collection_name)
        print("Deleted.")
    else:
        print(f"Collection {collection_name} does not exist.")
else:
    print("Missing environment variables.")