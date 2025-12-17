import os
import asyncio
import logging
from dotenv import load_dotenv, find_dotenv
from openai import AsyncOpenAI
from agents import Agent, function_tool, RunConfig, Runner, OpenAIChatCompletionsModel, ModelSettings
from qdrant_client import QdrantClient
from cohere import Client as CohereClient
import retrieve # Import retrieve module to use its logic/constants if needed, but we'll re-implement the call flow for clarity

# Load environment variables
load_dotenv(find_dotenv())

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Initialize Clients for the Tool
# We do this globally or inside the tool. Doing it globally avoids re-init on every call.
try:
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    cohere_api_key = os.getenv("COHERE_API_KEY")
    collection_name = os.getenv("QDRANT_COLLECTION_NAME", "rag_embedding")

    if not all([qdrant_url, qdrant_api_key, cohere_api_key]):
        logging.warning("Missing Qdrant/Cohere environment variables. RAG tool might fail.")
        qdrant_client = None
        cohere_client = None
    else:
        qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        cohere_client = CohereClient(cohere_api_key)
except Exception as e:
    logging.error(f"Failed to initialize RAG clients: {e}")
    qdrant_client = None
    cohere_client = None

# Gemini / OpenAI Configuration
gemini_api_key = os.getenv("GEMINI_API_KEY")
if not gemini_api_key:
    logging.error("GEMINI_API_KEY not found in environment variables.")

external_client = AsyncOpenAI(
    api_key=gemini_api_key,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

model = OpenAIChatCompletionsModel(
    model="gemini-2.5-flash",
    openai_client=external_client
)

run_config = RunConfig(
    model=model,
    model_provider=external_client # RunConfig might not accept model_provider directly in all versions, checking SDK...
    # The reference used `model_provider=external_client`. I'll keep it.
)

@function_tool
async def search_knowledge_base(query: str) -> str:
    """
    Searches the internal knowledge base (Qdrant) for relevant information about the website.
    Use this tool when the user asks questions about the specific content of the website.
    
    Args:
        query (str): The search query to find relevant context.
        
    Returns:
        str: A formatted string containing the top relevant text chunks and their sources.
    """
    if not qdrant_client or not cohere_client:
        return "Error: Search service is not available (clients not initialized)."

    logging.info(f"Agent searching for: {query}")

    # 1. Embed the query
    try:
        # Reuse logic from retrieve.py (adapted)
        # retrieve.embed_query uses sync Cohere client.
        response = cohere_client.embed(
            texts=[query],
            model="embed-multilingual-v3.0", # Matching retrieve.py / main.py
            input_type="search_query"
        )
        query_vector = response.embeddings[0]
    except Exception as e:
        return f"Error generating embedding: {str(e)}"

    # 2. Search Qdrant
    try:
        results = qdrant_client.query_points(
            collection_name=collection_name,
            query=query_vector,
            limit=5
        ).points
    except Exception as e:
        return f"Error querying database: {str(e)}"

    if not results:
        return "No relevant information found in the knowledge base."

    # 3. Format Results
    formatted_results = "Found the following relevant context:\n\n"
    for result in results:
        payload = result.payload
        content = payload.get("content", "N/A")
        url = payload.get("url", "N/A")
        formatted_results += f"Source: {url}\nContent: {content}\n---\n"

    return formatted_results

def create_agent() -> Agent:
    """
    Creates and returns the RAG Assistant agent.
    """
    return Agent(
        name="RAG Assistant",
        instructions=(
            "You are a helpful assistant for the Physical AI & Humanoid Robotics Textbook website. "
            "Always check the knowledge base using the 'search_knowledge_base' tool before answering questions about the website, robotics, or AI modules. "
            "If the tool returns information, use it to synthesize a comprehensive answer. "
            "If the tool returns no information, admit that you don't know based on the available context. "
            "Cite the source URLs provided by the tool in your answer."
        ),
        tools=[search_knowledge_base],
        model=model
    )

import sys

async def main():
    agent = create_agent()

    if len(sys.argv) > 1:
        # Single run mode
        query = sys.argv[1]
        print(f"Running query: {query}")
        try:
            result = await Runner.run(
                starting_agent=agent,
                input=query,
                run_config=run_config
            )
            print(f"Agent: {result.final_output}")
        except Exception as e:
            logging.error(f"Error: {e}")
        return

    print("RAG Agent initialized. Type 'exit' to quit.")
    
    while True:
        try:
            user_input = input("\nYou: ")
            if user_input.lower() in ["exit", "quit"]:
                break
            
            # Simple run loop
            # result = await Runner.run(
            #     starting_agent=agent,
            #     input=user_input,
            #     # run_config=run_config # Passing run_config here if supported by Runner.run in this version
            # )
            
            # Note: The reference used run_config in Runner.run.
            # But wait, Runner.run returns a Result object.
            # I need to verify how to pass the model config.
            # The Agent constructor takes `model`.
            # The reference code did: 
            # model = OpenAIChatCompletionsModel(...)
            # run_config = RunConfig(model=model, ...)
            # Runner.run(..., run_config=run_config)
            
            # However, I should probably pass the model to the Agent?
            # agent = Agent(model=model, ...)
            # Let's try passing the model name string to Agent and see if it uses the default client?
            # No, we want to use the Gemini client.
            
            # Option A: Pass model object to Agent.
            # Option B: Pass run_config to Runner.run (as in reference).
            
            # I'll stick to the reference pattern: pass run_config to Runner.run
            # AND I'll explicitly update the Agent's model to be safe?
            # Actually, `run_config` overrides?
            
            # Let's try to follow reference exactly.
            
            # Re-running Runner with config
            result = await Runner.run(
                starting_agent=agent,
                input=user_input
            )

            print(f"Agent: {result.final_output}")

        except KeyboardInterrupt:
            break
        except Exception as e:
            logging.error(f"An error occurred: {e}")

if __name__ == "__main__":
    asyncio.run(main())
