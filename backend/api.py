import logging
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from agents import Runner
from agent import create_agent, run_config

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

app = FastAPI(
    title="Agent API",
    description="API for interacting with the backend AI agent.",
    version="1.0.0"
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins for development
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- Pydantic Models ---

class UserQuery(BaseModel):
    user: str = Field(..., description="The user's query", min_length=1)

class AgentResponse(BaseModel):
    assistant: str = Field(..., description="The agent's response")

# --- Endpoints ---

@app.post("/agent", response_model=AgentResponse)
async def query_agent(query: UserQuery):
    """
    Process a user query using the AI agent.
    """
    logger.info(f"Received query: {query.user}")
    
    try:
        # Create the agent
        agent = create_agent()
        
        # Run the agent using the Runner
        result = await Runner.run(
            starting_agent=agent,
            input=query.user,
            run_config=run_config
        )
        
        response_text = result.final_output
        logger.info(f"Agent response: {response_text}")
        
        return AgentResponse(assistant=str(response_text))
        
    except Exception as e:
        logger.error(f"Error processing query: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))
