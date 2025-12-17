# Research: Create API Endpoints for Frontend

**Feature**: Create API Endpoints for Frontend
**Status**: Completed

## 1. FastAPI and OpenAI Agents SDK Integration

**Unknown**: Best practices for serving an OpenAI Agent (from `openai-agents` SDK) via FastAPI.

**Findings**:
- The `openai-agents` SDK uses a `Runner` class to execute agents asynchronously (`await Runner.run(...)`).
- This fits perfectly with FastAPI's `async def` route handlers.
- **Pattern**:
  ```python
  @app.post("/agent")
  async def run_agent(request: UserQuery):
      agent = get_agent() # From refactored module
      result = await Runner.run(starting_agent=agent, input=request.user)
      return {"assistant": result.final_output}
  ```

## 2. Refactoring `agent.py`

**Unknown**: Cleanest pattern to refactor `agent.py`.

**Findings**:
- The current `agent.py` creates the agent inside `main()`.
- **Decision**: Extract the agent creation logic into a function `create_agent()` or a module-level variable `agent` (if stateless).
- Given the `Agent` class holds configuration (tools, instructions), it can be instantiated once (Singleton) or per request. Since it holds no conversational state (that's in the Runner's session/run), a Singleton `agent` instance is efficient.
- **Refactor Plan**:
  - Move initialization code (env checks, tool definitions, agent creation) to top-level or a `get_agent()` function.
  - Keep `if __name__ == "__main__":` block for CLI usage, calling `get_agent()` and running the loop.

## Decisions

- **Architecture**: Use `FastAPI` with `async` routes calling `openai-agents.Runner.run()`.
- **Refactoring**: Modify `agent.py` to expose `agent` (or `create_agent`) for import.
- **Data Model**: Use Pydantic models `UserQuery(BaseModel)` and `AgentResponse(BaseModel)` for strict validation.