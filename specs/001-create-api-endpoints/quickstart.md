# Quickstart: Create API Endpoints for Frontend

## Prerequisites

- Python 3.10+
- `uv` installed
- Environment variables set in `backend/.env` (QDRANT_URL, API keys, etc.)

## Running the API

1. **Navigate to the backend directory**:
   ```bash
   cd backend
   ```

2. **Install dependencies**:
   ```bash
   uv sync
   ```

3. **Run the FastAPI server**:
   ```bash
   uv run fastapi dev api.py
   ```
   *Note: `fastapi dev` enables hot-reloading. For production, use `fastapi run api.py`.*

4. **Access the API**:
   - Swagger UI: http://127.0.0.1:8000/docs
   - Endpoint: `POST http://127.0.0.1:8000/agent`

## Testing

1. **Manual Test (curl)**:
   ```bash
   curl -X POST "http://127.0.0.1:8000/agent" \
        -H "Content-Type: application/json" \
        -d '{"user": "What is the capital of France?"}'
   ```
