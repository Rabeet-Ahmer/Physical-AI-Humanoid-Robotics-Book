# Contracts: Docusaurus RAG Chat Widget

This feature consumes the API contract defined by the backend RAG Agent API.

## Consumed API

- **Endpoint**: `POST /agent`
- **Specification**: Refer to `specs/001-create-api-endpoints/contracts/openapi.yaml`
- **Request Body**: `{"user": "string"}`
- **Response Body**: `{"assistant": "string"}`
