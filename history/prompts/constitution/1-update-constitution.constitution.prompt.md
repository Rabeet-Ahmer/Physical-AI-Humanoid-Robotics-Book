---
id: 1
title: Update Constitution
stage: constitution
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: Rabeet Ahmer
command: /sp.constitution
labels: [constitution,rag-ai-chatbot,tech-stack]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```text
Project: RAG AI Chatbot for a book

# Tech Stack:
- 'uv' as package manager.
- FastAPI[standards] for backend.
- OpenAI Agents SDK (package: openai-agents) to build AI agents.
- Qdrant for RAG Vector database (the book content will be stored as vector embeddings).
- Neon Serverless Postgres database to store other data.

# Core Principles:
- Python as a core language.
- For any technical info about the tech stack or about the technical documentation for the code  must be fetched using 'Context7' mcp.
- No hallucinations, assumptions, incorrect code
- If any data is required search on web or if the required data is about tech use Context7
```

## Response snapshot

Updated the project constitution with principles and guidelines for the RAG AI Chatbot for a book project. The new constitution includes:

- Python-First Development principle
- Tech Stack Adherence principle
- No Hallucinations Policy
- Context7 Documentation Standard
- RAG-Centric Architecture principle
- Security and Data Integrity principle
- Technology and Infrastructure Standards section
- Development Workflow section
- Governance section

## Outcome

- ✅ Impact: Updated constitution with project-specific principles and guidelines
- 🧪 Tests: none
- 📁 Files: .specify/memory/constitution.md
- 🔁 Next prompts:
- 🧠 Reflection: Successfully created a comprehensive constitution for the RAG AI Chatbot project based on the provided tech stack and principles.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL):
- Prompt variant (if applicable):
- Next experiment (smallest change to try):