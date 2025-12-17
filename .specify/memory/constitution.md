<!--
Sync Impact Report:
Version change: 1.0.0 -> 1.1.0
Added sections: Detailed subsections for each principle, Acceptance Criteria, Error Handling, Performance Standards, Compliance Requirements
Removed sections: None
Templates requiring updates: ⚠ pending - .specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md
Follow-up TODOs: None
-->
# RAG AI Chatbot Constitution

## Core Principles

### Project Directory
All backend code must be written within the dedicated folder named **@Hackathon1.0/backend**.

### Python-First Development
All backend services and components must be implemented in Python as the core language. Python libraries and frameworks are preferred for consistency, maintainability, and team expertise.

**Requirements:**
- All backend services must be written in Python 3.9+
- Use of Python standard libraries and established frameworks is preferred
- Third-party Python packages must be justified and reviewed
- Code must follow PEP 8 style guidelines and be formatted with black

**Rationale:**
Python provides excellent ecosystem support for AI/ML applications, has strong community support, and integrates well with the specified tech stack.

### Tech Stack Adherence
Strict adherence to the designated technology stack: uv as package manager, FastAPI[standards] for backend APIs, OpenAI Agents SDK for AI functionality, Qdrant for vector database operations, and Neon Serverless Postgres for relational data storage.

**Required Technologies:**
- **Package Management**: uv for fast, secure dependency management
- **Backend Framework**: FastAPI with standards extras for API development
- **AI Agents**: OpenAI Agents SDK for building intelligent agents
- **Vector Database**: Qdrant for RAG operations and embeddings storage
- **Relational Database**: Neon Serverless Postgres for structured data

**Acceptance Criteria:**
- All dependencies must be managed through uv with pyproject.toml and uv.lock files
- API endpoints must be documented with OpenAPI/Swagger via FastAPI
- Database connections must use the specified database technologies
- No unauthorized technology additions without explicit approval

**Requirements:**
- All functions and classes must have proper type hints
- Error handling must be comprehensive with appropriate logging
- Code must be modular with clear separation of concerns
- Security best practices must be followed (input validation, sanitization, etc.)
- Scalability considerations must be addressed in architecture decisions
- Maintainability through clean code practices and documentation
- Observability through proper logging, monitoring, and metrics

**Rationale:**
Production-grade code ensures reliability, security, and maintainability of the application over time.

### No Hallucinations Policy
No hallucinations, assumptions, or incorrect code are allowed. All technical information must be verified through reliable sources like Context7 MCP for tech stack documentation, or web searches for general information.

**Requirements:**
- All technical claims must be verified through Context7 MCP or reliable sources
- Code examples must be tested before implementation
- Documentation must be accurate and up-to-date
- Assumptions must be explicitly documented and validated

**Error Handling:**
- When uncertain about technical implementation, consult Context7 MCP first
- If documentation is unclear, ask for clarification rather than assuming
- Create issues to track any knowledge gaps for future resolution

### Context7 Documentation Standard
For any technical information about the tech stack or code documentation, Context7 MCP must be used to fetch accurate, up-to-date technical documentation and avoid assumptions.

**Process:**
- Before implementing any feature, check Context7 for current best practices
- Use Context7 for API reference and usage patterns
- Document Context7 references in code comments when applicable
- Keep Context7 documentation links in technical decisions

**Compliance:**
- All technical decisions must reference verified documentation
- Code reviews must verify documentation alignment
- New implementations must follow documented patterns

### RAG-Centric Architecture
All content and document retrieval must leverage the RAG (Retrieval Augmented Generation) architecture with Qdrant vector database storing content as vector embeddings for efficient similarity search and AI-powered responses.

**Architecture Requirements:**
- Content must be processed into vector embeddings
- Vector storage must use Qdrant with proper indexing for performance
- Search functionality must implement semantic similarity matching
- Response generation must integrate retrieved context with AI agent responses

**Performance Standards:**
- Vector search response time must be < 200ms for 95% of queries
- Document ingestion pipeline must handle various formats (PDF, DOCX, TXT, etc.)
- Embedding generation must be batch-processed for efficiency

## Technology and Infrastructure Standards

### API Standards
- All APIs must follow RESTful principles where possible
- OpenAPI documentation must be comprehensive and up-to-date
- API versioning must follow semantic versioning (v1, v2, etc.)
- Response formats must be consistent JSON with proper error handling
- HTTP status codes must be used appropriately

### Performance Standards
- API response times must be < 500ms for 95% of requests
- Database queries must be optimized with proper indexing
- Vector search performance must be monitored and optimized
- Caching strategies must be implemented for frequently accessed data

### Database Standards
- Database schema changes must be version controlled and migration-managed
- Connection pooling must be implemented for efficiency
- Query optimization must be performed regularly
- Data retention policies must be defined and enforced

## Development Workflow

### Code Quality Standards
- Type hints must be used for all public interfaces and strongly encouraged for private functions
- Documentation must be provided for all public functions/classes using docstrings
- Code must follow PEP 8 style guidelines and be formatted with black
- Unit tests must be written for all business logic
- Code complexity must be kept reasonable (avoid deeply nested functions)

### Review Process
- All pull requests must have at least one peer review
- Automated tests must pass before merging
- Code coverage must not decrease
- Security scans must pass
- Documentation updates must be included when applicable

### Dependency Management
- Dependencies must be managed through uv
- Security vulnerabilities must be addressed promptly
- New dependencies must be justified and reviewed

## Quality Assurance

### Acceptance Criteria
- All features must have clear, testable acceptance criteria
- User stories must be validated before implementation
- Performance benchmarks must be met before deployment
- Security requirements must be verified

### Error Handling
- All API endpoints must return appropriate error codes
- Error messages must be informative but not expose internal details
- Fallback mechanisms must be implemented for critical failures
- Logging must capture sufficient context for debugging

### Monitoring and Observability
- Application metrics must be collected and monitored
- Error tracking must be implemented
- Performance monitoring must cover all critical paths
- User activity must be logged for analytics

## Governance

### Amendment Process
- This constitution supersedes all other development practices
- Amendments require explicit documentation and team approval
- Changes must be communicated to all team members
- Migration plans must be created for breaking changes

### Compliance Verification
- All pull requests must verify compliance with these principles
- Regular audits must be conducted to ensure adherence
- Non-compliance issues must be addressed promptly
- Use of external libraries must align with the approved tech stack unless specifically justified

### Versioning Policy
- Constitution versioning follows semantic versioning principles
- MAJOR version changes require team approval and migration planning
- MINOR changes should be backward compatible
- PATCH changes are for clarifications and minor adjustments

**Version**: 1.1.0 | **Ratified**: 2025-12-15 | **Last Amended**: 2025-12-15