# Research Findings: Interactive RAG Chatbot Integration

## 1. React Testing Framework for Docusaurus Frontend

- **Decision**: Jest and React Testing Library (RTL)
- **Rationale**: These are the industry-standard and recommended tools for testing React applications, including those built with Docusaurus. RTL encourages user-centric testing, aligning with UX goals.
- **Alternatives Considered**: Enzyme (less recommended for modern React, focuses on implementation details).

## 2. Target Platform (Hosting Strategy) for Python Backend

- **Decision**: Google Cloud Functions
- **Rationale**: Aligns with existing Google services usage (Gemini API), offers serverless scalability, and suits the event-driven nature of API endpoints for the RAG chatbot. Simple for initial MVP.
- **Alternatives Considered**: Google Cloud Run (more flexible containerization, good for future growth), AWS Lambda, Azure Functions (less alignment with existing Google ecosystem).

## 3. Specific Risks Related to RAG Integration

### a. Retrieval Relevance Risk
- **Description**: Vector search may fail to retrieve the most relevant text chunks, leading to inaccurate or unhelpful chatbot responses (impacting SC-001).
- **Impact**: Poor user experience, reduced trust, unmet accuracy goals.
- **Mitigation**: Fine-tune chunking strategies, experiment with different embedding models, implement user feedback mechanisms for relevance, explore re-ranking techniques for retrieved documents.

### b. LLM Hallucination/Bias Risk
- **Description**: The LLM might generate responses not strictly grounded in retrieved textbook content (hallucination) or display biases.
- **Impact**: Misinformation, erosion of trust, ethical concerns.
- **Mitigation**: Strict prompt engineering emphasizing content grounding, post-processing for fact-checking, regular response reviews, clear indication of AI-generated content, consider content moderation.

### c. Performance/Latency Risk
- **Description**: Combined latency from vector search, LLM inference, and network communication could exceed NFR1.1 (5-second response time for 90% of queries), especially under load.
- **Impact**: Frustrated users, degraded user experience.
- **Mitigation**: Optimize vector DB queries, explore lower-latency LLMs, implement caching, optimize network, scale backend infrastructure appropriately.

### d. Ingestion/Indexing Lag Risk
- **Description**: Delays or failures in re-indexing (FR-011) could result in the chatbot providing outdated or missing textbook information.
- **Impact**: Inaccurate responses, user frustration.
- **Mitigation**: Implement robust monitoring and alerting for the indexing pipeline, ensure efficient incremental indexing, define clear rollback strategies for failed updates, provide user communication on data freshness.
