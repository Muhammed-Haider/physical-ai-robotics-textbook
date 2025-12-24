# Data Model: Interactive RAG Chatbot Integration

## 1. Entities

### 1.1 Textbook Content
Represents the entire body of knowledge from the Physical AI & Humanoid Robotics textbook.

-   **Attributes**:
    -   `id`: Unique identifier (string, primary key).
    -   `text_content`: Full text of the document (string).
    -   `source_path`: Original file path or URL (string).
    -   `metadata`: (JSON object) Additional information like:
        -   `chapter`: Chapter name (string).
        -   `section`: Section name (string).
        -   `week`: Associated week in curriculum (string, e.g., "week-01").
        -   `topic`: Main topic (string).
    -   `last_indexed_at`: Timestamp of last indexing (datetime).

-   **Relationships**:
    -   One-to-many with `Text Chunk` (a `Textbook Content` can be broken into multiple `Text Chunks`).

-   **Validation Rules**:
    -   `text_content` must not be empty.
    -   `source_path` must be unique.

### 1.2 Text Chunk
Smaller, embedded segments derived from the `Textbook Content`. These are stored in the Vector Database (Qdrant Cloud).

-   **Attributes**:
    -   `id`: Unique identifier (string, primary key).
    -   `text_content`: Segment of text (string).
    -   `embedding_vector`: Vector representation of `text_content` (array of floats).
    -   `original_source_ref`: Reference to the `Textbook Content` from which it was derived (string, foreign key to `Textbook Content.id`).
    -   `metadata`: (JSON object) Inherited or enriched metadata (e.g., page number, paragraph index).

-   **Relationships**:
    -   Many-to-one with `Textbook Content`.

-   **Validation Rules**:
    -   `text_content` must not be empty.
    -   `embedding_vector` must be a valid vector of expected dimension.
    -   `original_source_ref` must point to an existing `Textbook Content`.

### 1.3 User Query
A natural language question or input from the user via the chatbot interface.

-   **Attributes**:
    -   `id`: Unique identifier (string, primary key).
    -   `user_id`: Identifier of the user (string).
    -   `text_content`: The user's question (string).
    -   `timestamp`: Time of the query (datetime).
    -   `embedding_vector`: Vector representation of `text_content` (array of floats).
    -   `session_id`: Identifier for the conversational session (string).

-   **Relationships**:
    -   One-to-many with `Chatbot Response` (a `User Query` can lead to multiple `Chatbot Responses` in a conversation).
    -   Many-to-one with `User` (a user can make many queries).

-   **Validation Rules**:
    -   `text_content` must not be empty.
    -   `text_content` length should be within acceptable limits (to prevent overly long queries).

### 1.4 Chatbot Response
The generated natural language answer from the LLM based on retrieved text chunks.

-   **Attributes**:
    -   `id`: Unique identifier (string, primary key).
    -   `query_id`: Reference to the `User Query` it responds to (string, foreign key to `User Query.id`).
    -   `text_content`: The generated answer (string).
    -   `timestamp`: Time of the response (datetime).
    -   `retrieved_chunks_refs`: List of references to `Text Chunk`s used for generation (array of strings, foreign keys to `Text Chunk.id`, optional).
    -   `confidence_score`: (float, optional) A measure of the response's confidence or relevance.

-   **Relationships**:
    -   Many-to-one with `User Query`.
    -   Many-to-many with `Text Chunk` (via `retrieved_chunks_refs`).

-   **Validation Rules**:
    -   `text_content` must not be empty.
    -   `query_id` must point to an existing `User Query`.

### 1.5 User (Inferred from User Roles and Permissions)
Represents a user of the system (end-user or administrator).

-   **Attributes**:
    -   `id`: Unique identifier (string, primary key).
    -   `role`: User role (`end-user` or `admin`).
    -   `authentication_details`: Reference to an external authentication system (e.g., Better Auth `user_id`).

-   **Relationships**:
    -   One-to-many with `User Query`.

-   **Validation Rules**:
    -   `id` must be unique.
    -   `role` must be one of `end-user` or `admin`.

## 2. State Transitions (for conversational sessions)

### 2.1 Chat Session
The sequence of interactions between a user and the chatbot.

-   **States**:
    -   `active`: Session is ongoing, awaiting user input or generating response.
    -   `idle`: Session is open but no recent activity (e.g., for X minutes).
    -   `closed`: Session has ended (e.g., explicit user action, timeout).

-   **Transitions**:
    -   `new_query` (from `idle` or `active`) → `active`
    -   `no_activity_timeout` (from `active`) → `idle`
    -   `idle_timeout` (from `idle`) → `closed`
    -   `user_ends_session` (from `active` or `idle`) → `closed`
