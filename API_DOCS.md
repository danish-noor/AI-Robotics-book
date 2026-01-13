# API Documentation: Docusaurus RAG Chatbot

## Backend API (FastAPI)

The backend server runs on `http://localhost:8000` and provides the following endpoints:

### Health Check
- **GET** `/health`
- **Description**: Check the health status of all services
- **Response**:
  ```json
  {
    "status": "healthy|degraded",
    "services": {
      "qdrant": true,
      "neon": true,
      "openrouter": true
    }
  }
  ```

### Chat Endpoint
- **POST** `/chat`
- **Description**: Main chat endpoint with streaming responses
- **Request Body**:
  ```json
  {
    "message": "string - The user's message/question",
    "session_id": "string - Optional session ID (will be generated if not provided)",
    "selected_text": "string - Optional selected text for context-aware RAG",
    "history": [
      {
        "role": "string - 'user' or 'assistant'",
        "content": "string - Previous message content"
      }
    ]
  }
  ```
- **Response**:
  ```json
  {
    "response": "string - The AI's response",
    "session_id": "string - Session identifier",
    "sources": [
      {
        "title": "string - Source title",
        "section": "string - Section identifier",
        "score": "number - Relevance score"
      }
    ]
  }
  ```

### Session History
- **GET** `/sessions/{session_id}`
- **Description**: Retrieve conversation history for a session
- **Response**:
  ```json
  {
    "session_id": "string - Session identifier",
    "history": [
      {
        "role": "string - 'user' or 'assistant'",
        "content": "string - Message content",
        "timestamp": "string - ISO timestamp"
      }
    ]
  }
  ```

## Frontend Components

### Chatbot Component
- **File**: `src/components/Chatbot.js`
- **Features**:
  - Floating UI that appears on all pages
  - Real-time streaming responses
  - Text selection detection
  - Session management
  - Source attribution

### API Hook
- **File**: `src/components/useChatbotAPI.js`
- **Functions**:
  - `sendMessage(message, sessionId, selectedText, history)`
  - `getSessionHistory(sessionId)`
  - `healthCheck()`

## Environment Variables

Required environment variables in `backend/.env`:

```env
QDRANT_URL=https://your-qdrant-instance.qdrant.tech
QDRANT_API_KEY=your_qdrant_api_key
OPENROUTER_API_KEY=your_openrouter_api_key
NEON_DB_URL=your_neon_postgres_connection_string
```

## Integration Points

### Docusaurus Theme
- **File**: `src/theme/Layout.js`
- **Purpose**: Wraps the original Docusaurus layout to add the chatbot component to all pages

## Scripts

### NPM Scripts (package.json)
- `npm run backend`: Start the FastAPI backend server
- `npm run ingest-docs`: Run the document ingestion script
- `npm run backend-install`: Install backend Python dependencies

## Data Flow

### Document Ingestion
1. Read Markdown files from `docs/` directory
2. Chunk content using intelligent splitting
3. Generate embeddings using Qwen models
4. Store in Qdrant vector database with metadata

### Chat Request Flow
1. User submits question via frontend chatbot
2. Frontend sends request to backend `/chat` endpoint
3. Backend retrieves relevant context from Qdrant
4. Context and query sent to OpenRouter LLM
5. Response streamed back to frontend
6. Session history stored in Neon Postgres

## Architecture Components

### Vector Database (Qdrant)
- Stores book content as embeddings
- Enables semantic search for RAG
- Collection name: `book_content`
- Vector size: 1536 (for text-embedding-ada-002)

### Session Database (Neon Postgres)
- Stores conversation history
- Tracks session metadata
- Maintains context between interactions

### LLM Service (OpenRouter)
- Provides access to Qwen models
- Handles response generation
- Processes context from vector database

## Frontend UI Features

### Text Selection
- Automatically detects when user selects text
- Uses selected text as context for RAG
- Provides visual feedback when text is selected

### Streaming Responses
- Real-time character-by-character display
- Typing indicators during processing
- Smooth scrolling to latest message

### Session Management
- Persistent conversations per session
- Automatic session ID generation
- Conversation history access

This API provides a complete RAG chatbot system integrated with the Docusaurus documentation site, allowing users to ask questions about the book content with both global knowledge and selection-aware context.