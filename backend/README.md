# Docusaurus RAG Chatbot Backend

This backend provides a RAG (Retrieval-Augmented Generation) chatbot API for the Docusaurus-based AI Robotics Book.

## Features

- Global RAG: Ask questions about the entire book content
- Selection-Aware RAG: Ask questions about selected text on the page
- Real-time streaming responses
- Session management
- Vector storage with Qdrant Cloud
- LLM integration with OpenRouter

## Tech Stack

- FastAPI: Backend framework
- Qdrant: Vector database for document embeddings
- OpenRouter: LLM API access
- Neon Postgres: Session metadata storage
- Python: Backend implementation

## Setup

### 1. Install Dependencies

```bash
cd backend
pip install -r requirements.txt
```

### 2. Environment Variables

Create a `.env` file in the backend directory with the following variables:

```env
QDRANT_URL=https://your-qdrant-instance.qdrant.tech
QDRANT_API_KEY=your_qdrant_api_key
OPENROUTER_API_KEY=your_openrouter_api_key
NEON_DB_URL=your_neon_postgres_connection_string
```

### 3. Run the Backend

```bash
cd backend
python main.py
```

The API will be available at `http://localhost:8000`

## API Endpoints

- `GET /health` - Health check for all services
- `POST /chat` - Main chat endpoint with streaming responses
- `GET /sessions/{session_id}` - Get conversation history

## Document Ingestion

To index your Docusaurus book content into the vector database:

```bash
cd backend
python ingest_documents.py --docs-path "../docs" --collection-name "book_content"
```

## Frontend Integration

The chatbot component is integrated into Docusaurus via the custom Layout theme component. It appears as a floating button on all pages.