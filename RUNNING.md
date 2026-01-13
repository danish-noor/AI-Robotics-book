# Running the AI Robotics Book with RAG Chatbot

This document explains how to run the complete application with both the Docusaurus frontend and FastAPI backend.

## Prerequisites

- Node.js (v16 or higher)
- Python (v3.8 or higher)
- Access to OpenRouter API
- Qdrant Cloud account
- Neon Postgres account

## Quick Setup

1. **Install dependencies:**
   ```bash
   npm run backend-install  # Install backend Python dependencies
   npm install             # Install frontend dependencies
   ```

2. **Configure environment variables:**
   - Copy the `.env` file in the `backend/` directory and update with your credentials

3. **Index the book content:**
   ```bash
   npm run ingest-docs
   ```

## Running the Application

The application consists of three main components that need to run simultaneously:

### 1. Backend Server (FastAPI)

```bash
npm run backend
```

This starts the FastAPI server on `http://localhost:8000`

### 2. Frontend Server (Docusaurus)

In a separate terminal:

```bash
npm start
```

This starts the Docusaurus development server on `http://localhost:3000`

### 3. Document Ingestion

Before using the chatbot, you need to index the book content:

```bash
npm run ingest-docs
```

## Development Workflow

### Adding New Content

1. Add your new markdown files to the `docs/` directory
2. Re-run the ingestion script to update the vector database:
   ```bash
   npm run ingest-docs
   ```

### Environment Variables

Create a `.env` file in the `backend/` directory with:

```env
QDRANT_URL=https://your-qdrant-instance.qdrant.tech
QDRANT_API_KEY=your_qdrant_api_key
OPENROUTER_API_KEY=your_openrouter_api_key
NEON_DB_URL=your_neon_postgres_connection_string
```

## Architecture Overview

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Docusaurus    │    │   FastAPI        │    │   Qdrant        │
│   Frontend      │◄──►│   Backend        │◄──►│   Vector DB     │
│   (Port 3000)   │    │   (Port 8000)    │    │                 │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         ▲                       ▲
         │                       │
         │                ┌──────────────────┐
         │                │   Neon Postgres  │
         └────────────────│   Session DB     │
                          └──────────────────┘
```

## Troubleshooting

### Common Issues

1. **Backend not connecting to Qdrant:**
   - Verify your QDRANT_URL and QDRANT_API_KEY are correct
   - Check that your Qdrant Cloud instance is running

2. **OpenRouter API errors:**
   - Verify your OPENROUTER_API_KEY is valid
   - Check that you have access to the Qwen models

3. **Document ingestion fails:**
   - Ensure all required environment variables are set
   - Verify that the docs directory contains markdown files

4. **Chatbot not appearing:**
   - Make sure the Docusaurus site has reloaded after adding the theme components
   - Check browser console for any JavaScript errors

## API Endpoints

### Backend (http://localhost:8000)

- `GET /health` - Health check
- `POST /chat` - Chat endpoint with streaming responses
- `GET /sessions/{session_id}` - Get conversation history

### Frontend (http://localhost:3000)

The chatbot is integrated into all pages as a floating button in the bottom-right corner.