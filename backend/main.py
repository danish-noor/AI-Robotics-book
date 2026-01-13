from fastapi import FastAPI, HTTPException, Query
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import os
import asyncio
from contextlib import asynccontextmanager
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import required libraries (will need to install these)
try:
    import openrouter
    from qdrant_client import QdrantClient
    from qdrant_client.http import models
    import asyncpg
except ImportError:
    logger.warning("Required libraries not installed. Install with: pip install openrouter qdrant-client asyncpg")

# Configuration
QDRANT_URL = os.getenv("QDRANT_URL", "https://your-qdrant-instance.qdrant.tech")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY")
NEON_DB_URL = os.getenv("NEON_DB_URL")

class ChatMessage(BaseModel):
    role: str  # "user" or "assistant"
    content: str

class ChatRequest(BaseModel):
    message: str
    session_id: Optional[str] = None
    selected_text: Optional[str] = None  # For selection-aware RAG
    history: Optional[List[ChatMessage]] = []

class ChatResponse(BaseModel):
    response: str
    session_id: str
    sources: Optional[List[Dict[str, Any]]] = []

class HealthResponse(BaseModel):
    status: str
    services: Dict[str, bool]

# Global variables for services
qdrant_client = None
neon_pool = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Initialize services on startup and clean up on shutdown."""
    global qdrant_client, neon_pool

    # Initialize Qdrant client
    try:
        qdrant_client = QdrantClient(
            url=QDRANT_URL,
            api_key=QDRANT_API_KEY,
            timeout=10
        )
        # Verify connection
        qdrant_client.get_collections()
        logger.info("Connected to Qdrant successfully")
    except Exception as e:
        logger.error(f"Failed to connect to Qdrant: {e}")
        qdrant_client = None

    # Initialize Neon Postgres connection pool
    try:
        neon_pool = await asyncpg.create_pool(NEON_DB_URL, min_size=1, max_size=10)
        logger.info("Connected to Neon Postgres successfully")
    except Exception as e:
        logger.error(f"Failed to connect to Neon Postgres: {e}")
        neon_pool = None

    yield

    # Cleanup
    if qdrant_client:
        qdrant_client.close()
    if neon_pool:
        await neon_pool.close()

app = FastAPI(
    title="Docusaurus RAG Chatbot API",
    description="API for RAG-powered chatbot integrated with Docusaurus documentation",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, configure specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/health", response_model=HealthResponse)
async def health_check():
    """Health check endpoint to verify all services are running."""
    services_status = {
        "qdrant": qdrant_client is not None,
        "neon": neon_pool is not None,
        "openrouter": OPENROUTER_API_KEY is not None
    }

    overall_status = "healthy" if all(services_status.values()) else "degraded"

    return HealthResponse(
        status=overall_status,
        services=services_status
    )

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    Main chat endpoint that handles both global RAG and selection-aware RAG.
    """
    try:
        # Generate or validate session ID
        session_id = request.session_id or f"session_{hash(request.message) % 1000000}"

        # Prepare context based on selection
        if request.selected_text:
            # Selection-aware RAG: use selected text as primary context
            context = await get_context_for_selection(request.selected_text)
        else:
            # Global RAG: search across entire book content
            context = await get_context_for_query(request.message)

        # Prepare the full prompt with context
        full_prompt = prepare_prompt(request.message, context, request.history)

        # Generate response using OpenRouter
        response_text = await generate_response(full_prompt)

        # Save to session history
        if neon_pool:
            await save_to_session_history(session_id, request.message, response_text)

        return ChatResponse(
            response=response_text,
            session_id=session_id,
            sources=context.get("sources", []) if context else []
        )

    except Exception as e:
        logger.error(f"Error in chat endpoint: {e}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")

@app.get("/sessions/{session_id}")
async def get_session_history(session_id: str):
    """Retrieve conversation history for a session."""
    if not neon_pool:
        raise HTTPException(status_code=500, detail="Database not available")

    try:
        async with neon_pool.acquire() as connection:
            result = await connection.fetch(
                "SELECT message, role, timestamp FROM chat_history WHERE session_id = $1 ORDER BY timestamp ASC",
                session_id
            )

            history = [
                {"role": row["role"], "content": row["message"], "timestamp": row["timestamp"]}
                for row in result
            ]

            return {"session_id": session_id, "history": history}

    except Exception as e:
        logger.error(f"Error retrieving session history: {e}")
        raise HTTPException(status_code=500, detail="Failed to retrieve session history")

async def get_context_for_query(query: str) -> Optional[Dict[str, Any]]:
    """
    Retrieve relevant context from Qdrant vector database based on the query.
    """
    if not qdrant_client:
        return None

    try:
        # Search for relevant chunks in the vector database
        search_results = qdrant_client.search(
            collection_name="book_content",
            query_text=query,
            limit=5,  # Retrieve top 5 most relevant chunks
            with_payload=True
        )

        context_chunks = []
        sources = []

        for result in search_results:
            if result.payload:
                context_chunks.append(result.payload.get("content", ""))
                sources.append({
                    "title": result.payload.get("title", ""),
                    "section": result.payload.get("section", ""),
                    "page": result.payload.get("page", ""),
                    "score": result.score
                })

        return {
            "content": "\n\n".join(context_chunks),
            "sources": sources
        }

    except Exception as e:
        logger.error(f"Error retrieving context: {e}")
        return None

async def get_context_for_selection(selected_text: str) -> Optional[Dict[str, Any]]:
    """
    Retrieve context specifically focused on the selected text.
    """
    # For selection-aware RAG, we can use the selected text directly
    # or perform a focused search around the selected content
    return {
        "content": selected_text,
        "sources": [{"title": "Selected Text", "section": "User Selection", "score": 1.0}]
    }

def prepare_prompt(user_query: str, context: Optional[Dict[str, Any]], history: List[ChatMessage]) -> str:
    """
    Prepare the full prompt for the LLM with context and conversation history.
    """
    prompt_parts = []

    # Add system context
    prompt_parts.append("You are an AI assistant for an AI Robotics Book. Use the following context to answer the user's question. Be concise and accurate, referencing the book content when possible.")

    # Add context if available
    if context and context.get("content"):
        prompt_parts.append(f"\nContext from the book:\n{context['content']}")

    # Add conversation history
    if history:
        prompt_parts.append("\nPrevious conversation:")
        for msg in history:
            prompt_parts.append(f"{msg.role}: {msg.content}")

    # Add current query
    prompt_parts.append(f"\nUser question: {user_query}")
    prompt_parts.append("\nAssistant:")

    return "\n".join(prompt_parts)

async def generate_response(prompt: str) -> str:
    """
    Generate response using OpenRouter API with Qwen model.
    """
    if not OPENROUTER_API_KEY:
        return "Sorry, the AI service is not configured properly."

    try:
        # Using OpenRouter API - this is a placeholder implementation
        # Actual implementation would use the openrouter library or direct API calls
        import openai

        # Configure OpenAI to use OpenRouter
        openai.api_key = OPENROUTER_API_KEY
        openai.base_url = "https://openrouter.ai/api/v1"

        response = await openai.ChatCompletion.acreate(
            model="qwen/qwen-2-72b-instruct",  # Using Qwen model via OpenRouter
            messages=[{"role": "user", "content": prompt}],
            stream=False,
            temperature=0.7,
            max_tokens=1000
        )

        return response.choices[0].message.content

    except Exception as e:
        logger.error(f"Error generating response: {e}")
        return "Sorry, I encountered an error while generating a response. Please try again."

async def save_to_session_history(session_id: str, user_message: str, assistant_response: str):
    """
    Save the conversation to the session history in the database.
    """
    if not neon_pool:
        return

    try:
        async with neon_pool.acquire() as connection:
            # Insert user message
            await connection.execute(
                """
                INSERT INTO chat_history (session_id, message, role, timestamp)
                VALUES ($1, $2, 'user', NOW())
                """,
                session_id, user_message
            )

            # Insert assistant response
            await connection.execute(
                """
                INSERT INTO chat_history (session_id, message, role, timestamp)
                VALUES ($1, $2, 'assistant', NOW())
                """,
                session_id, assistant_response
            )
    except Exception as e:
        logger.error(f"Error saving to session history: {e}")

@app.get("/")
async def root():
    """Root endpoint for basic service information."""
    return {
        "message": "Docusaurus RAG Chatbot API",
        "version": "1.0.0",
        "endpoints": ["/health", "/chat", "/sessions/{session_id}"]
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)