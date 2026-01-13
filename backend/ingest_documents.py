"""
Script to ingest Docusaurus book content into Qdrant vector database
"""
import os
import asyncio
from typing import List, Dict, Any
import logging
from pathlib import Path
import hashlib

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

try:
    from qdrant_client import QdrantClient
    from qdrant_client.http import models
    from qdrant_client.models import PointStruct
    import openai
    import markdown
except ImportError:
    logger.error("Required libraries not installed. Install with: pip install qdrant-client openai markdown")

# Configuration
QDRANT_URL = os.getenv("QDRANT_URL", "https://your-qdrant-instance.qdrant.tech")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")  # Using OpenAI for embeddings, or OpenRouter
OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY")

# Initialize clients
qdrant_client = None
openai_client = None

class DocumentChunk:
    """Represents a chunk of a document with metadata."""
    def __init__(self, content: str, title: str, section: str, page: str, source_path: str):
        self.content = content
        self.title = title
        self.section = section
        self.page = page
        self.source_path = source_path
        self.id = self.generate_id()

    def generate_id(self) -> str:
        """Generate a unique ID for this chunk."""
        content_hash = hashlib.md5(self.content.encode()).hexdigest()
        return f"{self.page}_{content_hash[:8]}"

def extract_title_from_markdown(content: str) -> str:
    """Extract the title from markdown content (first H1)."""
    lines = content.split('\n')
    for line in lines:
        if line.startswith('# '):
            return line[2:].strip()
    return "Untitled"

def chunk_markdown_content(content: str, max_chunk_size: int = 1000, overlap: int = 100) -> List[str]:
    """
    Split markdown content into chunks of appropriate size.
    """
    # Convert markdown to plain text for processing
    try:
        import markdown
        plain_text = markdown.markdown(content)
        # Remove HTML tags
        import re
        plain_text = re.sub('<[^<]+?>', '', plain_text)
    except ImportError:
        # If markdown library not available, work with raw content
        plain_text = content

    # Split into sentences
    sentences = plain_text.split('. ')
    chunks = []
    current_chunk = ""

    for sentence in sentences:
        sentence = sentence.strip()
        if not sentence:
            continue

        # Check if adding this sentence would exceed the chunk size
        if len(current_chunk) + len(sentence) > max_chunk_size:
            if current_chunk.strip():
                chunks.append(current_chunk.strip())

            # Start a new chunk, possibly with overlap
            if overlap > 0:
                # Get the last part of current chunk to create overlap
                overlap_text = current_chunk[-overlap:] if len(current_chunk) > overlap else current_chunk
                current_chunk = overlap_text + " " + sentence
            else:
                current_chunk = sentence
        else:
            current_chunk += " " + sentence

    # Add the last chunk if it has content
    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    return chunks

def read_docusaurus_docs(docs_path: str) -> List[Dict[str, Any]]:
    """
    Read all markdown files from the Docusaurus docs directory.
    """
    docs_path = Path(docs_path)
    documents = []

    # Find all markdown files in the docs directory
    for md_file in docs_path.rglob("*.md"):
        try:
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract title
            title = extract_title_from_markdown(content)

            # Create document entry
            doc_info = {
                'content': content,
                'title': title,
                'path': str(md_file),
                'filename': md_file.name,
                'relative_path': str(md_file.relative_to(docs_path))
            }

            documents.append(doc_info)
            logger.info(f"Processed document: {doc_info['relative_path']}")

        except Exception as e:
            logger.error(f"Error reading file {md_file}: {e}")

    return documents

async def get_embedding(text: str, model: str = "text-embedding-ada-002") -> List[float]:
    """
    Get embedding for text using OpenAI or OpenRouter.
    """
    if not OPENAI_API_KEY and not OPENROUTER_API_KEY:
        raise ValueError("Either OPENAI_API_KEY or OPENROUTER_API_KEY must be set")

    # Use OpenAI API for embeddings
    try:
        import openai
        if OPENAI_API_KEY:
            openai.api_key = OPENAI_API_KEY
        else:
            openai.api_key = OPENROUTER_API_KEY
            # For OpenRouter, we might need to set the base URL
            openai.base_url = "https://openrouter.ai/api/v1"

        response = await openai.Embedding.acreate(
            input=text,
            model=model
        )

        return response.data[0].embedding
    except Exception as e:
        logger.error(f"Error getting embedding: {e}")
        # Return a dummy embedding if API fails
        return [0.0] * 1536  # Standard size for text-embedding-ada-002

async def create_qdrant_collection(collection_name: str = "book_content"):
    """
    Create a Qdrant collection for storing book content embeddings.
    """
    try:
        # Check if collection exists
        collections = qdrant_client.get_collections()
        collection_exists = any(col.name == collection_name for col in collections.collections)

        if collection_exists:
            logger.info(f"Collection '{collection_name}' already exists")
            return

        # Create new collection
        qdrant_client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(
                size=1536,  # Standard size for text-embedding-ada-002
                distance=models.Distance.COSINE
            )
        )

        logger.info(f"Created collection '{collection_name}' successfully")

    except Exception as e:
        logger.error(f"Error creating Qdrant collection: {e}")
        raise

async def ingest_documents(docs_path: str, collection_name: str = "book_content"):
    """
    Main function to ingest documents from Docusaurus into Qdrant.
    """
    global qdrant_client

    # Initialize Qdrant client
    if not QDRANT_API_KEY:
        raise ValueError("QDRANT_API_KEY environment variable must be set")

    qdrant_client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY,
        timeout=30
    )

    # Create the collection if it doesn't exist
    await create_qdrant_collection(collection_name)

    # Read documents from Docusaurus docs directory
    logger.info(f"Reading documents from: {docs_path}")
    documents = read_docusaurus_docs(docs_path)

    if not documents:
        logger.warning(f"No documents found in {docs_path}")
        return

    logger.info(f"Found {len(documents)} documents to process")

    # Process each document
    all_chunks = []
    for doc in documents:
        logger.info(f"Processing document: {doc['filename']}")

        # Extract title from document
        title = doc['title']

        # Chunk the content
        content_chunks = chunk_markdown_content(doc['content'])

        # Create DocumentChunk objects
        for i, chunk_text in enumerate(content_chunks):
            chunk = DocumentChunk(
                content=chunk_text,
                title=title,
                section=f"{doc['filename']}#{i+1}",
                page=doc['filename'],
                source_path=doc['relative_path']
            )
            all_chunks.append(chunk)

    logger.info(f"Created {len(all_chunks)} content chunks to index")

    # Process chunks in batches
    batch_size = 10  # Adjust based on API limits and performance
    for i in range(0, len(all_chunks), batch_size):
        batch = all_chunks[i:i + batch_size]
        logger.info(f"Processing batch {i//batch_size + 1}/{(len(all_chunks)-1)//batch_size + 1}")

        # Prepare batch data for Qdrant
        points = []
        for chunk in batch:
            # Get embedding for the chunk content
            try:
                embedding = await get_embedding(chunk.content)
            except Exception as e:
                logger.error(f"Failed to get embedding for chunk {chunk.id}: {e}")
                continue

            # Create Qdrant point
            point = PointStruct(
                id=chunk.id,
                vector=embedding,
                payload={
                    "content": chunk.content,
                    "title": chunk.title,
                    "section": chunk.section,
                    "page": chunk.page,
                    "source_path": chunk.source_path,
                    "chunk_id": chunk.id
                }
            )
            points.append(point)

        # Upload batch to Qdrant
        if points:
            try:
                qdrant_client.upsert(
                    collection_name=collection_name,
                    points=points
                )
                logger.info(f"Uploaded batch of {len(points)} points to Qdrant")
            except Exception as e:
                logger.error(f"Failed to upload batch to Qdrant: {e}")

    logger.info("Document ingestion completed successfully!")

async def main():
    """
    Main entry point for the ingestion script.
    """
    import argparse

    parser = argparse.ArgumentParser(description="Ingest Docusaurus book content into Qdrant")
    parser.add_argument(
        "--docs-path",
        type=str,
        default="D:/ai-book/docs",
        help="Path to Docusaurus docs directory (default: D:/ai-book/docs)"
    )
    parser.add_argument(
        "--collection-name",
        type=str,
        default="book_content",
        help="Name of the Qdrant collection (default: book_content)"
    )

    args = parser.parse_args()

    # Validate paths
    if not os.path.exists(args.docs_path):
        logger.error(f"Docs path does not exist: {args.docs_path}")
        return

    logger.info(f"Starting document ingestion from: {args.docs_path}")
    logger.info(f"Target collection: {args.collection_name}")

    try:
        await ingest_documents(args.docs_path, args.collection_name)
        logger.info("Ingestion process completed successfully!")
    except Exception as e:
        logger.error(f"Ingestion failed: {e}")
        raise

if __name__ == "__main__":
    asyncio.run(main())