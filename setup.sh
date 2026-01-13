#!/bin/bash
# Setup script for AI Robotics Book with RAG Chatbot

echo "Setting up AI Robotics Book with RAG Chatbot..."

# Install frontend dependencies
echo "Installing frontend dependencies..."
npm install

# Install backend dependencies
echo "Installing backend dependencies..."
cd backend
pip install -r requirements.txt

echo "Setup complete!"
echo ""
echo "To run the application:"
echo "1. Start the backend: npm run backend"
echo "2. In a new terminal, start the frontend: npm start"
echo "3. In another terminal, ingest the documents: npm run ingest-docs"
echo ""
echo "The Docusaurus site will be available at http://localhost:3000"
echo "The backend API will be available at http://localhost:8000"