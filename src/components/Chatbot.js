import React, { useState, useEffect, useRef } from 'react';
import './Chatbot.css';

const Chatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [sessionId, setSessionId] = useState(null);
  const messagesEndRef = useRef(null);
  const chatContainerRef = useRef(null);

  // Function to get selected text from the page
  const getSelectedText = () => {
    const selection = window.getSelection();
    const selectedText = selection.toString().trim();
    if (selectedText.length > 0) {
      setSelectedText(selectedText);
      return selectedText;
    }
    return '';
  };

  // Function to handle text selection
  useEffect(() => {
    const handleSelection = () => {
      const text = getSelectedText();
      // Optionally notify user that text has been selected for RAG
      if (text) {
        console.log('Selected text for RAG:', text);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  // Scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen && chatContainerRef.current) {
      // Focus the input when opening the chat
      setTimeout(() => {
        const input = chatContainerRef.current.querySelector('input');
        if (input) input.focus();
      }, 100);
    }
  };

  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = { role: 'user', content: inputValue, timestamp: new Date() };
    const newMessages = [...messages, userMessage];
    setMessages(newMessages);
    setInputValue('');
    setIsLoading(true);

    try {
      // Get current selected text if any
      const currentSelectedText = getSelectedText();

      // Prepare the request payload
      const requestBody = {
        message: inputValue,
        session_id: sessionId,
        selected_text: currentSelectedText || null,
        history: newMessages.filter(msg => msg.role !== 'assistant').slice(-5) // Send last 5 user messages
      };

      // Call the backend API
      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      // Update session ID if new one was provided
      if (data.session_id && !sessionId) {
        setSessionId(data.session_id);
      }

      // Add the assistant's response
      const assistantMessage = {
        role: 'assistant',
        content: data.response,
        sources: data.sources || [],
        timestamp: new Date()
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = {
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again.',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const clearChat = () => {
    setMessages([]);
    setSessionId(null);
  };

  return (
    <div className="chatbot">
      {/* Floating button */}
      {!isOpen && (
        <button className="chatbot-button" onClick={toggleChat} aria-label="Open chat">
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M21 15C21 15.5304 20.7893 16.0391 20.4142 16.4142C20.0391 16.7893 19.5304 17 19 17H16.58L15.5 19.45C15.36 19.75 15.11 19.97 14.8 20.07C14.49 20.17 14.16 20.14 13.88 19.99L7.38 16.64C7.16 16.53 7 16.32 7 16.08V15H5C4.46957 15 3.96086 14.7893 3.58579 14.4142C3.21071 14.0391 3 13.5304 3 13V5C3 4.46957 3.21071 3.96086 3.58579 3.58579C3.96086 3.21071 4.46957 3 5 3H19C19.5304 3 20.0391 3.21071 20.4142 3.58579C20.7893 3.96086 21 4.46957 21 5V15Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
            <path d="M10 9H14M10 13H12" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          </svg>
        </button>
      )}

      {/* Chat window */}
      {isOpen && (
        <div className="chatbot-window" ref={chatContainerRef}>
          <div className="chatbot-header">
            <h3>AI Robotics Book Assistant</h3>
            <div className="chatbot-header-actions">
              <button onClick={clearChat} className="clear-chat-btn" title="Clear chat">
                <svg width="16" height="16" viewBox="0 0 16 16" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="M2 4H3.33333H14" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
                  <path d="M5.3335 4V2.66667C5.3335 2.31305 5.47402 1.97382 5.72408 1.72376C5.97414 1.4737 6.31337 1.33333 6.667 1.33333H9.3335C9.68713 1.33333 10.0264 1.4737 10.2764 1.72376C10.5265 1.97382 10.667 2.31305 10.667 2.66667V4M12.667 4V13.3333C12.667 13.687 12.5265 14.0262 12.2764 14.2763C12.0264 14.5263 11.6871 14.6667 11.3335 14.6667H4.667C4.31337 14.6667 3.97414 14.5263 3.72408 14.2763C3.47402 14.0262 3.3335 13.687 3.3335 13.3333V4H12.667Z" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
                </svg>
              </button>
              <button onClick={toggleChat} className="close-chat-btn" title="Close chat">
                <svg width="16" height="16" viewBox="0 0 16 16" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="M12 4L4 12M4 4L12 12" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
                </svg>
              </button>
            </div>
          </div>

          <div className="chatbot-messages">
            {messages.length === 0 ? (
              <div className="chatbot-welcome">
                <h4>Hello! I'm your AI Robotics Book Assistant.</h4>
                <p>Ask me anything about the content in this book, or select text on the page to ask specific questions about it.</p>
                {selectedText && (
                  <div className="selected-text-preview">
                    <p><strong>Selected text:</strong> "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"</p>
                  </div>
                )}
              </div>
            ) : (
              messages.map((message, index) => (
                <div
                  key={index}
                  className={`chatbot-message ${message.role === 'user' ? 'user-message' : 'assistant-message'}`}
                >
                  <div className="message-content">
                    {message.role === 'assistant' && message.sources && message.sources.length > 0 && (
                      <div className="message-sources">
                        {message.sources.slice(0, 2).map((source, idx) => (
                          <span key={idx} className="source-tag">
                            {source.title || source.page}
                          </span>
                        ))}
                      </div>
                    )}
                    <div className="message-text">
                      {message.content.split('\n').map((line, i) => (
                        <React.Fragment key={i}>
                          {line}
                          <br />
                        </React.Fragment>
                      ))}
                    </div>
                  </div>
                </div>
              ))
            )}
            {isLoading && (
              <div className="chatbot-message assistant-message">
                <div className="message-content">
                  <div className="typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className="chatbot-input-area">
            {selectedText && (
              <div className="selected-text-indicator">
                Using selected text: "{selectedText.substring(0, 50)}{selectedText.length > 50 ? '...' : ''}"
              </div>
            )}
            <div className="input-container">
              <textarea
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Ask about the book content..."
                rows="1"
                disabled={isLoading}
              />
              <button
                onClick={sendMessage}
                disabled={!inputValue.trim() || isLoading}
                className="send-button"
              >
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="M22 2L11 13M22 2L15 22L11 13M11 13L2 9L22 2" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                </svg>
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default Chatbot;