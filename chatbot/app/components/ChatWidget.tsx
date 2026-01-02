'use client';

import { useState, useEffect, useRef } from 'react';

interface Citation {
  source_id: string;
  text?: string;
}

interface Message {
  id: string;
  role: 'user' | 'bot';
  content: string;
  timestamp: number;
  citations?: Citation[];
  isVerified?: boolean;
}

interface ChatWidgetProps {
  activeSection: string;
}

const API_BASE = process.env.NEXT_PUBLIC_API_BASE || 'https://backend-physical-ai-and-humanoid-robotics-production.up.railway.app';

export default function ChatWidget({ activeSection }: ChatWidgetProps) {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [currentSessionId, setCurrentSessionId] = useState<string | null>(null);
  const [retryMessage, setRetryMessage] = useState<string | null>(null);
  const [selection, setSelection] = useState<{ text: string; x: number; y: number } | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Initialize session and history on mount
  useEffect(() => {
    const startSession = async () => {
      try {
        const response = await fetch(`${API_BASE}/chat/start`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' }
        });
        if (response.ok) {
          const data = await response.json();
          setCurrentSessionId(data.session_id);
          fetchHistory(data.session_id);
        }
      } catch (error) {
        console.error('Failed to start chat session:', error);
      }
    };

    const fetchHistory = async (sid: string) => {
      try {
        const response = await fetch(`${API_BASE}/chat/history/${sid}`);
        if (response.ok) {
          const data = await response.json();
          // Assuming data is an array of messages or similar
          if (Array.isArray(data) && data.length > 0) {
            setMessages(data.map((m: any, idx: number) => ({
              id: `hist-${idx}`,
              role: m.role === 'assistant' ? 'bot' : 'user',
              content: m.content || m.answer || m.question,
              timestamp: Date.now(),
              citations: m.citations,
              isVerified: m.is_from_book
            })));
          } else {
            // Show welcome message if no history
            setMessages([{
              id: 'welcome',
              role: 'bot',
              content: 'Hi! I\'m your JS Guide. How can I help you today?',
              timestamp: Date.now()
            }]);
          }
        }
      } catch (e) {
        console.warn('Could not fetch remote history, using local if available');
        // Show welcome message if history fetch fails
        setMessages([{
          id: 'welcome',
          role: 'bot',
          content: 'Hi! I\'m your JS Guide. How can I help you today?',
          timestamp: Date.now()
        }]);
      }
    };

    startSession();

    // Text selection handler
    const handleMouseUp = () => {
      const selectedText = window.getSelection()?.toString().trim();
      if (selectedText && selectedText.length > 5) {
        const range = window.getSelection()?.getRangeAt(0);
        const rect = range?.getBoundingClientRect();
        if (rect) {
          setSelection({
            text: selectedText,
            x: rect.left + window.scrollX,
            y: rect.top + window.scrollY - 40
          });
        }
      } else {
        setSelection(null);
      }
    };

    document.addEventListener('mouseup', handleMouseUp);
    return () => document.removeEventListener('mouseup', handleMouseUp);
  }, []);

  // Save history to localStorage as fallback
  useEffect(() => {
    if (messages.length > 0) {
      localStorage.setItem('chat_history', JSON.stringify(messages));
    }
  }, [messages]);

  // Scroll to bottom
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const handleSend = async (overrideText?: string) => {
    const text = overrideText || inputValue;
    if (!text.trim() || isLoading) return;

    let sessionId = currentSessionId;
    if (!sessionId) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: text,
      timestamp: Date.now()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setRetryMessage(null);

    const callApi = async (attempt = 1): Promise<void> => {
      try {
        console.log('Sending request to:', `${API_BASE}/chat/send/${sessionId}`);
        console.log('Request body:', { question: text });

        const response = await fetch(`${API_BASE}/chat/send/${sessionId}`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ question: text })
        });

        console.log('Response status:', response.status);
        if (!response.ok) {
          const errorText = await response.text();
          console.error('API Error:', errorText);
          throw new Error(`HTTP error ${response.status}`);
        }

        const data = await response.json();
        console.log('API Response:', data);

        const botMessage: Message = {
          id: (Date.now() + 1).toString(),
          role: 'bot',
          content: data.answer || 'No response from AI.',
          timestamp: Date.now(),
          citations: data.citations,
          isVerified: data.is_from_book
        };
        setMessages(prev => [...prev, botMessage]);
        setRetryMessage(null);
      } catch (error) {
        console.error('Chat Error:', error);
        setMessages(prev => [...prev, {
          id: Date.now().toString(),
          role: 'bot',
          content: 'Sorry, I hit a snag. Please check your connection.',
          timestamp: Date.now()
        }]);
      } finally {
        setIsLoading(false);
      }
    };

    await callApi();
  };

  const handleContextQuery = () => {
    if (selection) {
      const contextText = `Context: [${selection.text}]\n\nQuestion: `;
      setInputValue(contextText);
      setIsOpen(true);
      setSelection(null);
      // Focus input
      setTimeout(() => {
        const input = document.querySelector('.chat-input') as HTMLInputElement;
        input?.focus();
      }, 300);
    }
  };

  const clearChat = async () => {
    localStorage.removeItem('chat_history');
    setMessages([]);
    const response = await fetch(`${API_BASE}/chat/start`, { method: 'POST' });
    const data = await response.json();
    setCurrentSessionId(data.session_id);
    setMessages([{ id: 'welcome', role: 'bot', content: 'History cleared. New session started!', timestamp: Date.now() }]);
  };

  return (
    <>
      {/* Floating Action Button for Selection */}
      {selection && (
        <button
          className="context-btn"
          style={{
            position: 'absolute',
            left: selection.x,
            top: selection.y,
            zIndex: 2000,
            background: 'var(--primary)',
            color: 'white',
            border: 'none',
            borderRadius: '20px',
            padding: '5px 12px',
            fontSize: '0.8rem',
            cursor: 'pointer',
            boxShadow: '0 4px 10px rgba(0,0,0,0.3)'
          }}
          onMouseDown={(e) => {
            e.preventDefault(); // Prevent losing selection
            handleContextQuery();
          }}
        >
          Ask about this ✨
        </button>
      )}

      <button
        className="chat-toggle-btn"
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle chat"
      >
        {isOpen ? (
        <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" className="w-5 h-5">
          <line x1="18" y1="6" x2="6" y2="18" />
          <line x1="6" y1="6" x2="18" y2="18" />
        </svg>
      ) : (
        <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" className="w-5 h-5">
          <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
        </svg>
      )}
      </button>

      <div className={`chat-container ${isOpen ? 'open' : ''}`}>
        <div className="chat-header">
          <div className="chat-header-title">
            <div className="chat-avatar">JS</div>
            <div className="chat-header-text">
              <h3>JS Guide</h3>
              <span>{currentSessionId ? 'Online' : 'Connecting...'}</span>
            </div>
          </div>
          <div style={{ display: 'flex', gap: '8px' }}>
            <button className="chat-new-btn" onClick={clearChat} title="New Chat">
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M21 12a9 9 0 1 1-9-9c2.52 0 4.93 1 6.74 2.74L21 8"></path>
                <polyline points="21 3 21 8 16 8"></polyline>
              </svg>
            </button>
            <button className="chat-close-btn" onClick={() => setIsOpen(false)}>✕</button>
          </div>
        </div>

        <div className="chat-messages">
          {messages.map(msg => (
            <div key={msg.id} className={`message ${msg.role}`}>
              <div className="message-content" style={{ whiteSpace: 'pre-wrap' }}>
                {msg.content}
                {msg.isVerified && <span className="verified-badge">✓</span>}
              </div>
              {msg.citations && msg.citations.length > 0 && (
                <div className="citations">
                  {msg.citations.map((cit, idx) => (
                    <span key={idx} className="citation-tag">[{cit.source_id}]</span>
                  ))}
                </div>
              )}
            </div>
          ))}
          {retryMessage && (
            <div className="message bot" style={{ opacity: 0.7, fontSize: '0.8rem', fontStyle: 'italic' }}>
              {retryMessage}
            </div>
          )}
          {isLoading && !retryMessage && (
            <div className="message bot loading">
              <span></span><span></span><span></span>
            </div>
          )}
          <div ref={messagesEndRef} />
        </div>

        <div className="chat-input-container">
          <input
            type="text"
            className="chat-input"
            placeholder="Ask a question..."
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyPress={(e) => e.key === 'Enter' && handleSend()}
            disabled={isLoading}
          />
          <button
            className="chat-send-btn"
            onClick={() => handleSend()}
            disabled={isLoading || !inputValue.trim()}
          >
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5">
              <line x1="22" y1="2" x2="11" y2="13"></line>
              <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
            </svg>
          </button>
        </div>
      </div>
    </>
  );
}
