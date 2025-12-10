// frontend/src/components/Chat/ChatWidget.tsx

import React, { useState, useRef, useEffect } from 'react';
import { MessageCircle, X, Send, Loader2, BookOpen, Sparkles } from 'lucide-react';
import { chatAPI } from '../../services/api.ts';
import type { ChatMessage } from '../../types/chat.types';

// API Response types
interface HybridChatResponse {
  response: string;
  source: 'book' | 'general_knowledge';
  chapters?: number[];
}

// Extended chat message type with source tracking
interface ExtendedChatMessage extends ChatMessage {
  source?: 'book' | 'general_knowledge';
  chapters?: number[];
}

export const ChatWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState<boolean>(false);
  const [messages, setMessages] = useState<ExtendedChatMessage[]>([
    {
      id: '1',
      text: "Hello! I'm your AI assistant for Physical AI & Humanoid Robotics by Azmat Ali. I search the book first, and give short answers for general questions. Ask me anything!",
      isUser: false,
      timestamp: new Date(),
      source: 'book',
    },
  ]);
  const [input, setInput] = useState<string>('');
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = (): void => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSend = async (): Promise<void> => {
    if (!input.trim() || isLoading) return;

    const userMessage: ExtendedChatMessage = {
      id: Date.now().toString(),
      text: input,
      isUser: true,
      timestamp: new Date(),
    };

    setMessages((prev) => [...prev, userMessage]);
    const currentInput = input;
    setInput('');
    setIsLoading(true);

    try {
      // Call backend API - it can return either string or object
      const response = await chatAPI.sendMessage(currentInput) as string | HybridChatResponse;

      let botMessage: ExtendedChatMessage;

      // Check if response is the new hybrid format (object) or old format (string)
      if (typeof response === 'object' && response !== null && 'response' in response) {
        // New hybrid backend response
        botMessage = {
          id: (Date.now() + 1).toString(),
          text: response.response,
          isUser: false,
          timestamp: new Date(),
          source: response.source,
          chapters: response.chapters,
        };
      } else {
        // Old backend response (just string)
        botMessage = {
          id: (Date.now() + 1).toString(),
          text: response as string,
          isUser: false,
          timestamp: new Date(),
        };
      }
      
      setMessages((prev) => [...prev, botMessage]);
    } catch (err) {
      // Error handling
      const errorMessage: ExtendedChatMessage = {
        id: (Date.now() + 1).toString(),
        text: 'Sorry, I encountered an error. Please make sure the backend server is running on port 8000.',
        isUser: false,
        timestamp: new Date(),
      };
      setMessages((prev) => [...prev, errorMessage]);
      console.error('Chat API Error:', err);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent<HTMLInputElement>): void => {
    if (e.key === 'Enter' && !isLoading) {
      handleSend();
    }
  };

  const toggleChat = (): void => {
    setIsOpen(!isOpen);
  };

  return (
    <>
      {/* FLOATING BUTTON */}
      <button
        onClick={toggleChat}
        className="fixed bg-primary hover:bg-primary-dark text-white rounded-full 
                   shadow-lg transition-all duration-300 hover:scale-110 z-[999]
                   flex items-center justify-center
                   w-11 h-11 bottom-3 right-3
                   xs:w-12 xs:h-12 xs:bottom-3.5 xs:right-3.5
                   sm:w-[52px] sm:h-[52px] sm:bottom-4 sm:right-4
                   md:w-14 md:h-14 md:bottom-5 md:right-5
                   lg:w-[60px] lg:h-[60px] lg:bottom-6 lg:right-6"
        aria-label={isOpen ? 'Close chat' : 'Open chat'}
      >
        {isOpen ? (
          <X className="w-5 h-5 sm:w-6 sm:h-6 md:w-7 md:h-7" />
        ) : (
          <MessageCircle className="w-5 h-5 sm:w-6 sm:h-6 md:w-7 md:h-7" />
        )}
      </button>

      {/* CHAT WINDOW */}
      {isOpen && (
        <div 
          className="fixed bg-white rounded-lg shadow-xl border-2 border-primary/20 
                     flex flex-col z-[998] animate-slide-up
                     w-[calc(100vw-16px)] h-[420px] bottom-[56px] right-2
                     xs:w-[calc(100vw-20px)] xs:h-[450px] xs:bottom-[60px] xs:right-2.5
                     sm:w-[calc(100vw-24px)] sm:max-w-[360px] sm:h-[480px] sm:bottom-[68px] sm:right-4
                     md:w-[380px] md:h-[520px] md:bottom-[84px] md:right-5
                     lg:w-96 lg:h-[500px] lg:bottom-24 lg:right-6"
        >
          {/* HEADER */}
          <div className="bg-primary text-white rounded-t-lg p-3 sm:p-4">
            <h3 className="font-bold text-base sm:text-lg">AI Assistant</h3>
            <p className="text-xs sm:text-sm text-white/80">📚 Searches book first</p>
          </div>

          {/* MESSAGES CONTAINER */}
          <div className="flex-1 overflow-y-auto p-3 sm:p-4 space-y-3 sm:space-y-4 bg-gray-50">
            {messages.map((msg) => (
              <div
                key={msg.id}
                className={`flex ${msg.isUser ? 'justify-end' : 'justify-start'}`}
              >
                <div
                  className={`max-w-[85%] p-2.5 sm:p-3 rounded-lg ${
                    msg.isUser
                      ? 'bg-primary text-white rounded-br-none'
                      : 'bg-white text-gray-800 border border-gray-200 rounded-bl-none'
                  }`}
                >
                  <p className="text-sm sm:text-base whitespace-pre-wrap break-words leading-relaxed">
                    {msg.text}
                  </p>
                  
                  {/* Source Badge - Only for bot messages */}
                  {!msg.isUser && msg.source && (
                    <div className="mt-2 flex items-center gap-1.5">
                      {msg.source === 'book' ? (
                        <>
                          <BookOpen className="w-3 h-3 sm:w-3.5 sm:h-3.5 text-primary" />
                          <span className="text-xs text-primary font-medium">From Book</span>
                          {msg.chapters && msg.chapters.length > 0 && (
                            <span className="text-xs text-gray-600">
                              • Ch. {msg.chapters.join(', ')}
                            </span>
                          )}
                        </>
                      ) : msg.source === 'general_knowledge' ? (
                        <>
                          <Sparkles className="w-3 h-3 sm:w-3.5 sm:h-3.5 text-purple-500" />
                          <span className="text-xs text-purple-600 font-medium">General Knowledge</span>
                        </>
                      ) : null}
                    </div>
                  )}
                  
                  <span className="text-[10px] sm:text-xs opacity-70 mt-1.5 block">
                    {msg.timestamp.toLocaleTimeString('en-US', { 
                      hour: '2-digit', 
                      minute: '2-digit' 
                    })}
                  </span>
                </div>
              </div>
            ))}
            
            {/* Loading Indicator */}
            {isLoading && (
              <div className="flex justify-start">
                <div className="bg-white border border-gray-200 p-2.5 sm:p-3 rounded-lg 
                                flex items-center space-x-2">
                  <Loader2 className="w-3.5 h-3.5 sm:w-4 sm:h-4 text-primary animate-spin" />
                  <span className="text-xs sm:text-sm text-gray-600">AI is thinking...</span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* INPUT AREA */}
          <div className="p-2.5 sm:p-4 border-t border-gray-200 bg-white rounded-b-lg">
            <div className="flex space-x-2">
              <input
                type="text"
                value={input}
                onChange={(e: React.ChangeEvent<HTMLInputElement>) => setInput(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Ask about the book..."
                disabled={isLoading}
                className="flex-1 px-2.5 py-2 sm:px-3 sm:py-2 border border-gray-300 rounded-lg 
                           text-xs sm:text-sm
                           focus:outline-none focus:border-primary focus:ring-2 focus:ring-primary/20
                           disabled:bg-gray-100 disabled:cursor-not-allowed
                           placeholder:text-xs sm:placeholder:text-sm"
              />
              <button
                onClick={handleSend}
                disabled={isLoading || !input.trim()}
                className="bg-primary text-white rounded-lg hover:bg-primary-dark
                           disabled:opacity-50 disabled:cursor-not-allowed 
                           transition-colors duration-200
                           p-2 sm:p-2.5 flex items-center justify-center
                           min-w-[36px] sm:min-w-[40px]"
                aria-label="Send message"
              >
                {isLoading ? (
                  <Loader2 className="w-4 h-4 sm:w-5 sm:h-5 animate-spin" />
                ) : (
                  <Send className="w-4 h-4 sm:w-5 sm:h-5" />
                )}
              </button>
            </div>
          </div>
        </div>
      )}
    </>
  );
};