import React, { useRef, useEffect, useState } from 'react';
import { useChat, ChatMessage as TChatMessage } from './ChatContext';
import ChatMessage from './ChatMessage';
import { callChatApi } from '../../utils/api'; // Import the API utility
import styles from './ChatPanel.module.css';

const ChatPanel: React.FC = () => {
  const { chatState, addMessage, toggleChat } = useChat();
  const [inputMessage, setInputMessage] = useState<string>('');
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [chatState.history]);

  const handleSendMessage = async (e: React.FormEvent) => {
    e.preventDefault();
    if (inputMessage.trim() && !isLoading) {
      const userMessage: TChatMessage = {
        text: inputMessage.trim(),
        sender: 'user',
        timestamp: Date.now(),
      };
      addMessage(userMessage);
      setInputMessage('');
      setIsLoading(true);

      try {
        const botResponseText = await callChatApi(userMessage.text);
        const botMessage: TChatMessage = {
          text: botResponseText,
          sender: 'chatbot',
          timestamp: Date.now(),
        };
        addMessage(botMessage);
      } catch (error) {
        console.error('Chat API call failed:', error);
        // Display an error message to the user
        const errorMessage: TChatMessage = {
          text: 'Error: Could not get a response from the chatbot. Please try again.',
          sender: 'chatbot',
          timestamp: Date.now(),
        };
        addMessage(errorMessage);
      } finally {
        setIsLoading(false);
      }
    }
  };

  if (!chatState.isOpen) {
    return null;
  }

  return (
    <div className={styles.chatPanelContainer} aria-live="polite" aria-relevant="additions text">
      <div className={styles.chatPanelHeader}>
        <h3>AI Chatbot</h3>
        <button onClick={toggleChat} className={styles.closeButton} aria-label="Close chat widget">
          ✕
        </button>
      </div>
      <div className={styles.chatPanelBody}>
        {chatState.history.length === 0 ? (
          <p className={styles.noMessages}>Start a conversation!</p>
        ) : (
          chatState.history.map((message, index) => (
            <ChatMessage key={index} message={message} />
          ))
        )}
        {isLoading && <p className={styles.loadingIndicator}>Thinking...</p>}
        <div ref={messagesEndRef} />
      </div>
      <form className={styles.chatPanelFooter} onSubmit={handleSendMessage}>
        <input
          type="text"
          placeholder="Type your message..."
          className={styles.chatInput}
          value={inputMessage}
          onChange={(e) => setInputMessage(e.target.value)}
          aria-label="Chat input"
          disabled={isLoading}
        />
        <button type="submit" className={styles.sendButton} aria-label="Send message" disabled={isLoading}>
          Send
        </button>
      </form>
    </div>
  );
};

export default ChatPanel;
