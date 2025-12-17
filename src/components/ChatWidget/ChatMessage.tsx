import React from 'react';
import { ChatMessage as TChatMessage, ChatSender } from './ChatContext';
import styles from './ChatMessage.module.css';

interface ChatMessageProps {
  message: TChatMessage;
}

const ChatMessage: React.FC<ChatMessageProps> = ({ message }) => {
  const isUser = message.sender === 'user';
  const messageClass = isUser ? styles.userMessage : styles.chatbotMessage;
  const senderName = isUser ? 'You' : 'Chatbot';

  // Format timestamp for display
  const date = new Date(message.timestamp);
  const timeString = date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });

  return (
    <div className={`${styles.chatMessageContainer} ${messageClass}`}>
      <div className={styles.messageBubble}>
        <div className={styles.messageSender}>{senderName}</div>
        <div className={styles.messageText}>{message.text}</div>
        <div className={styles.messageTimestamp}>{timeString}</div>
      </div>
    </div>
  );
};

export default ChatMessage;
