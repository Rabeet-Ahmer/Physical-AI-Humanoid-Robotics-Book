import React from 'react';
import { useChat } from './ChatContext';
import styles from './ChatIcon.module.css'; // Assuming a CSS module for styling

const ChatIcon: React.FC = () => {
  const { toggleChat, chatState } = useChat();

  return (
    <button
      className={styles.chatIconButton}
      onClick={toggleChat}
      aria-label={chatState.isOpen ? 'Close chat widget' : 'Open chat widget'}
    >
      {/* You can replace this with an SVG icon for a chatbot */}
      <span role="img" aria-label="chatbot icon">💬</span>
    </button>
  );
};

export default ChatIcon;
