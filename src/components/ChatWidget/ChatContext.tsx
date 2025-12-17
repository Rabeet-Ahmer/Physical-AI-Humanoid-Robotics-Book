import React, { createContext, useContext, useState, ReactNode } from 'react';
import useLocalStorage from '../../hooks/useLocalStorage'; // Adjust path as needed

// Define interfaces for ChatMessage and ChatUIState
export type ChatSender = 'user' | 'chatbot';

export interface ChatMessage {
  text: string;
  sender: ChatSender;
  timestamp: number; // Add timestamp for ordering/display
}

export interface ChatUIState {
  isOpen: boolean;
  history: ChatMessage[];
}

// Define the shape of the ChatContext
interface ChatContextType {
  chatState: ChatUIState;
  toggleChat: () => void;
  addMessage: (message: ChatMessage) => void;
  clearHistory: () => void;
  setChatState: React.Dispatch<React.SetStateAction<ChatUIState>>;
}

const ChatContext = createContext<ChatContextType | undefined>(undefined);

interface ChatProviderProps {
  children: ReactNode;
}

export const ChatProvider: React.FC<ChatProviderProps> = ({ children }) => {
  // Use useLocalStorage to persist the chat state
  const [chatState, setChatState] = useLocalStorage<ChatUIState>('docusaurus-chat-state', {
    isOpen: false,
    history: [],
  });

  const toggleChat = () => {
    setChatState((prevState) => ({ ...prevState, isOpen: !prevState.isOpen }));
  };

  const addMessage = (message: ChatMessage) => {
    setChatState((prevState) => ({
      ...prevState,
      history: [...prevState.history, { ...message, timestamp: Date.now() }],
    }));
  };

  const clearHistory = () => {
    setChatState((prevState) => ({ ...prevState, history: [] }));
  };

  return (
    <ChatContext.Provider value={{ chatState, toggleChat, addMessage, clearHistory, setChatState }}>
      {children}
    </ChatContext.Provider>
  );
};

export const useChat = () => {
  const context = useContext(ChatContext);
  if (context === undefined) {
    throw new Error('useChat must be used within a ChatProvider');
  }
  return context;
};
