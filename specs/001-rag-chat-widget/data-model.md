# Data Model: Docusaurus RAG Chat Widget

**Feature**: Docusaurus RAG Chat Widget

## Entities

### ChatMessage

Represents a single message in the chat conversation, indicating the content and who sent it.

| Field | Type | Required | Description | Constraints |
|-------|------|----------|-------------|-------------|
| text  | string | Yes      | The content of the message. | Non-empty string.   |
| sender | 'user' \| 'chatbot' | Yes | Indicates if the message was sent by the user or the chatbot. | Must be either 'user' or 'chatbot'. |

### ChatUIState

Represents the state of the chat widget, including its visibility and the conversation history.

| Field | Type | Required | Description | Constraints |
|-------|------|----------|-------------|-------------|
| isOpen | boolean | Yes      | True if the chat panel is visible, false otherwise. | - |
| history | ChatMessage[] | Yes | An ordered list of chat messages, forming the conversation. | Can be empty. |

## TypeScript Interfaces (Reference)

```typescript
export type ChatSender = 'user' | 'chatbot';

export interface ChatMessage {
  text: string;
  sender: ChatSender;
}

export interface ChatUIState {
  isOpen: boolean;
  history: ChatMessage[];
}
```
