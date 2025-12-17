# Quickstart: Docusaurus RAG Chat Widget

## Prerequisites

- Node.js (v18 or higher recommended)
- npm or yarn
- The backend RAG Agent API (from feature `001-create-api-endpoints`) running and accessible at `http://127.0.0.1:8000` (or configured URL).

## Running the Docusaurus Site

1.  **Navigate to the project root**:
    ```bash
    cd <project-root>
    ```

2.  **Install frontend dependencies**:
    ```bash
    npm install  # or yarn install
    ```

3.  **Start the Docusaurus development server**:
    ```bash
    npm run start # or yarn start
    ```

    The Docusaurus site will typically open in your browser at `http://localhost:3000`.

## Interacting with the Chat Widget

1.  **Open the Docusaurus site**: Ensure the Docusaurus development server is running (`npm run start`).
2.  **Locate the Chatbot Icon**: On any page, you should see a persistent chatbot icon fixed to the bottom-right corner of the viewport.
3.  **Open Chat UI**: Click the chatbot icon. A small chat panel should appear.
4.  **Send a Query**: Type a message into the input field within the chat panel and press Enter.
5.  **Observe Response**: The chatbot's response (from the backend RAG API) should appear in the chat history.
6.  **Close Chat UI**: Click the close button within the chat panel to dismiss it.
