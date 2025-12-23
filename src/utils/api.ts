// src/utils/api.ts

// Define the API endpoint for the RAG agent
// Safely check for process.env to avoid ReferenceError in browser
const isDevelopment = typeof process !== "undefined" && process.env && process.env.NODE_ENV === "development";
const envApiUrl = typeof process !== "undefined" && process.env ? process.env.NEXT_PUBLIC_RAG_API_URL : undefined;

const API_ENDPOINT = envApiUrl || (isDevelopment ? "http://127.0.0.1:8000/agent" : "http://127.0.0.1:8000/agent");
// Note: In production, if NEXT_PUBLIC_RAG_API_URL is not set, it defaults to localhost which will fail. 
// You should ensure the environment variable is set or update this default to your production backend URL.

interface ChatApiRequest {
  user: string;
  session_id: string; // Add session_id
}

interface ChatApiResponse {
  assistant: string;
}

/**
 * Calls the backend RAG agent API to get a chatbot response.
 * @param message The user's message to send to the chatbot.
 * @param sessionId The unique ID for the current chat session.
 * @returns A promise that resolves to the chatbot's response text.
 * @throws An error if the API call fails or returns an invalid response.
 */
export async function callChatApi(message: string, sessionId: string): Promise<string> {
  try {
    const requestBody: ChatApiRequest = { user: message, session_id: sessionId }; // Include session_id

    const response = await fetch(API_ENDPOINT, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      credentials: 'include', // Send cookies (Better Auth session)
      body: JSON.stringify(requestBody),
    });

    if (response.status === 401) {
       // Redirect to login or throw specific error
       window.location.href = '/sign-in';
       throw new Error('Unauthorized: Please log in.');
    }

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({ message: 'Unknown error' }));
      throw new Error(`API call failed: ${response.status} ${response.statusText} - ${errorData.message}`);
    }

    const data: ChatApiResponse = await response.json();

    if (!data.assistant) {
      throw new Error('Invalid API response: "assistant" field missing.');
    }

    return data.assistant;
  } catch (error) {
    console.error('Error in callChatApi:', error);
    throw error; // Re-throw to be handled by the caller
  }
}
