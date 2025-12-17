// src/utils/api.ts

// Define the API endpoint for the RAG agent
const API_ENDPOINT = 'http://127.0.0.1:8000/agent'; // As per quickstart.md and contracts/README.md

interface ChatApiRequest {
  user: string;
}

interface ChatApiResponse {
  assistant: string;
}

/**
 * Calls the backend RAG agent API to get a chatbot response.
 * @param message The user's message to send to the chatbot.
 * @returns A promise that resolves to the chatbot's response text.
 * @throws An error if the API call fails or returns an invalid response.
 */
export async function callChatApi(message: string): Promise<string> {
  try {
    const requestBody: ChatApiRequest = { user: message };

    const response = await fetch(API_ENDPOINT, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(requestBody),
    });

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
