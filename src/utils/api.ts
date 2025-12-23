// src/utils/api.ts


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

    const response = await fetch("https://rabeet-ahmer-hackathon1-0.hf.space/agent", {
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
