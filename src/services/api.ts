// frontend/src/services/api.ts

const API_BASE_URL = 'http://localhost:8000';

interface ChatResponse {
  response: string;
  source: string;
  chapters: number[];
}

interface HealthResponse {
  status: string;
  book_chapters_loaded?: number;
  gemini_configured?: boolean;
}

export const chatAPI = {
  // Send message to AI
  async sendMessage(message: string): Promise<ChatResponse> {
    try {
      const response = await fetch(`${API_BASE_URL}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ message })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data: ChatResponse = await response.json();
      return data;
    } catch (error) {
      console.error('Error calling chat API:', error);
      throw error;
    }
  },

  // Check backend health
  async checkHealth(): Promise<HealthResponse> {
    try {
      const response = await fetch(`${API_BASE_URL}/health`);
      const data: HealthResponse = await response.json();
      return data;
    } catch (error) {
      console.error('Backend health check failed:', error);
      return { status: 'unhealthy' };
    }
  }
};