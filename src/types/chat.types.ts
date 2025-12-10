export interface ChatMessage {
  id: string;
  text: string;
  isUser: boolean;
  timestamp: Date;
  citations?: Citation[];
}

export interface Citation {
  page: number;
  chapter: number;
  snippet: string;
}

export interface ChatResponse {
  answer: string;
  citations: Citation[];
}