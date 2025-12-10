// frontend/src/data/searchIndex.ts

import { bookData } from './bookContent';

export interface SearchResult {
  chapterId: number;
  chapterTitle: string;
  page: number;
  snippet: string;
  relevance: number;
  matchType: 'title' | 'keyword' | 'content';
}

/**
 * Search through all book content
 * @param query - Search term
 * @param maxResults - Maximum number of results to return
 * @returns Array of search results sorted by relevance
 */
export const searchBook = (query: string, maxResults: number = 10): SearchResult[] => {
  if (!query || query.trim().length < 2) {
    return [];
  }

  const searchTerm = query.toLowerCase().trim();
  const results: SearchResult[] = [];

  bookData.forEach((chapter) => {
    let relevance = 0;
    let matchType: 'title' | 'keyword' | 'content' = 'content';

    // Check title (highest priority)
    if (chapter.title.toLowerCase().includes(searchTerm)) {
      relevance += 100;
      matchType = 'title';
    }

    // Check keywords (high priority)
    const keywordMatch = chapter.keywords.some(keyword => 
      keyword.toLowerCase().includes(searchTerm)
    );
    if (keywordMatch) {
      relevance += 50;
      if (matchType === 'content') matchType = 'keyword';
    }

    // Check content (lower priority)
    const contentLower = chapter.content.toLowerCase();
    if (contentLower.includes(searchTerm)) {
      // Count occurrences
      const occurrences = (contentLower.match(new RegExp(searchTerm, 'g')) || []).length;
      relevance += occurrences * 5;
    }

    // If we found a match, add to results
    if (relevance > 0) {
      // Extract snippet around the match
      const snippet = extractSnippet(chapter.content, searchTerm);
      
      results.push({
        chapterId: chapter.id,
        chapterTitle: chapter.title,
        page: chapter.startPage,
        snippet,
        relevance,
        matchType
      });
    }
  });

  // Sort by relevance (highest first)
  results.sort((a, b) => b.relevance - a.relevance);

  // Return top results
  return results.slice(0, maxResults);
};

/**
 * Extract a snippet of text around the search term
 */
const extractSnippet = (content: string, searchTerm: string, contextLength: number = 100): string => {
  const lowerContent = content.toLowerCase();
  const lowerTerm = searchTerm.toLowerCase();
  
  const index = lowerContent.indexOf(lowerTerm);
  
  if (index === -1) {
    // Return first part of content if no match found
    return content.substring(0, contextLength) + '...';
  }

  // Calculate start and end positions
  const start = Math.max(0, index - contextLength / 2);
  const end = Math.min(content.length, index + searchTerm.length + contextLength / 2);

  let snippet = content.substring(start, end);

  // Clean up snippet (remove markdown symbols)
  snippet = snippet
    .replace(/#{1,6}\s/g, '') // Remove markdown headers
    .replace(/\*\*/g, '')      // Remove bold
    .replace(/\*/g, '')        // Remove italic
    .replace(/`/g, '')         // Remove code
    .replace(/\n+/g, ' ')      // Replace newlines with space
    .trim();

  // Add ellipsis
  if (start > 0) snippet = '...' + snippet;
  if (end < content.length) snippet = snippet + '...';

  return snippet;
};

/**
 * Get all unique keywords from the book
 */
export const getAllKeywords = (): string[] => {
  const keywordSet = new Set<string>();
  
  bookData.forEach(chapter => {
    chapter.keywords.forEach(keyword => keywordSet.add(keyword));
  });

  return Array.from(keywordSet).sort();
};

/**
 * Get suggestions based on partial input
 */
export const getSuggestions = (input: string, maxSuggestions: number = 5): string[] => {
  if (!input || input.length < 2) {
    return [];
  }

  const inputLower = input.toLowerCase();
  const suggestions = new Set<string>();

  // Add matching keywords
  bookData.forEach(chapter => {
    chapter.keywords.forEach(keyword => {
      if (keyword.toLowerCase().includes(inputLower)) {
        suggestions.add(keyword);
      }
    });

    // Add matching chapter titles
    if (chapter.title.toLowerCase().includes(inputLower)) {
      suggestions.add(chapter.title);
    }
  });

  return Array.from(suggestions).slice(0, maxSuggestions);
};

/**
 * Search by chapter number or page number
 */
export const searchByNumber = (num: number, searchType: 'chapter' | 'page'): SearchResult[] => {
  const results: SearchResult[] = [];

  bookData.forEach(chapter => {
    if (searchType === 'chapter' && chapter.id === num) {
      results.push({
        chapterId: chapter.id,
        chapterTitle: chapter.title,
        page: chapter.startPage,
        snippet: `Chapter ${chapter.id}: ${chapter.title}`,
        relevance: 100,
        matchType: 'title'
      });
    } else if (searchType === 'page' && num >= chapter.startPage && num <= chapter.endPage) {
      results.push({
        chapterId: chapter.id,
        chapterTitle: chapter.title,
        page: num,
        snippet: `Found in ${chapter.title} (Pages ${chapter.startPage}-${chapter.endPage})`,
        relevance: 100,
        matchType: 'title'
      });
    }
  });

  return results;
};

/**
 * Get popular search terms (predefined common topics)
 */
export const popularSearches = [
  'ROS2',
  'URDF',
  'Gazebo',
  'Reinforcement Learning',
  'Humanoid',
  'Kinematics',
  'Navigation',
  'Computer Vision',
  'Isaac Sim',
  'Linux',
  'Simulation',
  'Ethics'
];

/**
 * Search across specific part of the book
 */
export const searchByPart = (query: string, part: 1 | 2 | 3): SearchResult[] => {
  const allResults = searchBook(query, 100);
  
  return allResults.filter(result => {
    const chapter = bookData.find(ch => ch.id === result.chapterId);
    return chapter?.part === part;
  });
};