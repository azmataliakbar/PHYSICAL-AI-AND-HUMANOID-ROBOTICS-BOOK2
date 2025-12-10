// frontend/src/components/SearchBar.tsx

import React, { useState, useEffect, useRef } from 'react';
import { Search, X, Clock, TrendingUp } from 'lucide-react';
import { searchBook, getSuggestions, popularSearches } from '../data/searchIndex';
import type { SearchResult } from '../data/searchIndex';

interface SearchBarProps {
  onResultClick: (page: number) => void;
}

export const SearchBar: React.FC<SearchBarProps> = ({ onResultClick }) => {
  const [query, setQuery] = useState('');
  const [results, setResults] = useState<SearchResult[]>([]);
  const [suggestions, setSuggestions] = useState<string[]>([]);
  const [isOpen, setIsOpen] = useState(false);
  const [showSuggestions, setShowSuggestions] = useState(false);
  
  // Initialize recent searches from localStorage using lazy initialization
  const [recentSearches, setRecentSearches] = useState<string[]>(() => {
    try {
      const saved = localStorage.getItem('recentSearches');
      if (saved) {
        const parsed = JSON.parse(saved);
        return Array.isArray(parsed) ? parsed : [];
      }
    } catch (error) {
      console.error('Failed to load recent searches:', error);
    }
    return [];
  });
  
  const searchRef = useRef<HTMLDivElement>(null);

  // Close dropdown when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (searchRef.current && !searchRef.current.contains(event.target as Node)) {
        setIsOpen(false);
        setShowSuggestions(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, []);

  // Handle search
  const handleSearch = (searchQuery: string) => {
    setQuery(searchQuery);

    if (searchQuery.trim().length < 2) {
      setResults([]);
      setIsOpen(false);
      return;
    }

    const searchResults = searchBook(searchQuery, 10);
    setResults(searchResults);
    setIsOpen(true);
    setShowSuggestions(false);

    // Save to recent searches
    if (searchQuery.trim() && !recentSearches.includes(searchQuery)) {
      const updated = [searchQuery, ...recentSearches.slice(0, 4)];
      setRecentSearches(updated);
      localStorage.setItem('recentSearches', JSON.stringify(updated));
    }
  };

  // Handle input change with suggestions
  const handleInputChange = (value: string) => {
    setQuery(value);

    if (value.length >= 2) {
      const sug = getSuggestions(value, 5);
      setSuggestions(sug);
      setShowSuggestions(sug.length > 0);
      setIsOpen(false);
    } else {
      setSuggestions([]);
      setShowSuggestions(false);
      setResults([]);
      setIsOpen(false);
    }
  };

  // Handle suggestion click
  const handleSuggestionClick = (suggestion: string) => {
    setQuery(suggestion);
    handleSearch(suggestion);
  };

  // Handle result click
  const handleResultClick = (page: number) => {
    onResultClick(page);
    setIsOpen(false);
    setQuery('');
  };

  // Clear search
  const handleClear = () => {
    setQuery('');
    setResults([]);
    setSuggestions([]);
    setIsOpen(false);
    setShowSuggestions(false);
  };

  // Get relevance badge
  const getRelevanceBadge = (matchType: string) => {
    switch (matchType) {
      case 'title':
        return <span className="px-2 py-0.5 text-xs bg-green-100 text-green-700 rounded">Title Match</span>;
      case 'keyword':
        return <span className="px-2 py-0.5 text-xs bg-blue-100 text-blue-700 rounded">Keyword</span>;
      case 'content':
        return <span className="px-2 py-0.5 text-xs bg-gray-100 text-gray-700 rounded">Content</span>;
      default:
        return null;
    }
  };

  return (
    <div ref={searchRef} className="relative w-full max-w-2xl">
      {/* Search Input */}
      <div className="relative">
        <Search
          className="absolute left-4 top-1/2 transform -translate-y-1/2 text-gray-400"
          size={20}
        />
        <input
          type="text"
          value={query}
          onChange={(e) => handleInputChange(e.target.value)}
          onFocus={() => {
            if (query.length >= 2 && suggestions.length > 0) {
              setShowSuggestions(true);
            } else if (results.length > 0) {
              setIsOpen(true);
            }
          }}
          onKeyPress={(e) => {
            if (e.key === 'Enter' && query.trim()) {
              handleSearch(query);
            }
          }}
          placeholder="Search chapters, topics, or keywords..."
          className="w-full pl-12 pr-12 py-3 border-2 border-gray-300 rounded-lg
                   focus:outline-none focus:border-primary transition-colors
                   text-sm"
        />
        {query && (
          <button
            type="button"
            onClick={handleClear}
            className="absolute right-4 top-1/2 transform -translate-y-1/2 
                     text-gray-400 hover:text-gray-600 transition-colors"
            aria-label="Clear search"
          >
            <X size={20} />
          </button>
        )}
      </div>

      {/* Suggestions Dropdown */}
      {showSuggestions && suggestions.length > 0 && (
        <div className="absolute top-full mt-2 w-full bg-white border-2 border-gray-200 
                      rounded-lg shadow-lg z-50 max-h-96 overflow-y-auto">
          <div className="p-2">
            <div className="text-xs font-semibold text-gray-500 px-2 py-1 mb-1">
              Suggestions
            </div>
            {suggestions.map((suggestion, index) => (
              <button
                key={index}
                type="button"
                onClick={() => handleSuggestionClick(suggestion)}
                className="w-full text-left px-3 py-2 hover:bg-gray-100 rounded
                         transition-colors flex items-center gap-2"
                aria-label={`Search for ${suggestion}`}
              >
                <Search size={14} className="text-gray-400" />
                <span className="text-sm">{suggestion}</span>
              </button>
            ))}
          </div>
        </div>
      )}

      {/* Search Results */}
      {isOpen && results.length > 0 && (
        <div className="absolute top-full mt-2 w-full bg-white border-2 border-gray-200 
                      rounded-lg shadow-lg z-50 max-h-96 overflow-y-auto">
          <div className="p-2">
            <div className="text-xs font-semibold text-gray-500 px-2 py-1 mb-2 flex items-center justify-between">
              <span>Search Results ({results.length})</span>
              <button
                type="button"
                onClick={handleClear}
                className="text-primary hover:underline text-xs font-normal"
                aria-label="Clear search results"
              >
                Clear
              </button>
            </div>
            {results.map((result, index) => (
              <button
                key={index}
                type="button"
                onClick={() => handleResultClick(result.page)}
                className="w-full text-left p-3 hover:bg-gray-50 rounded-lg
                         transition-colors border-b border-gray-100 last:border-0"
                aria-label={`Go to ${result.chapterTitle} on page ${result.page}`}
              >
                <div className="flex items-start justify-between gap-2 mb-1">
                  <div className="font-semibold text-sm text-gray-900">
                    {result.chapterTitle}
                  </div>
                  {getRelevanceBadge(result.matchType)}
                </div>
                <div className="text-xs text-gray-600 mb-2 line-clamp-2">
                  {result.snippet}
                </div>
                <div className="text-xs text-gray-400">
                  Page {result.page}
                </div>
              </button>
            ))}
          </div>
        </div>
      )}

      {/* No Results */}
      {isOpen && query.length >= 2 && results.length === 0 && (
        <div className="absolute top-full mt-2 w-full bg-white border-2 border-gray-200 
                      rounded-lg shadow-lg z-50 p-4">
          <div className="text-center text-gray-500">
            <Search size={32} className="mx-auto mb-2 opacity-50" />
            <p className="text-sm font-semibold">No results found</p>
            <p className="text-xs mt-1">Try different keywords or browse the table of contents</p>
          </div>
        </div>
      )}

      {/* Popular Searches (show when input is focused but empty) */}
      {!query && isOpen && recentSearches.length === 0 && (
        <div className="absolute top-full mt-2 w-full bg-white border-2 border-gray-200 
                      rounded-lg shadow-lg z-50 p-3">
          <div className="text-xs font-semibold text-gray-500 px-2 py-1 mb-2 flex items-center gap-1">
            <TrendingUp size={14} />
            Popular Topics
          </div>
          <div className="flex flex-wrap gap-2 px-2">
            {popularSearches.map((topic, index) => (
              <button
                key={index}
                type="button"
                onClick={() => handleSuggestionClick(topic)}
                className="px-3 py-1.5 bg-gray-100 hover:bg-primary hover:text-white
                         text-xs rounded-full transition-colors"
                aria-label={`Search for ${topic}`}
              >
                {topic}
              </button>
            ))}
          </div>
        </div>
      )}

      {/* Recent Searches */}
      {!query && recentSearches.length > 0 && isOpen && (
        <div className="absolute top-full mt-2 w-full bg-white border-2 border-gray-200 
                      rounded-lg shadow-lg z-50 p-3">
          <div className="text-xs font-semibold text-gray-500 px-2 py-1 mb-2 flex items-center gap-1">
            <Clock size={14} />
            Recent Searches
          </div>
          {recentSearches.map((search, index) => (
            <button
              key={index}
              type="button"
              onClick={() => handleSuggestionClick(search)}
              className="w-full text-left px-3 py-2 hover:bg-gray-100 rounded
                       transition-colors flex items-center gap-2 text-sm"
              aria-label={`Search for ${search} again`}
            >
              <Clock size={14} className="text-gray-400" />
              {search}
            </button>
          ))}
        </div>
      )}
    </div>
  );
};