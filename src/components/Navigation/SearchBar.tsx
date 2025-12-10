import React, { useState } from 'react';
import { Search } from 'lucide-react';

interface SearchBarProps {
  onSearch: (query: string) => void;
  onPageJump: (pageNumber: number) => void;
}

export const SearchBar: React.FC<SearchBarProps> = ({ onSearch, onPageJump }) => {
  const [query, setQuery] = useState('');

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    
    const pageNum = parseInt(query);
    if (!isNaN(pageNum)) {
      onPageJump(pageNum);
    } else {
      onSearch(query);
    }
  };

  return (
    <form onSubmit={handleSubmit} className="w-full max-w-xl mx-auto">
      <div className="flex items-center gap-2 bg-white rounded-xl shadow-green-md border-2 border-primary/30 p-2 hover:border-primary transition-all duration-300">
        <Search className="ml-2 text-primary w-5 h-5 flex-shrink-0" />
        <input
          type="text"
          value={query}
          onChange={(e) => setQuery(e.target.value)}
          placeholder="Search by page number or content..."
          className="flex-1 px-2 py-2 focus:outline-none text-gray-700"
        />
        <button
          type="submit"
          className="px-6 py-2 bg-gradient-to-r from-primary to-primary-dark text-white rounded-lg 
                     hover:shadow-green-md transition-all duration-300 font-bold text-sm
                     transform hover:scale-105 active:scale-95 flex-shrink-0"
        >
          Go
        </button>
      </div>
    </form>
  );
};