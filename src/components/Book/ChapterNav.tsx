// frontend/src/components/ChapterNav.tsx

import { BookOpen, ChevronLeft, ChevronRight, Filter } from 'lucide-react';
import React, { useState } from 'react';
import { bookData, getChaptersByPart } from '../data/bookContent';

interface ChapterNavProps {
  currentPage: number;
  onPageChange: (page: number) => void;
  isOpen: boolean;
  onToggle: () => void;
}

export const ChapterNav: React.FC<ChapterNavProps> = ({
  currentPage,
  onPageChange,
  isOpen,
  onToggle
}) => {
  const [selectedPart, setSelectedPart] = useState<1 | 2 | 3 | 'all'>('all');

  // Get current chapter based on page
  const currentChapter = bookData.find(
    chapter => currentPage >= chapter.startPage && currentPage <= chapter.endPage
  );

  // Get chapters to display based on filter
  const displayChapters = selectedPart === 'all' 
    ? bookData 
    : getChaptersByPart(selectedPart);

  const getPartColor = (part: number) => {
    switch (part) {
      case 1: return 'border-green-500 bg-green-50 text-green-700';
      case 2: return 'border-yellow-500 bg-yellow-50 text-yellow-700';
      case 3: return 'border-red-500 bg-red-50 text-red-700';
      default: return 'border-gray-500 bg-gray-50 text-gray-700';
    }
  };

  const getPartBadge = (part: number) => {
    switch (part) {
      case 1: return { icon: '📘', label: 'Student', color: 'bg-green-500' };
      case 2: return { icon: '📗', label: 'Researcher', color: 'bg-yellow-500' };
      case 3: return { icon: '📕', label: 'Expert', color: 'bg-red-500' };
      default: return { icon: '📚', label: 'All', color: 'bg-gray-500' };
    }
  };

  return (
    <>
      {/* Toggle Button (when closed) */}
      {!isOpen && (
        <button
          type="button"
          onClick={onToggle}
          className="fixed left-0 top-1/2 transform -translate-y-1/2 
                   bg-primary text-white p-3 rounded-r-lg shadow-lg
                   hover:bg-primary-dark transition-all duration-300 z-40"
          aria-label="Open chapter navigation"
        >
          <ChevronRight size={20} />
        </button>
      )}

      {/* Sidebar */}
      <div
        className={`fixed left-0 top-0 h-full bg-white shadow-2xl z-50
                  transition-transform duration-300 ease-in-out
                  ${isOpen ? 'translate-x-0' : '-translate-x-full'}
                  w-80 flex flex-col`}
      >
        {/* Header */}
        <div className="p-4 border-b border-gray-200 bg-gradient-to-r from-primary to-primary-dark">
          <div className="flex items-center justify-between mb-3">
            <div className="flex items-center gap-2 text-white">
              <BookOpen size={24} />
              <h3 className="font-bold text-lg">Chapters</h3>
            </div>
            <button
              type="button"
              onClick={onToggle}
              className="text-white hover:bg-white/20 p-2 rounded-lg transition-colors"
              aria-label="Close chapter navigation"
            >
              <ChevronLeft size={20} />
            </button>
          </div>

          {/* Current Chapter Info */}
          {currentChapter && (
            <div className="bg-white/10 backdrop-blur-sm rounded-lg p-3 text-white">
              <div className="text-xs opacity-80 mb-1">Currently Reading</div>
              <div className="font-semibold text-sm">
                Ch {currentChapter.id}: {currentChapter.title}
              </div>
              <div className="text-xs opacity-80 mt-1">
                Page {currentPage} of {currentChapter.endPage}
              </div>
            </div>
          )}
        </div>

        {/* Filter Buttons */}
        <div className="p-4 border-b border-gray-200 bg-gray-50">
          <div className="flex items-center gap-2 mb-2">
            <Filter size={16} className="text-gray-600" />
            <span className="text-xs font-semibold text-gray-600">Filter by Level</span>
          </div>
          <div className="grid grid-cols-4 gap-2">
            <button
              type="button"
              onClick={() => setSelectedPart('all')}
              className={`px-3 py-2 rounded-lg text-xs font-medium transition-all
                        ${selectedPart === 'all' 
                          ? 'bg-primary text-white shadow-md' 
                          : 'bg-white text-gray-600 hover:bg-gray-100'
                        }`}
              aria-label="Show all chapters"
            >
              All
            </button>
            <button
              type="button"
              onClick={() => setSelectedPart(1)}
              className={`px-3 py-2 rounded-lg text-xs font-medium transition-all
                        ${selectedPart === 1 
                          ? 'bg-green-500 text-white shadow-md' 
                          : 'bg-white text-gray-600 hover:bg-gray-100'
                        }`}
              aria-label="Show student chapters"
            >
              📘 Easy
            </button>
            <button
              type="button"
              onClick={() => setSelectedPart(2)}
              className={`px-3 py-2 rounded-lg text-xs font-medium transition-all
                        ${selectedPart === 2 
                          ? 'bg-yellow-500 text-white shadow-md' 
                          : 'bg-white text-gray-600 hover:bg-gray-100'
                        }`}
              aria-label="Show researcher chapters"
            >
              📗 Med
            </button>
            <button
              type="button"
              onClick={() => setSelectedPart(3)}
              className={`px-3 py-2 rounded-lg text-xs font-medium transition-all
                        ${selectedPart === 3 
                          ? 'bg-red-500 text-white shadow-md' 
                          : 'bg-white text-gray-600 hover:bg-gray-100'
                        }`}
              aria-label="Show expert chapters"
            >
              📕 Hard
            </button>
          </div>
        </div>

        {/* Chapters List */}
        <div className="flex-1 overflow-y-auto p-4 space-y-2">
          {displayChapters.map((chapter) => {
            const isActive = currentPage >= chapter.startPage && currentPage <= chapter.endPage;
            const partBadge = getPartBadge(chapter.part);

            return (
              <button
                key={chapter.id}
                type="button"
                onClick={() => onPageChange(chapter.startPage)}
                className={`w-full text-left p-3 rounded-lg transition-all duration-200
                          border-2 ${isActive 
                            ? 'border-primary bg-primary text-white shadow-md scale-105' 
                            : 'border-gray-200 hover:border-gray-300 hover:bg-gray-50'
                          }`}
                aria-label={`Go to ${chapter.title}`}
              >
                <div className="flex items-start justify-between gap-2 mb-2">
                  <div className="flex items-center gap-2">
                    <span className={`w-2 h-2 rounded-full ${partBadge.color}`} />
                    <span className={`text-xs font-semibold ${
                      isActive ? 'text-white' : 'text-gray-900'
                    }`}>
                      Chapter {chapter.id}
                    </span>
                  </div>
                  <span className={`text-xs px-2 py-0.5 rounded-full ${
                    isActive 
                      ? 'bg-white/20 text-white' 
                      : `${getPartColor(chapter.part)} border`
                  }`}>
                    {partBadge.label}
                  </span>
                </div>

                <div className={`font-semibold text-sm mb-1 ${
                  isActive ? 'text-white' : 'text-gray-800'
                }`}>
                  {chapter.title}
                </div>

                <div className="flex items-center justify-between text-xs">
                  <span className={isActive ? 'text-white/80' : 'text-gray-500'}>
                    {chapter.estimatedTime}
                  </span>
                  <span className={`font-mono ${isActive ? 'text-white/80' : 'text-gray-400'}`}>
                    p.{chapter.startPage}-{chapter.endPage}
                  </span>
                </div>
              </button>
            );
          })}
        </div>

        {/* Footer */}
        <div className="p-4 border-t border-gray-200 bg-gray-50">
          <div className="text-xs text-gray-600 text-center">
            {displayChapters.length} {displayChapters.length === 1 ? 'Chapter' : 'Chapters'}
            {selectedPart !== 'all' && ` • ${getPartBadge(selectedPart as number).label} Level`}
          </div>
        </div>
      </div>

      {/* Overlay (when sidebar is open) */}
      {isOpen && (
        <div
          className="fixed inset-0 bg-black/30 z-40"
          onClick={onToggle}
          aria-hidden="true"
        />
      )}
    </>
  );
};