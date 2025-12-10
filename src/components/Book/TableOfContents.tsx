// frontend/src/components/TableOfContents.tsx

import React, { useState } from 'react';
import { Book, ChevronDown, ChevronRight } from 'lucide-react';
import { bookData, bookMetadata } from '../data/bookContent';

interface TableOfContentsProps {
  currentPage: number;
  onPageChange: (page: number) => void;
  onClose?: () => void;
}

export const TableOfContents: React.FC<TableOfContentsProps> = ({
  currentPage,
  onPageChange,
  onClose
}) => {
  const [expandedParts, setExpandedParts] = useState<Set<number>>(new Set([1, 2, 3]));

  const togglePart = (partNumber: number) => {
    const newExpanded = new Set(expandedParts);
    if (newExpanded.has(partNumber)) {
      newExpanded.delete(partNumber);
    } else {
      newExpanded.add(partNumber);
    }
    setExpandedParts(newExpanded);
  };

  const handleChapterClick = (page: number) => {
    onPageChange(page);
    if (onClose) onClose();
  };

  const getPartColor = (part: number) => {
    switch (part) {
      case 1: return 'text-green-600 bg-green-50 border-green-200';
      case 2: return 'text-yellow-600 bg-yellow-50 border-yellow-200';
      case 3: return 'text-red-600 bg-red-50 border-red-200';
      default: return 'text-gray-600 bg-gray-50 border-gray-200';
    }
  };

  const getPartIcon = (part: number) => {
    switch (part) {
      case 1: return '📘';
      case 2: return '📗';
      case 3: return '📕';
      default: return '📚';
    }
  };

  return (
    <div className="h-full flex flex-col bg-white">
      {/* Header */}
      <div className="p-6 border-b border-gray-200 bg-gradient-to-r from-primary to-primary-dark">
        <div className="flex items-center gap-3 text-white">
          <Book size={28} />
          <div>
            <h2 className="text-xl font-bold">{bookMetadata.title}</h2>
            <p className="text-sm text-white/80">by {bookMetadata.author}</p>
          </div>
        </div>
        <div className="mt-4 grid grid-cols-3 gap-2 text-white/90 text-xs">
          <div className="text-center">
            <div className="font-bold text-lg">{bookMetadata.totalChapters}</div>
            <div>Chapters</div>
          </div>
          <div className="text-center">
            <div className="font-bold text-lg">{bookMetadata.totalPages}</div>
            <div>Pages</div>
          </div>
          <div className="text-center">
            <div className="font-bold text-lg">3</div>
            <div>Parts</div>
          </div>
        </div>
      </div>

      {/* Table of Contents */}
      <div className="flex-1 overflow-y-auto p-4 space-y-2">
        {bookMetadata.parts.map((partInfo) => {
          const isExpanded = expandedParts.has(partInfo.number);
          const partChapters = bookData.filter(ch => ch.part === partInfo.number);
          const partColor = getPartColor(partInfo.number);

          return (
            <div key={partInfo.number} className="mb-4">
              {/* Part Header */}
              <button
                onClick={() => togglePart(partInfo.number)}
                className={`w-full p-3 rounded-lg border-2 ${partColor} 
                          transition-all duration-200 hover:shadow-md
                          flex items-center justify-between group`}
              >
                <div className="flex items-center gap-3">
                  <span className="text-2xl">{getPartIcon(partInfo.number)}</span>
                  <div className="text-left">
                    <div className="font-bold text-sm">
                      Part {partInfo.number}: {partInfo.name}
                    </div>
                    <div className="text-xs opacity-75">
                      {partInfo.chapters} chapters • {partInfo.pages} pages • {partInfo.difficulty}
                    </div>
                  </div>
                </div>
                <div className="transform transition-transform duration-200">
                  {isExpanded ? (
                    <ChevronDown size={20} />
                  ) : (
                    <ChevronRight size={20} />
                  )}
                </div>
              </button>

              {/* Chapters List */}
              {isExpanded && (
                <div className="mt-2 ml-6 space-y-1 animate-slide-down">
                  {partChapters.map((chapter) => {
                    const isCurrentChapter = 
                      currentPage >= chapter.startPage && 
                      currentPage <= chapter.endPage;

                    return (
                      <button
                        key={chapter.id}
                        onClick={() => handleChapterClick(chapter.startPage)}
                        className={`w-full text-left p-3 rounded-lg transition-all duration-200
                                  ${isCurrentChapter 
                                    ? 'bg-primary text-white shadow-md' 
                                    : 'hover:bg-gray-100 text-gray-700'
                                  }`}
                      >
                        <div className="flex items-start justify-between gap-2">
                          <div className="flex-1 min-w-0">
                            <div className={`text-sm font-semibold ${
                              isCurrentChapter ? 'text-white' : 'text-gray-900'
                            }`}>
                              Ch {chapter.id}: {chapter.title}
                            </div>
                            <div className={`text-xs mt-1 ${
                              isCurrentChapter ? 'text-white/80' : 'text-gray-500'
                            }`}>
                              {chapter.estimatedTime} • {chapter.difficulty}
                            </div>
                          </div>
                          <div className={`text-xs font-mono whitespace-nowrap ${
                            isCurrentChapter ? 'text-white' : 'text-gray-400'
                          }`}>
                            p.{chapter.startPage}
                          </div>
                        </div>
                      </button>
                    );
                  })}
                </div>
              )}
            </div>
          );
        })}
      </div>

      {/* Footer Stats */}
      <div className="p-4 border-t border-gray-200 bg-gray-50">
        <div className="text-xs text-gray-600 text-center">
          Currently on page <span className="font-bold text-primary">{currentPage}</span>
        </div>
      </div>
    </div>
  );
};