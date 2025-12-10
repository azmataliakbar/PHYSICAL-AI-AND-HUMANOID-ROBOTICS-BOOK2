import { Book, Home, Menu, Search as SearchIcon } from 'lucide-react';
import React, { useState } from 'react';
import '../../styles/content.css';
import { SearchBar } from '../Common/SearchBar';
import { bookData, bookMetadata } from '../data/bookContent';
import { ChapterNav } from './ChapterNav';
import { ProgressTracker } from './ProgressTracker';
import { TableOfContents } from './TableOfContents';

export const BookReader: React.FC = () => {
  const [currentPage, setCurrentPage] = useState(1);
  const [showTOC, setShowTOC] = useState(false);
  const [showSearch, setShowSearch] = useState(false);
  const [showChapterNav, setShowChapterNav] = useState(false);
  const [completedChapters, setCompletedChapters] = useState<number[]>([]);

  // Get current chapter based on page
  const currentChapter = bookData.find(
    chapter => currentPage >= chapter.startPage && currentPage <= chapter.endPage
  );

  // Navigation functions
  const goToNextPage = () => {
    if (currentPage < bookMetadata.totalPages) {
      setCurrentPage(prev => prev + 1);
    }
  };

  const goToPreviousPage = () => {
    if (currentPage > 1) {
      setCurrentPage(prev => prev - 1);
    }
  };

  const goToHomePage = () => {
    setCurrentPage(1);
  };

  const handlePageChange = (page: number) => {
    setCurrentPage(page);
    setShowTOC(false);
    setShowChapterNav(false);
  };

  // Mark chapter as completed
  const markChapterComplete = (chapterId: number) => {
    if (!completedChapters.includes(chapterId)) {
      setCompletedChapters([...completedChapters, chapterId]);
    }
  };

  // Render markdown content
  const renderContent = (content: string) => {
    return content.split('\n').map((line, index) => {
      if (line.startsWith('# ')) {
        return <h1 key={index}>{line.substring(2)}</h1>;
      }
      if (line.startsWith('## ')) {
        return <h2 key={index}>{line.substring(3)}</h2>;
      }
      if (line.startsWith('### ')) {
        return <h3 key={index}>{line.substring(4)}</h3>;
      }
      if (line.startsWith('```')) {
        return null;
      }
      if (line.startsWith('- ') || line.startsWith('* ')) {
        return <li key={index}>{line.substring(2)}</li>;
      }
      if (line.trim() === '') {
        return <br key={index} />;
      }
      return <p key={index} dangerouslySetInnerHTML={{ __html: line }} />;
    });
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-gray-50 to-gray-100">
      {/* Header */}
      <header className="bg-white border-b border-gray-200 sticky top-0 z-30 shadow-sm">
        <div className="max-w-7xl mx-auto px-2 sm:px-4 lg:px-8">
          <div className="flex items-center justify-between h-14 sm:h-16">
            <div className="flex items-center gap-2 sm:gap-4">
              <button
                type="button"
                onClick={() => setShowChapterNav(!showChapterNav)}
                className="p-2 rounded-lg hover:bg-gray-100 transition-colors"
                aria-label="Toggle chapter navigation"
              >
                <Menu size={20} className="text-gray-700 sm:w-6 sm:h-6" />
              </button>
              
              <div className="flex items-center gap-2 sm:gap-3">
                <Book size={24} className="text-red-600 sm:w-7 sm:h-7" />
                <div className="hidden sm:block">
                  <h1 className="text-base sm:text-lg font-bold text-green-600">
                    {bookMetadata.title}
                  </h1>
                  <p className="text-xs sm:text-sm text-blue-600 font-bold">
                    by {bookMetadata.author}
                  </p>
                </div>
              </div>
            </div>

            <div className="hidden lg:block flex-1 max-w-2xl mx-8">
              <SearchBar onResultClick={handlePageChange} />
            </div>

            <div className="flex items-center gap-2">
              <button
                type="button"
                onClick={() => setShowSearch(!showSearch)}
                className="lg:hidden p-2 rounded-lg hover:bg-gray-100 transition-colors"
                aria-label="Toggle search"
              >
                <SearchIcon size={18} className="text-gray-700 sm:w-5 sm:h-5" />
              </button>
              
              <button
                type="button"
                onClick={() => setShowTOC(!showTOC)}
                className="px-3 py-1.5 sm:px-4 sm:py-2 bg-primary text-white rounded-lg hover:bg-primary-dark
                        transition-colors font-semibold text-xs sm:text-sm"
              >
                Contents
              </button>
            </div>
          </div>

          {showSearch && (
            <div className="lg:hidden pb-4">
              <SearchBar onResultClick={(page) => {
                handlePageChange(page);
                setShowSearch(false);
              }} />
            </div>
          )}
        </div>
      </header>

      {/* Main Content */}
      <div className="max-w-7xl mx-auto px-2 sm:px-4 lg:px-8 py-4 sm:py-8">
        <div className="grid grid-cols-1 lg:grid-cols-4 gap-4 sm:gap-8">
          <aside className="hidden lg:block">
            <div className="sticky top-24">
              <ProgressTracker
                currentPage={currentPage}
                completedChapters={completedChapters}
              />
            </div>
          </aside>

          <main className="lg:col-span-3">
            <div className="bg-white rounded-lg shadow-lg border border-gray-300 min-h-[600px]">
              {currentChapter && (
                <div className="border-b border-gray-200 p-3 sm:p-6 bg-gradient-to-r from-primary/5 to-primary/10">
                  <div className="flex items-center justify-between mb-2">
                    <span className="text-xs font-semibold text-primary uppercase tracking-wide">
                      Ch {currentChapter.id} • {currentChapter.partName}
                    </span>
                    <span className="text-xs px-2 py-1 rounded-full bg-primary/20 text-primary font-semibold">
                      {currentChapter.difficulty}
                    </span>
                  </div>
                  <h1 className="text-lg sm:text-2xl font-bold text-gray-900 mb-2">
                    {currentChapter.title}
                  </h1>
                  <div className="flex items-center gap-2 sm:gap-4 text-xs text-gray-600">
                    <span>⏱️ {currentChapter.estimatedTime}</span>
                    <span className="hidden sm:inline">•</span>
                    <span className="hidden sm:inline">
                      📄 Pages {currentChapter.startPage}-{currentChapter.endPage}
                    </span>
                  </div>
                </div>
              )}

              <div className="p-4 sm:p-8">
                <div className="book-content">
                  {currentChapter ? (
                    renderContent(currentChapter.content)
                  ) : (
                    <div className="text-center py-8 sm:py-12 welcome-page">
                      <Book 
                        size={48} 
                        className="mx-auto mb-4 sm:w-16 sm:h-16" 
                        style={{ stroke: '#ef4444' }} 
                      />
                      <h2 className="text-xl sm:text-2xl font-bold mb-2 title-text px-2">
                        Welcome to Physical AI and Humanoid Robotics
                      </h2>
                      <div className="flex justify-center w-full mb-4 sm:mb-6">
                        <p className="author-text text-sm sm:text-base">
                          by <span className="author-text font-semibold">{bookMetadata.author}</span>
                        </p>
                      </div>
                      <button
                        type="button"
                        onClick={() => setCurrentPage(5)}
                        className="px-5 py-2.5 sm:px-6 sm:py-3 rounded-lg font-semibold transition-colors start-button text-sm sm:text-base"
                      >
                        Start Reading
                      </button>
                    </div>
                  )}
                </div>
              </div>

              {/* ============================================
                  FULLY RESPONSIVE FOOTER - 270px to Desktop
                  ============================================ */}
              <div className="border-t border-gray-200 p-3 sm:p-6 bg-gray-50">
                <div className="flex flex-col gap-3 sm:grid sm:grid-cols-3 sm:gap-4 items-center">
                  
                  {/* PAGE INFO - Shows FIRST on mobile, CENTER on desktop */}
                  <div className="text-center order-1 sm:order-2 w-full">
                    <div className="text-xs sm:text-sm font-semibold text-gray-700">
                      Page {currentPage} of {bookMetadata.totalPages}
                    </div>
                    {currentChapter && (
                      <button
                        type="button"
                        onClick={() => markChapterComplete(currentChapter.id)}
                        className={`text-xs mt-1 ${
                          completedChapters.includes(currentChapter.id)
                            ? 'text-green-600'
                            : 'text-primary hover:underline'
                        }`}
                      >
                        {completedChapters.includes(currentChapter.id)
                          ? '✓ Completed'
                          : 'Mark as Complete'}
                      </button>
                    )}
                  </div>

                  {/* HOME BUTTON - Shows SECOND on mobile, LEFT on desktop */}
                  <div className="flex justify-center sm:justify-start order-2 sm:order-1 w-full sm:w-auto">
                    <button
                      type="button"
                      onClick={goToHomePage}
                      className="px-4 py-2 rounded-lg border border-gray-300 hover:bg-gray-100
                               transition-colors font-semibold text-sm flex items-center gap-2 
                               w-full sm:w-auto justify-center"
                      title="Go to Home"
                    >
                      <Home size={18} />
                      <span>Home</span>
                    </button>
                  </div>

                  {/* PREV/NEXT BUTTONS - Shows THIRD on mobile, RIGHT on desktop */}
                  <div className="flex justify-center sm:justify-end gap-2 order-3 w-full sm:w-auto">
                    <button
                      type="button"
                      onClick={goToPreviousPage}
                      disabled={currentPage === 1}
                      className="px-3 sm:px-4 py-2 rounded-lg border border-gray-300 hover:bg-gray-100
                               disabled:opacity-50 disabled:cursor-not-allowed transition-colors
                               font-semibold text-xs sm:text-sm flex-1 sm:flex-none"
                    >
                      <span className="hidden sm:inline">← </span>
                      <span className="sm:hidden">←</span>
                      <span className="hidden xs:inline sm:hidden">Prev</span>
                      <span className="hidden sm:inline">Previous</span>
                    </button>

                    <button
                      type="button"
                      onClick={goToNextPage}
                      disabled={currentPage === bookMetadata.totalPages}
                      className="px-3 sm:px-4 py-2 rounded-lg border border-gray-300 hover:bg-gray-100
                               disabled:opacity-50 disabled:cursor-not-allowed transition-colors
                               font-semibold text-xs sm:text-sm flex-1 sm:flex-none"
                    >
                      <span className="hidden sm:inline">Next </span>
                      <span className="sm:hidden">Next</span>
                      <span className="hidden sm:inline">→</span>
                      <span className="sm:hidden">→</span>
                    </button>
                  </div>
                  
                </div>
              </div>
            </div>

            <div className="lg:hidden mt-4 sm:mt-6">
              <ProgressTracker
                currentPage={currentPage}
                completedChapters={completedChapters}
              />
            </div>
          </main>
        </div>
      </div>

      {showTOC && (
        <>
          <div
            className="fixed inset-0 bg-black/50 z-40"
            onClick={() => setShowTOC(false)}
          />
          <div className="fixed right-0 top-0 h-full w-full sm:w-96 bg-white shadow-2xl z-50 overflow-y-auto">
            <TableOfContents
              currentPage={currentPage}
              onPageChange={handlePageChange}
              onClose={() => setShowTOC(false)}
            />
          </div>
        </>
      )}

      <ChapterNav
        currentPage={currentPage}
        onPageChange={handlePageChange}
        isOpen={showChapterNav}
        onToggle={() => setShowChapterNav(!showChapterNav)}
      />
    </div>
  );
};

export default BookReader;