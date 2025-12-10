import React from 'react';
import ReactMarkdown from 'react-markdown';
import '../../styles/navigation.css';

interface BookPageProps {
  pageNumber: number;
  content: string;
  chapterTitle: string;
}

export const BookPage: React.FC<BookPageProps> = ({ 
  pageNumber, 
  content, 
  chapterTitle 
}) => {
  let listItemIndex = 0;

  return (
    <div className="relative p-8 md:p-16 bg-gradient-to-br from-white to-green-50/30 min-h-[700px]">
      {/* Decorative Corner Elements */}
      <div className="absolute top-0 left-0 w-40 h-40 bg-gradient-to-br from-blue-400/10 to-transparent rounded-br-full"></div>
      <div className="absolute bottom-0 right-0 w-40 h-40 bg-gradient-to-tl from-purple-400/10 to-transparent rounded-tl-full"></div>

      {/* Chapter Badge - BLUE ZONE */}
      <div className="flex justify-center mb-8">
        <div className="inline-flex items-center px-8 py-3 bg-gradient-to-r from-blue-600 to-indigo-600 rounded-full shadow-xl transform hover:scale-105 transition-all">
          <span className="text-base font-black text-white tracking-widest">PART 1: STUDENTS</span>
        </div>
      </div>

      {/* Chapter Title - PURPLE GRADIENT ZONE */}
      <div className="text-center mb-12 bg-gradient-to-r from-purple-50 via-pink-50 to-purple-50 py-8 rounded-3xl">
        <h1 className="text-5xl md:text-7xl font-black mb-4 bg-gradient-to-r from-purple-600 via-pink-600 to-purple-600 bg-clip-text text-transparent leading-tight drop-shadow-2xl">
          {chapterTitle}
        </h1>
        <div className="flex justify-center mt-6">
          <div className="h-2 w-40 bg-gradient-to-r from-purple-400 via-pink-400 to-purple-400 rounded-full shadow-lg"></div>
        </div>
      </div>

      {/* Content */}
      <div className="max-w-5xl mx-auto">
        <div className="prose prose-xl max-w-none">
          <ReactMarkdown
            components={{
              h1: ({ children }) => (
                <div className="bg-gradient-to-r from-orange-100 to-red-100 p-6 rounded-2xl mb-8">
                  <h1 className="text-6xl font-black text-center bg-gradient-to-r from-orange-600 to-red-600 bg-clip-text text-transparent">
                    {children}
                  </h1>
                </div>
              ),
              h2: ({ children }) => (
                <div className="bg-gradient-to-r from-teal-100 to-cyan-100 p-6 rounded-2xl my-8">
                  <h2 className="text-5xl font-black text-center bg-gradient-to-r from-teal-600 to-cyan-600 bg-clip-text text-transparent">
                    {children}
                  </h2>
                </div>
              ),
              h3: ({ children }) => (
                <div className="bg-gradient-to-r from-emerald-100 to-green-100 p-5 rounded-2xl my-8">
                  <h3 className="text-4xl font-black text-center bg-gradient-to-r from-emerald-600 to-green-600 bg-clip-text text-transparent">
                    {children}
                  </h3>
                </div>
              ),
              p: ({ children }) => (
                <div className="bg-gradient-to-r from-slate-50 to-gray-50 p-5 rounded-xl my-6 border-l-4 border-indigo-500">
                  <p className="text-2xl font-bold text-gray-900 leading-relaxed text-center">
                    {children}
                  </p>
                </div>
              ),
              ul: ({ children }) => {
                listItemIndex = 0;
                return (
                  <div className="my-10">
                    <div className="flex flex-wrap justify-center gap-6 max-w-5xl mx-auto">
                      {children}
                    </div>
                  </div>
                );
              },
              li: ({ children }) => {
                const cards = [
                  { color: 'from-blue-500 to-blue-700', shadow: 'shadow-blue-500/50' },
                  { color: 'from-purple-500 to-purple-700', shadow: 'shadow-purple-500/50' },
                  { color: 'from-pink-500 to-pink-700', shadow: 'shadow-pink-500/50' },
                  { color: 'from-orange-500 to-orange-700', shadow: 'shadow-orange-500/50' }
                ];
                const currentIndex = listItemIndex;
                const card = cards[currentIndex % cards.length];
                listItemIndex++;
                
                return (
                  <div className={`group relative bg-gradient-to-br ${card.color} p-8 rounded-3xl shadow-2xl ${card.shadow}
                                  hover:shadow-3xl transform hover:scale-110 transition-all duration-300 cursor-pointer
                                  w-full sm:w-[calc(50%-1rem)] min-h-[140px] flex flex-col justify-center`}>
                    <div className="absolute top-4 right-4 w-10 h-10 bg-white/30 rounded-full flex items-center justify-center backdrop-blur-sm">
                      <span className="text-white font-black text-lg">{currentIndex + 1}</span>
                    </div>
                    <div className="text-white font-black text-2xl text-center leading-relaxed px-4">
                      {children}
                    </div>
                    <div className="absolute bottom-3 left-3 w-8 h-8 bg-white/20 rounded-full"></div>
                    <div className="absolute top-3 left-3 w-4 h-4 bg-white/20 rounded-full"></div>
                  </div>
                );
              },
            }}
          >
            {content}
          </ReactMarkdown>
        </div>
      </div>

      {/* Author Signature - GREEN ZONE */}
      <div className="mt-16 text-center">
        <div className="inline-block px-10 py-5 bg-gradient-to-r from-green-500 to-emerald-600 rounded-3xl shadow-2xl shadow-green-500/50 transform hover:scale-105 transition-all">
          <p className="text-3xl font-black text-white mb-2">Azmat Ali</p>
          <p className="text-base text-white/90 font-bold tracking-wide">Author & Robotics Expert</p>
        </div>
      </div>

      {/* Page Footer - AMBER ZONE */}
      <div className="mt-12 pt-8 border-t-4 border-gradient-to-r from-amber-400 to-orange-400 flex justify-center">
        <div className="flex items-center space-x-5 bg-gradient-to-r from-amber-500 to-orange-600 px-10 py-5 rounded-full shadow-2xl shadow-amber-500/50">
          <div className="w-14 h-14 rounded-full bg-white flex items-center justify-center shadow-lg">
            <span className="text-orange-600 font-black text-2xl">{pageNumber}</span>
          </div>
          <span className="text-white font-black text-3xl">Page {pageNumber}</span>
        </div>
      </div>
    </div>
  );
};