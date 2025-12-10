// frontend/src/components/ProgressTracker.tsx

import React, { useEffect, useState } from 'react';
import { BookOpen, CheckCircle, Clock, TrendingUp } from 'lucide-react';
import { bookData, bookMetadata } from '../data/bookContent';

interface ProgressTrackerProps {
  currentPage: number;
  completedChapters?: number[];
}

export const ProgressTracker: React.FC<ProgressTrackerProps> = ({
  currentPage,
  completedChapters = []
}) => {
  const [readingTime, setReadingTime] = useState(0);

  const totalPages = bookMetadata.totalPages;
  const progressPercentage = Math.round((currentPage / totalPages) * 100);
  
  const currentChapter = bookData.find(
    ch => currentPage >= ch.startPage && currentPage <= ch.endPage
  );

  const completedCount = completedChapters.length;
  const chaptersPercentage = Math.round((completedCount / bookMetadata.totalChapters) * 100);

  useEffect(() => {
    const interval = setInterval(() => {
      setReadingTime(prev => prev + 1);
    }, 1000);
    return () => clearInterval(interval);
  }, []);

  const formatTime = (seconds: number): string => {
    const hours = Math.floor(seconds / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    return hours > 0 ? `${hours}h ${minutes}m` : `${minutes}m`;
  };

  const getPartProgress = (partNumber: 1 | 2 | 3) => {
    const partChapters = bookData.filter(ch => ch.part === partNumber);
    const completedInPart = partChapters.filter(ch => completedChapters.includes(ch.id)).length;
    return Math.round((completedInPart / partChapters.length) * 100);
  };

  return (
    <div className="bg-white rounded-lg shadow-md p-4 space-y-4">
      <div className="flex items-center justify-between">
        <h3 className="font-bold text-gray-800 flex items-center gap-2">
          <TrendingUp size={20} className="text-primary" />
          Your Progress
        </h3>
        <div className="text-xs text-gray-500 flex items-center gap-1">
          <Clock size={14} />
          {formatTime(readingTime)}
        </div>
      </div>

      <div>
        <div className="flex items-center justify-between text-sm mb-2">
          <span className="text-gray-600">Book Progress</span>
          <span className="font-bold text-primary">{progressPercentage}%</span>
        </div>
        <div className="w-full bg-gray-200 rounded-full h-3 overflow-hidden">
          <div
            className="bg-gradient-to-r from-primary to-primary-dark h-full rounded-full transition-all duration-500 ease-out"
            style={{ width: `${progressPercentage}%` }}
          />
        </div>
        <div className="text-xs text-gray-500 mt-1">
          Page {currentPage} of {totalPages}
        </div>
      </div>

      {currentChapter && (
        <div className="bg-primary/5 rounded-lg p-3 border border-primary/20">
          <div className="flex items-start gap-2">
            <BookOpen size={16} className="text-primary mt-0.5" />
            <div className="flex-1">
              <div className="text-xs text-primary font-semibold mb-1">Currently Reading</div>
              <div className="text-sm font-bold text-gray-800">Ch {currentChapter.id}: {currentChapter.title}</div>
              <div className="text-xs text-gray-600 mt-1">{currentChapter.estimatedTime} • {currentChapter.difficulty}</div>
            </div>
          </div>
        </div>
      )}

      <div>
        <div className="flex items-center justify-between text-sm mb-2">
          <span className="text-gray-600 flex items-center gap-1">
            <CheckCircle size={16} />
            Chapters Completed
          </span>
          <span className="font-bold text-green-600">{completedCount}/{bookMetadata.totalChapters}</span>
        </div>
        <div className="w-full bg-gray-200 rounded-full h-2 overflow-hidden">
          <div
            className="bg-gradient-to-r from-green-400 to-green-600 h-full rounded-full transition-all duration-500"
            style={{ width: `${chaptersPercentage}%` }}
          />
        </div>
      </div>

      <div className="space-y-2">
        <div className="text-xs font-semibold text-gray-600 mb-2">Progress by Level</div>
        
        <div className="flex items-center gap-2">
          <span className="text-xs w-16 text-gray-600">📘 Easy</span>
          <div className="flex-1 bg-gray-200 rounded-full h-2">
            <div className="bg-green-500 h-full rounded-full transition-all duration-300" style={{ width: `${getPartProgress(1)}%` }} />
          </div>
          <span className="text-xs font-semibold text-gray-700 w-10 text-right">{getPartProgress(1)}%</span>
        </div>

        <div className="flex items-center gap-2">
          <span className="text-xs w-16 text-gray-600">📗 Medium</span>
          <div className="flex-1 bg-gray-200 rounded-full h-2">
            <div className="bg-yellow-500 h-full rounded-full transition-all duration-300" style={{ width: `${getPartProgress(2)}%` }} />
          </div>
          <span className="text-xs font-semibold text-gray-700 w-10 text-right">{getPartProgress(2)}%</span>
        </div>

        <div className="flex items-center gap-2">
          <span className="text-xs w-16 text-gray-600">📕 Hard</span>
          <div className="flex-1 bg-gray-200 rounded-full h-2">
            <div className="bg-red-500 h-full rounded-full transition-all duration-300" style={{ width: `${getPartProgress(3)}%` }} />
          </div>
          <span className="text-xs font-semibold text-gray-700 w-10 text-right">{getPartProgress(3)}%</span>
        </div>
      </div>

      <div className="border-t border-gray-200 pt-3">
        <div className="grid grid-cols-3 gap-2 text-center">
          <div>
            <div className="text-2xl mb-1">{completedCount >= 10 ? '🏆' : completedCount >= 5 ? '⭐' : '📚'}</div>
            <div className="text-xs text-gray-600">Chapters</div>
          </div>
          <div>
            <div className="text-2xl mb-1">{progressPercentage >= 75 ? '🔥' : progressPercentage >= 50 ? '💪' : '🚀'}</div>
            <div className="text-xs text-gray-600">Progress</div>
          </div>
          <div>
            <div className="text-2xl mb-1">{readingTime > 3600 ? '📖' : readingTime > 1800 ? '⏱️' : '👀'}</div>
            <div className="text-xs text-gray-600">Reading</div>
          </div>
        </div>
      </div>
    </div>
  );
};

export const CompactProgress: React.FC<{ currentPage: number }> = ({ currentPage }) => {
  const totalPages = bookMetadata.totalPages;
  const progressPercentage = Math.round((currentPage / totalPages) * 100);

  return (
    <div className="flex items-center gap-2">
      <div className="w-24 bg-gray-200 rounded-full h-2">
        <div className="bg-primary h-full rounded-full transition-all duration-300" style={{ width: `${progressPercentage}%` }} />
      </div>
      <span className="text-xs font-semibold text-gray-700">{progressPercentage}%</span>
    </div>
  );
};