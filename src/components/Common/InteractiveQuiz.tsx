// frontend/src/components/InteractiveQuiz.tsx

import React, { useState } from 'react';
import { CheckCircle, XCircle, RotateCcw, ChevronRight } from 'lucide-react';

interface QuizOption {
  id: string;
  text: string;
  isCorrect: boolean;
}

interface QuizQuestion {
  id: number;
  question: string;
  options: QuizOption[];
  explanation: string;
  difficulty: 'Easy' | 'Medium' | 'Hard';
}

interface InteractiveQuizProps {
  questions: QuizQuestion[];
  title?: string;
  onComplete?: (score: number) => void;
}

export const InteractiveQuiz: React.FC<InteractiveQuizProps> = ({
  questions,
  title = '📝 Knowledge Check',
  onComplete
}) => {
  const [currentQuestion, setCurrentQuestion] = useState(0);
  const [selectedAnswer, setSelectedAnswer] = useState<string | null>(null);
  const [showExplanation, setShowExplanation] = useState(false);
  const [score, setScore] = useState(0);
  const [answeredQuestions, setAnsweredQuestions] = useState<number[]>([]);
  const [isComplete, setIsComplete] = useState(false);

  const question = questions[currentQuestion];
  const isLastQuestion = currentQuestion === questions.length - 1;
  const hasAnswered = answeredQuestions.includes(currentQuestion);

  const handleAnswerSelect = (optionId: string) => {
    if (hasAnswered) return;

    setSelectedAnswer(optionId);
    setShowExplanation(true);

    const isCorrect = question.options.find(opt => opt.id === optionId)?.isCorrect;
    if (isCorrect) {
      setScore(prev => prev + 1);
    }

    setAnsweredQuestions([...answeredQuestions, currentQuestion]);
  };

  const handleNext = () => {
    if (isLastQuestion) {
      setIsComplete(true);
      if (onComplete) {
        onComplete(score);
      }
    } else {
      setCurrentQuestion(prev => prev + 1);
      setSelectedAnswer(null);
      setShowExplanation(false);
    }
  };

  const handleReset = () => {
    setCurrentQuestion(0);
    setSelectedAnswer(null);
    setShowExplanation(false);
    setScore(0);
    setAnsweredQuestions([]);
    setIsComplete(false);
  };

  const getScorePercentage = () => {
    return Math.round((score / questions.length) * 100);
  };

  const getScoreEmoji = () => {
    const percentage = getScorePercentage();
    if (percentage >= 90) return '🏆';
    if (percentage >= 70) return '⭐';
    if (percentage >= 50) return '👍';
    return '📚';
  };

  const getDifficultyColor = (difficulty: string) => {
    switch (difficulty) {
      case 'Easy': return 'bg-green-100 text-green-700';
      case 'Medium': return 'bg-yellow-100 text-yellow-700';
      case 'Hard': return 'bg-red-100 text-red-700';
      default: return 'bg-gray-100 text-gray-700';
    }
  };

  if (isComplete) {
    return (
      <div className="my-6 p-6 bg-gradient-to-br from-primary/5 to-primary/10 rounded-lg border-2 border-primary/30">
        <div className="text-center">
          <div className="text-6xl mb-4">{getScoreEmoji()}</div>
          <h3 className="text-2xl font-bold text-gray-800 mb-2">Quiz Complete!</h3>
          <div className="text-4xl font-bold text-primary mb-4">
            {score}/{questions.length}
          </div>
          <div className="text-lg text-gray-600 mb-6">
            You scored {getScorePercentage()}%
          </div>

          <div className="flex justify-center gap-3">
            <button
              type="button"
              onClick={handleReset}
              className="flex items-center gap-2 px-6 py-3 bg-primary text-white rounded-lg
                       hover:bg-primary-dark transition-colors font-semibold"
            >
              <RotateCcw size={20} />
              Try Again
            </button>
          </div>

          <div className="mt-6 p-4 bg-white rounded-lg">
            <div className="text-sm font-semibold text-gray-700 mb-2">Performance Breakdown</div>
            <div className="flex items-center gap-2">
              <div className="flex-1 bg-gray-200 rounded-full h-3 overflow-hidden">
                <div
                  className="bg-gradient-to-r from-green-400 to-green-600 h-full rounded-full transition-all duration-500"
                  style={{ width: `${getScorePercentage()}%` }}
                />
              </div>
              <span className="text-sm font-bold text-gray-700">{getScorePercentage()}%</span>
            </div>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="my-6 p-6 bg-white rounded-lg border-2 border-primary/30 shadow-md">
      {/* Header */}
      <div className="flex items-center justify-between mb-6">
        <h3 className="text-xl font-bold text-gray-800">{title}</h3>
        <div className="flex items-center gap-3">
          <span className={`px-3 py-1 rounded-full text-xs font-semibold ${getDifficultyColor(question.difficulty)}`}>
            {question.difficulty}
          </span>
          <span className="text-sm text-gray-600">
            Question {currentQuestion + 1}/{questions.length}
          </span>
        </div>
      </div>

      {/* Progress Bar */}
      <div className="mb-6">
        <div className="w-full bg-gray-200 rounded-full h-2 overflow-hidden">
          <div
            className="bg-primary h-full rounded-full transition-all duration-300"
            style={{ width: `${((currentQuestion + 1) / questions.length) * 100}%` }}
          />
        </div>
      </div>

      {/* Question */}
      <div className="mb-6">
        <p className="text-lg font-semibold text-gray-800 mb-4">{question.question}</p>

        {/* Options */}
        <div className="space-y-3">
          {question.options.map((option) => {
            const isSelected = selectedAnswer === option.id;
            const isCorrect = option.isCorrect;
            
            let buttonClass = 'w-full text-left p-4 rounded-lg border-2 transition-all ';
            
            if (!hasAnswered) {
              buttonClass += 'border-gray-300 hover:border-primary hover:bg-primary/5 cursor-pointer';
            } else {
              if (isSelected) {
                if (isCorrect) {
                  buttonClass += 'border-green-500 bg-green-50';
                } else {
                  buttonClass += 'border-red-500 bg-red-50';
                }
              } else if (isCorrect) {
                buttonClass += 'border-green-500 bg-green-50';
              } else {
                buttonClass += 'border-gray-300 bg-gray-50 opacity-50';
              }
            }

            return (
              <button
                key={option.id}
                type="button"
                onClick={() => handleAnswerSelect(option.id)}
                disabled={hasAnswered}
                className={buttonClass}
              >
                <div className="flex items-center justify-between">
                  <span className="text-sm font-medium text-gray-800">{option.text}</span>
                  {hasAnswered && (
                    <>
                      {isSelected && isCorrect && <CheckCircle className="text-green-600" size={24} />}
                      {isSelected && !isCorrect && <XCircle className="text-red-600" size={24} />}
                      {!isSelected && isCorrect && <CheckCircle className="text-green-600" size={24} />}
                    </>
                  )}
                </div>
              </button>
            );
          })}
        </div>
      </div>

      {/* Explanation */}
      {showExplanation && (
        <div className={`mb-6 p-4 rounded-lg ${
          question.options.find(opt => opt.id === selectedAnswer)?.isCorrect
            ? 'bg-green-50 border-2 border-green-300'
            : 'bg-red-50 border-2 border-red-300'
        }`}>
          <div className="flex items-start gap-3">
            {question.options.find(opt => opt.id === selectedAnswer)?.isCorrect ? (
              <CheckCircle className="text-green-600 flex-shrink-0 mt-1" size={20} />
            ) : (
              <XCircle className="text-red-600 flex-shrink-0 mt-1" size={20} />
            )}
            <div>
              <div className="font-semibold text-sm mb-1">
                {question.options.find(opt => opt.id === selectedAnswer)?.isCorrect ? 'Correct!' : 'Incorrect'}
              </div>
              <div className="text-sm text-gray-700">{question.explanation}</div>
            </div>
          </div>
        </div>
      )}

      {/* Navigation */}
      <div className="flex items-center justify-between">
        <div className="text-sm text-gray-600">
          Score: {score}/{answeredQuestions.length}
        </div>
        {hasAnswered && (
          <button
            type="button"
            onClick={handleNext}
            className="flex items-center gap-2 px-6 py-3 bg-primary text-white rounded-lg
                     hover:bg-primary-dark transition-colors font-semibold"
          >
            {isLastQuestion ? 'Finish' : 'Next Question'}
            <ChevronRight size={20} />
          </button>
        )}
      </div>
    </div>
  );
};

// Simple quiz question generator helper
// eslint-disable-next-line react-refresh/only-export-components
export const createQuizQuestion = (
  id: number,
  question: string,
  options: Array<{ text: string; isCorrect: boolean }>,
  explanation: string,
  difficulty: 'Easy' | 'Medium' | 'Hard' = 'Medium'
): QuizQuestion => {
  return {
    id,
    question,
    options: options.map((opt, index) => ({
      id: `option-${index}`,
      text: opt.text,
      isCorrect: opt.isCorrect
    })),
    explanation,
    difficulty
  };
};