// frontend/src/components/Callout.tsx

import React from 'react';
import { 
  Lightbulb, 
  AlertTriangle, 
  Info, 
  Zap, 
  CheckCircle, 
  XCircle,
  BookOpen,
  Sparkles
} from 'lucide-react';

type CalloutType = 'tip' | 'warning' | 'note' | 'danger' | 'success' | 'info' | 'pro' | 'important';

interface CalloutProps {
  type?: CalloutType;
  title?: string;
  children: React.ReactNode;
}

export const Callout: React.FC<CalloutProps> = ({ 
  type = 'note', 
  title,
  children 
}) => {
  const configs = {
    tip: {
      icon: Lightbulb,
      bgColor: 'bg-blue-50',
      borderColor: 'border-blue-300',
      iconColor: 'text-blue-600',
      textColor: 'text-blue-900',
      titleColor: 'text-blue-800',
      defaultTitle: '💡 Tip'
    },
    warning: {
      icon: AlertTriangle,
      bgColor: 'bg-yellow-50',
      borderColor: 'border-yellow-300',
      iconColor: 'text-yellow-600',
      textColor: 'text-yellow-900',
      titleColor: 'text-yellow-800',
      defaultTitle: '⚠️ Warning'
    },
    note: {
      icon: Info,
      bgColor: 'bg-gray-50',
      borderColor: 'border-gray-300',
      iconColor: 'text-gray-600',
      textColor: 'text-gray-900',
      titleColor: 'text-gray-800',
      defaultTitle: '📝 Note'
    },
    danger: {
      icon: XCircle,
      bgColor: 'bg-red-50',
      borderColor: 'border-red-300',
      iconColor: 'text-red-600',
      textColor: 'text-red-900',
      titleColor: 'text-red-800',
      defaultTitle: '❌ Danger'
    },
    success: {
      icon: CheckCircle,
      bgColor: 'bg-green-50',
      borderColor: 'border-green-300',
      iconColor: 'text-green-600',
      textColor: 'text-green-900',
      titleColor: 'text-green-800',
      defaultTitle: '✅ Success'
    },
    info: {
      icon: Info,
      bgColor: 'bg-indigo-50',
      borderColor: 'border-indigo-300',
      iconColor: 'text-indigo-600',
      textColor: 'text-indigo-900',
      titleColor: 'text-indigo-800',
      defaultTitle: 'ℹ️ Information'
    },
    pro: {
      icon: Sparkles,
      bgColor: 'bg-purple-50',
      borderColor: 'border-purple-300',
      iconColor: 'text-purple-600',
      textColor: 'text-purple-900',
      titleColor: 'text-purple-800',
      defaultTitle: '✨ Pro Tip'
    },
    important: {
      icon: Zap,
      bgColor: 'bg-orange-50',
      borderColor: 'border-orange-300',
      iconColor: 'text-orange-600',
      textColor: 'text-orange-900',
      titleColor: 'text-orange-800',
      defaultTitle: '⚡ Important'
    }
  };

  const config = configs[type];
  const Icon = config.icon;
  const displayTitle = title || config.defaultTitle;

  return (
    <div 
      className={`my-4 p-4 rounded-lg border-l-4 ${config.bgColor} ${config.borderColor}`}
      role="note"
      aria-label={`${type} callout`}
    >
      <div className="flex gap-3">
        <div className={`flex-shrink-0 ${config.iconColor}`}>
          <Icon size={20} />
        </div>
        <div className="flex-1">
          {displayTitle && (
            <div className={`font-bold mb-1 ${config.titleColor}`}>
              {displayTitle}
            </div>
          )}
          <div className={`text-sm ${config.textColor}`}>
            {children}
          </div>
        </div>
      </div>
    </div>
  );
};

// Specific callout shortcuts
export const TipCallout: React.FC<Omit<CalloutProps, 'type'>> = (props) => (
  <Callout type="tip" {...props} />
);

export const WarningCallout: React.FC<Omit<CalloutProps, 'type'>> = (props) => (
  <Callout type="warning" {...props} />
);

export const NoteCallout: React.FC<Omit<CalloutProps, 'type'>> = (props) => (
  <Callout type="note" {...props} />
);

export const DangerCallout: React.FC<Omit<CalloutProps, 'type'>> = (props) => (
  <Callout type="danger" {...props} />
);

export const SuccessCallout: React.FC<Omit<CalloutProps, 'type'>> = (props) => (
  <Callout type="success" {...props} />
);

export const ProTipCallout: React.FC<Omit<CalloutProps, 'type'>> = (props) => (
  <Callout type="pro" {...props} />
);

export const ImportantCallout: React.FC<Omit<CalloutProps, 'type'>> = (props) => (
  <Callout type="important" {...props} />
);

// Exercise/Quiz Box
interface ExerciseBoxProps {
  title?: string;
  difficulty?: 'Easy' | 'Medium' | 'Hard';
  children: React.ReactNode;
}

export const ExerciseBox: React.FC<ExerciseBoxProps> = ({ 
  title = '✍️ Exercise',
  difficulty,
  children 
}) => {
  const difficultyColors = {
    Easy: 'bg-green-100 text-green-700',
    Medium: 'bg-yellow-100 text-yellow-700',
    Hard: 'bg-red-100 text-red-700'
  };

  return (
    <div className="my-4 p-4 rounded-lg border-2 border-primary/30 bg-primary/5">
      <div className="flex items-center justify-between mb-3">
        <div className="font-bold text-primary flex items-center gap-2">
          <BookOpen size={20} />
          {title}
        </div>
        {difficulty && (
          <span className={`text-xs px-2 py-1 rounded-full font-semibold ${difficultyColors[difficulty]}`}>
            {difficulty}
          </span>
        )}
      </div>
      <div className="text-sm text-gray-800">
        {children}
      </div>
    </div>
  );
};