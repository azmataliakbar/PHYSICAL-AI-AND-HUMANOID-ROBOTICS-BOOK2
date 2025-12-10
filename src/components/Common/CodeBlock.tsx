// frontend/src/components/CodeBlock.tsx

import React, { useState } from 'react';
import { Copy, Check } from 'lucide-react';

interface CodeBlockProps {
  code: string;
  language?: string;
  filename?: string;
  showLineNumbers?: boolean;
}

export const CodeBlock: React.FC<CodeBlockProps> = ({
  code,
  language = 'python',
  filename,
  showLineNumbers = true
}) => {
  const [copied, setCopied] = useState(false);

  const handleCopy = async () => {
    try {
      await navigator.clipboard.writeText(code);
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    } catch (err) {
      console.error('Failed to copy code:', err);
    }
  };

  const getLanguageColor = (lang: string) => {
    const colors: { [key: string]: string } = {
      python: 'bg-blue-500',
      javascript: 'bg-yellow-500',
      typescript: 'bg-blue-600',
      bash: 'bg-gray-700',
      xml: 'bg-orange-500',
      cpp: 'bg-purple-600',
      c: 'bg-purple-700',
      yaml: 'bg-red-500',
    };
    return colors[lang.toLowerCase()] || 'bg-gray-500';
  };

  const lines = code.trim().split('\n');

  return (
    <div className="my-4 rounded-lg overflow-hidden border border-gray-300 shadow-sm">
      {/* Header */}
      <div className="bg-gray-800 text-white px-4 py-2 flex items-center justify-between">
        <div className="flex items-center gap-2">
          <span className={`w-2 h-2 rounded-full ${getLanguageColor(language)}`} />
          <span className="text-xs font-mono">
            {filename || language}
          </span>
        </div>
        <button
          type="button"
          onClick={handleCopy}
          className="flex items-center gap-1 px-2 py-1 text-xs rounded hover:bg-gray-700 
                   transition-colors"
          aria-label={copied ? 'Copied!' : 'Copy code'}
        >
          {copied ? (
            <>
              <Check size={14} />
              <span>Copied!</span>
            </>
          ) : (
            <>
              <Copy size={14} />
              <span>Copy</span>
            </>
          )}
        </button>
      </div>

      {/* Code Content */}
      <div className="bg-gray-900 text-gray-100 overflow-x-auto">
        <pre className="p-4">
          <code className="text-sm font-mono">
            {showLineNumbers ? (
              <table className="w-full border-separate border-spacing-0">
                <tbody>
                  {lines.map((line, index) => (
                    <tr key={index}>
                      <td className="text-gray-500 text-right pr-4 select-none align-top min-w-[3rem]">
                        {index + 1}
                      </td>
                      <td className="text-left whitespace-pre">
                        {line || '\n'}
                      </td>
                    </tr>
                  ))}
                </tbody>
              </table>
            ) : (
              code
            )}
          </code>
        </pre>
      </div>
    </div>
  );
};

// Terminal/Console output component
interface TerminalProps {
  children: React.ReactNode;
  title?: string;
}

export const Terminal: React.FC<TerminalProps> = ({ children, title = 'Terminal' }) => {
  return (
    <div className="my-4 rounded-lg overflow-hidden border border-gray-300 shadow-sm">
      {/* Terminal Header */}
      <div className="bg-gray-700 px-4 py-2 flex items-center gap-2">
        <div className="flex gap-1.5">
          <div className="w-3 h-3 rounded-full bg-red-500" />
          <div className="w-3 h-3 rounded-full bg-yellow-500" />
          <div className="w-3 h-3 rounded-full bg-green-500" />
        </div>
        <span className="text-xs text-gray-300 ml-2">{title}</span>
      </div>

      {/* Terminal Content */}
      <div className="bg-black text-green-400 p-4 font-mono text-sm overflow-x-auto">
        <pre>{children}</pre>
      </div>
    </div>
  );
};

// Inline code component
interface InlineCodeProps {
  children: React.ReactNode;
}

export const InlineCode: React.FC<InlineCodeProps> = ({ children }) => {
  return (
    <code className="px-1.5 py-0.5 bg-gray-100 text-red-600 rounded text-sm font-mono 
                   border border-gray-300">
      {children}
    </code>
  );
};