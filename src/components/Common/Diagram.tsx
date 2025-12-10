// frontend/src/components/Diagram.tsx

import React from 'react';
import { Maximize2 } from 'lucide-react';

interface DiagramProps {
  title?: string;
  children: React.ReactNode;
  caption?: string;
  fullWidth?: boolean;
}

export const Diagram: React.FC<DiagramProps> = ({
  title,
  children,
  caption,
  fullWidth = false
}) => {
  return (
    <div className={`my-6 ${fullWidth ? '' : 'max-w-3xl mx-auto'}`}>
      {title && (
        <div className="text-sm font-semibold text-gray-700 mb-2 flex items-center gap-2">
          <Maximize2 size={16} className="text-primary" />
          {title}
        </div>
      )}
      <div className="bg-white border-2 border-gray-300 rounded-lg p-6 shadow-sm">
        {children}
      </div>
      {caption && (
        <div className="text-xs text-gray-500 mt-2 text-center italic">
          {caption}
        </div>
      )}
    </div>
  );
};

// ASCII/Text-based diagram component
interface TextDiagramProps {
  content: string;
  title?: string;
}

export const TextDiagram: React.FC<TextDiagramProps> = ({ content, title }) => {
  return (
    <Diagram title={title}>
      <pre className="text-sm font-mono text-gray-800 overflow-x-auto whitespace-pre">
        {content}
      </pre>
    </Diagram>
  );
};

// Simple flowchart component
interface FlowchartNode {
  id: string;
  label: string;
  type: 'start' | 'process' | 'decision' | 'end';
}

interface FlowchartProps {
  nodes: FlowchartNode[];
  title?: string;
}

export const Flowchart: React.FC<FlowchartProps> = ({ nodes, title }) => {
  const getNodeStyle = (type: string) => {
    switch (type) {
      case 'start':
      case 'end':
        return 'rounded-full bg-green-100 border-green-500';
      case 'decision':
        return 'rotate-45 bg-yellow-100 border-yellow-500';
      case 'process':
      default:
        return 'rounded-lg bg-blue-100 border-blue-500';
    }
  };

  return (
    <Diagram title={title}>
      <div className="flex flex-col items-center gap-4">
        {nodes.map((node, index) => (
          <div key={node.id} className="flex flex-col items-center">
            <div
              className={`px-6 py-4 border-2 min-w-[150px] text-center font-semibold text-sm
                        ${getNodeStyle(node.type)}`}
            >
              <div className={node.type === 'decision' ? '-rotate-45' : ''}>
                {node.label}
              </div>
            </div>
            {index < nodes.length - 1 && (
              <div className="w-0.5 h-8 bg-gray-400" />
            )}
          </div>
        ))}
      </div>
    </Diagram>
  );
};

// Architecture diagram component
interface ArchitectureLayer {
  name: string;
  components: string[];
  color: string;
}

interface ArchitectureDiagramProps {
  layers: ArchitectureLayer[];
  title?: string;
}

export const ArchitectureDiagram: React.FC<ArchitectureDiagramProps> = ({ layers, title }) => {
  return (
    <Diagram title={title} fullWidth>
      <div className="space-y-4">
        {layers.map((layer, index) => (
          <div key={index}>
            <div className={`${layer.color} p-4 rounded-lg border-2 border-gray-300`}>
              <div className="font-bold text-sm mb-3">{layer.name}</div>
              <div className="grid grid-cols-2 md:grid-cols-3 lg:grid-cols-4 gap-2">
                {layer.components.map((component, idx) => (
                  <div
                    key={idx}
                    className="bg-white px-3 py-2 rounded border border-gray-300 text-xs text-center
                             font-medium text-gray-700"
                  >
                    {component}
                  </div>
                ))}
              </div>
            </div>
            {index < layers.length - 1 && (
              <div className="flex justify-center my-2">
                <div className="text-gray-400">↓</div>
              </div>
            )}
          </div>
        ))}
      </div>
    </Diagram>
  );
};

// Timeline diagram
interface TimelineEvent {
  label: string;
  description: string;
}

interface TimelineDiagramProps {
  events: TimelineEvent[];
  title?: string;
}

export const TimelineDiagram: React.FC<TimelineDiagramProps> = ({ events, title }) => {
  return (
    <Diagram title={title}>
      <div className="space-y-6">
        {events.map((event, index) => (
          <div key={index} className="flex gap-4">
            <div className="flex flex-col items-center">
              <div className="w-4 h-4 rounded-full bg-primary border-4 border-white shadow" />
              {index < events.length - 1 && (
                <div className="w-0.5 flex-1 bg-primary/30 mt-2" />
              )}
            </div>
            <div className="flex-1 pb-6">
              <div className="font-semibold text-sm text-gray-800 mb-1">{event.label}</div>
              <div className="text-xs text-gray-600">{event.description}</div>
            </div>
          </div>
        ))}
      </div>
    </Diagram>
  );
};

// Comparison table
interface ComparisonItem {
  feature: string;
  option1: string;
  option2: string;
}

interface ComparisonTableProps {
  title?: string;
  option1Name: string;
  option2Name: string;
  items: ComparisonItem[];
}

export const ComparisonTable: React.FC<ComparisonTableProps> = ({
  title,
  option1Name,
  option2Name,
  items
}) => {
  return (
    <Diagram title={title} fullWidth>
      <div className="overflow-x-auto">
        <table className="w-full text-sm">
          <thead>
            <tr className="border-b-2 border-gray-300">
              <th className="text-left p-3 font-bold text-gray-700">Feature</th>
              <th className="text-left p-3 font-bold text-primary">{option1Name}</th>
              <th className="text-left p-3 font-bold text-primary">{option2Name}</th>
            </tr>
          </thead>
          <tbody>
            {items.map((item, index) => (
              <tr key={index} className="border-b border-gray-200 hover:bg-gray-50">
                <td className="p-3 font-semibold text-gray-700">{item.feature}</td>
                <td className="p-3 text-gray-600">{item.option1}</td>
                <td className="p-3 text-gray-600">{item.option2}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </Diagram>
  );
};

// Simple network diagram
interface NetworkNode {
  id: string;
  label: string;
  type: 'server' | 'client' | 'database' | 'service';
}

interface NetworkDiagramProps {
  nodes: NetworkNode[];
  title?: string;
}

export const NetworkDiagram: React.FC<NetworkDiagramProps> = ({ nodes, title }) => {
  const getNodeIcon = (type: string) => {
    switch (type) {
      case 'server': return '🖥️';
      case 'client': return '💻';
      case 'database': return '🗄️';
      case 'service': return '⚙️';
      default: return '📦';
    }
  };

  const getNodeColor = (type: string) => {
    switch (type) {
      case 'server': return 'bg-blue-100 border-blue-500';
      case 'client': return 'bg-green-100 border-green-500';
      case 'database': return 'bg-purple-100 border-purple-500';
      case 'service': return 'bg-orange-100 border-orange-500';
      default: return 'bg-gray-100 border-gray-500';
    }
  };

  return (
    <Diagram title={title}>
      <div className="grid grid-cols-2 md:grid-cols-3 gap-6">
        {nodes.map((node) => (
          <div key={node.id} className="flex flex-col items-center">
            <div
              className={`w-24 h-24 rounded-lg border-2 flex flex-col items-center justify-center
                        ${getNodeColor(node.type)} transition-transform hover:scale-105`}
            >
              <div className="text-3xl mb-1">{getNodeIcon(node.type)}</div>
              <div className="text-xs font-semibold text-gray-700">{node.label}</div>
            </div>
          </div>
        ))}
      </div>
    </Diagram>
  );
};