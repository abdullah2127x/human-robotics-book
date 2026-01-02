'use client';

import { useState } from 'react';

interface CodeBlockProps {
  code: string;
  language?: string;
  output?: string;
}

const CodeBlock = ({ code, language = 'javascript', output }: CodeBlockProps) => {
  const [copied, setCopied] = useState(false);

  const handleCopy = () => {
    navigator.clipboard.writeText(code);
    setCopied(true);
    setTimeout(() => setCopied(false), 2000);
  };

  // Simple syntax highlighting
  const highlightCode = (code: string) => {
    return code
      .replace(/\b(const|let|var|function|return|if|else|for|while|import|export|from|async|await|class|extends|new|try|catch|finally)\b/g, '<span class="keyword">$1</span>')
      .replace(/\b(\w+)(?=\s*\()/g, '<span class="function">$1</span>')
      .replace(/'([^']*)'/g, '<span class="string">\'$1\'</span>')
      .replace(/"([^"]*)"/g, '<span class="string">"$1"</span>')
      .replace(/`([^`]*)`/g, '<span class="string">`$1`</span>')
      .replace(/\/\/.*$/gm, '<span class="comment">$&</span>')
      .replace(/\b(\d+)\b/g, '<span class="number">$1</span>');
  };

  return (
    <div className="code-block">
      <div className="code-header">
        <div className="code-lang">
          <div className="code-dots">
            <span className="code-dot red"></span>
            <span className="code-dot yellow"></span>
            <span className="code-dot green"></span>
          </div>
          <span style={{ marginLeft: '10px' }}>{language}</span>
        </div>
        <button
          className={`copy-btn ${copied ? 'copied' : ''}`}
          onClick={handleCopy}
        >
          {copied ? 'Copied!' : 'Copy'}
        </button>
      </div>
      <div className="code-content">
        <pre><code dangerouslySetInnerHTML={{ __html: highlightCode(code) }}></code></pre>
      </div>
      {output && (
        <div className="output-badge">
          Output: {output}
        </div>
      )}
    </div>
  );
};

interface FeatureCardProps {
  id: string;
  title: string;
  description: string;
  code: string;
  output?: string;
}

export default function FeatureCard({ id, title, description, code, output }: FeatureCardProps) {
  return (
    <div className="feature-card" id={id}>
      <div className="feature-header">
        <div className="feature-icon">
          {title.charAt(0).toUpperCase()}
        </div>
        <h3 className="feature-title">{title}</h3>
      </div>
      <p className="feature-description">{description}</p>
      <CodeBlock code={code} output={output} />
    </div>
  );
}
