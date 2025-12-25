import React from 'react';
import './SourceDisplay.css'; // Assuming you'll create this CSS file

function SourceDisplay({ sources }) {
  if (!sources || sources.length === 0) {
    return null;
  }

  return (
    <div className="source-display">
      <h4>Sources:</h4>
      <ul>
        {sources.map((source) => (
          <li key={source.document_id}>
            <a href={source.url} target="_blank" rel="noopener noreferrer">
              <strong>{source.title}</strong>
            </a>
            <p className="source-excerpt">{source.excerpt}</p>
          </li>
        ))}
      </ul>
    </div>
  );
}

export default SourceDisplay;
