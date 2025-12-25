import React, { useRef, useEffect } from 'react';
import './ChatDisplay.css'; // Assuming you'll create this CSS file
import SourceDisplay from './SourceDisplay'; // Import the new component

function ChatDisplay({ messages, onFeedbackSubmit, isBotTyping }) { // Add isBotTyping prop
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages, isBotTyping]); // Also scroll when typing status changes

  const handleFeedback = (responseId, feedbackType) => {
    if (onFeedbackSubmit) {
      onFeedbackSubmit(responseId, feedbackType);
    }
  };

  return (
    <div className="chat-display">
      {messages.map((msg, index) => (
        <div key={index} className={`chat-message ${msg.type}`}>
          <div className="message-bubble">
            <p>{msg.text}</p>
            {msg.type === 'bot' && msg.sources && msg.sources.length > 0 && (
              <SourceDisplay sources={msg.sources} />
            )}
            {msg.type === 'bot' && msg.response_id && (
              <div className="feedback-controls">
                <span>Was this helpful?</span>
                <button 
                  className="feedback-button" 
                  onClick={() => handleFeedback(msg.response_id, 'positive')}
                >
                  👍
                </button>
                <button 
                  className="feedback-button" 
                  onClick={() => handleFeedback(msg.response_id, 'negative')}
                >
                  👎
                </button>
              </div>
            )}
            <span className="message-timestamp">
              {msg.timestamp ? new Date(msg.timestamp).toLocaleTimeString() : ''}
            </span>
          </div>
        </div>
      ))}
      {isBotTyping && (
        <div className="chat-message bot typing-indicator">
          <div className="message-bubble">
            <p>Bot is typing<span>.</span><span>.</span><span>.</span></p>
          </div>
        </div>
      )}
      <div ref={messagesEndRef} />
    </div>
  );
}

export default ChatDisplay;
