import React, { useState, useEffect } from 'react'; // Import useEffect
import Layout from '@theme/Layout';
import ChatInput from '../components/ChatInput';
import ChatDisplay from '../components/ChatDisplay';
import { sendChatMessage, getChatHistory, submitFeedback } from '../services/chat_api'; // Import getChatHistory and submitFeedback

function ChatbotPage() {
  const [messages, setMessages] = useState([]);
  const [isBotTyping, setIsBotTyping] = useState(false); // New state for typing indicator
  const [isLoading, setIsLoading] = useState(false); // Existing state for input loading

  // Temporary user_id and session_id for demonstration
  // In a real application, these would come from user authentication/session management
  const userId = 'temp_user_id';
  const sessionId = 'temp_session_id';

  useEffect(() => {
    const fetchHistory = async () => {
      try {
        const history = await getChatHistory(userId, sessionId);
        // Ensure messages are in the correct format and sorted by timestamp
        const formattedHistory = history.map(msg => ({
          type: msg.type,
          text: msg.text,
          timestamp: new Date(msg.timestamp).toISOString(),
          response_id: msg.response_id, // Ensure response_id is carried over for feedback
        })).sort((a, b) => new Date(a.timestamp).getTime() - new Date(b.timestamp).getTime());
        setMessages(formattedHistory);
      } catch (error) {
        console.error('Error fetching chat history:', error);
        setMessages([
          { type: 'bot', text: 'Failed to load chat history.', timestamp: new Date().toISOString() },
        ]);
      }
    };
    fetchHistory();
  }, [userId, sessionId]); // Refetch when userId or sessionId changes

  const handleSendMessage = async (text) => {
    const newMessage = { type: 'user', text, timestamp: new Date().toISOString() };
    setMessages((prevMessages) => [...prevMessages, newMessage]);
    setIsLoading(true);
    setIsBotTyping(true); // Set bot as typing

    try {
      const response = await sendChatMessage(userId, text, sessionId);
      
      const botResponse = {
        type: 'bot',
        text: response.text_content, // Use the actual response content
        timestamp: response.timestamp,
        response_id: response.id, // Pass response_id for feedback
        sources: response.sources || [], 
      };
      setMessages((prevMessages) => [...prevMessages, botResponse]);
    } catch (error) {
      console.error('Error sending message:', error);
      setMessages((prevMessages) => [
        ...prevMessages,
        { type: 'bot', text: 'Sorry, something went wrong. Please try again.', timestamp: new Date().toISOString() },
      ]);
    } finally {
      setIsLoading(false);
      setIsBotTyping(false); // Bot stops typing
    }
  };

  const handleFeedbackSubmit = async (responseId, feedbackType, comment = null) => {
    try {
      await submitFeedback(responseId, feedbackType, comment);
      console.log(`Feedback submitted: ${feedbackType} for ${responseId}`);
      // Optionally, update the UI to show that feedback has been submitted
    } catch (error) {
      console.error('Error submitting feedback:', error);
      alert('Failed to submit feedback.');
    }
  };

  return (
    <Layout
      title="Chatbot"
      description="Interactive RAG Chatbot for your textbook"
    >
      <main className="container margin-vert--lg">
        <h1>Textbook Chatbot</h1>
        <div style={{ display: 'flex', flexDirection: 'column', height: '70vh', border: '1px solid #ddd', borderRadius: '8px' }}>
          <ChatDisplay messages={messages} onFeedbackSubmit={handleFeedbackSubmit} isBotTyping={isBotTyping} /> {/* Pass isBotTyping */}
          <ChatInput onSendMessage={handleSendMessage} isLoading={isLoading} />
        </div>
      </main>
    </Layout>
  );
}

export default ChatbotPage;
