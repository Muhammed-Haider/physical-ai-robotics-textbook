// frontend/src/services/chat_api.js

const API_BASE_URL = 'http://localhost:8000/api/v1'; // Replace with your actual backend URL

export async function sendChatMessage(userId, queryText, sessionId) {
  try {
    const response = await fetch(`${API_BASE_URL}/chat/query`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ user_id: userId, query_text: queryText, session_id: sessionId }),
    });

    if (!response.ok) {
      const errorData = await response.json();
      throw new Error(errorData.detail || 'Failed to send message');
    }

    const data = await response.json();
    return data;
  } catch (error) {
    console.error('Error in sendChatMessage:', error);
    throw error;
  }
}

export async function getChatHistory(userId, sessionId = null) {
  try {
    let url = `${API_BASE_URL}/chat/history?user_id=${userId}`;
    if (sessionId) {
      url += `&session_id=${sessionId}`;
    }

    const response = await fetch(url, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
      },
    });

    if (!response.ok) {
      const errorData = await response.json();
      throw new Error(errorData.detail || 'Failed to fetch chat history');
    }

    const data = await response.json();
    return data;
  } catch (error) {
    console.error('Error in getChatHistory:', error);
    throw error;
  }
}

export async function submitFeedback(responseId, feedbackType, comment = null) {
  try {
    const response = await fetch(`${API_BASE_URL}/chat/feedback`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ response_id: responseId, feedback_type: feedbackType, comment: comment }),
    });

    if (!response.ok) {
      const errorData = await response.json();
      throw new Error(errorData.detail || 'Failed to submit feedback');
    }

    const data = await response.json();
    return data;
  } catch (error) {
    console.error('Error in submitFeedback:', error);
    throw error;
  }
}
