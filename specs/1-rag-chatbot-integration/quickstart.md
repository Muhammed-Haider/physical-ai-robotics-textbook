# Quickstart Guide: Interactive RAG Chatbot

This guide provides a quick overview of how to interact with the RAG Chatbot as an end-user and how to perform basic administrative tasks.

## 1. End-User Interaction

### 1.1 Asking Questions
To get answers from the chatbot:
1.  Navigate to the chatbot interface on the textbook website.
2.  Type your natural language question into the chat input field (e.g., "What are the main principles of ROS 2 communication?").
3.  Press Enter or click the send button.
4.  The chatbot will process your query and display a response in the conversational thread.

### 1.2 Viewing Source Documents
If available, the chatbot's response may include references to the source documents used to formulate the answer.
1.  Click on the provided links or indicators within the chatbot's response to view the relevant sections of the textbook.

### 1.3 Submitting Feedback
You can provide feedback on the chatbot's responses to help improve its accuracy and relevance.
1.  Look for a feedback mechanism (e.g., a thumbs up/down icon or a feedback button) associated with a chatbot response.
2.  Select the appropriate feedback type (e.g., positive, negative, irrelevant).
3.  Optionally, add a comment to provide more details.

## 2. Administrator Tasks

### 2.1 Configuring Documentation Sources
Administrators can configure which documentation sources the RAG chatbot uses.
1.  Access the administrative interface (details TBD).
2.  Navigate to the "Documentation Sources" section.
3.  Add a new source by providing a name, type (e.g., local directory, GitHub repository), and its path/URL.
4.  Existing sources can be updated or deleted from this section.

### 2.2 Triggering Manual Re-indexing
While re-indexing is periodic, administrators can manually trigger a full re-index of all active documentation sources.
1.  Access the administrative interface.
2.  Navigate to the "Indexing" or "Data Management" section.
3.  Initiate the "Re-index All Sources" action.

### 2.3 Viewing System Logs
Administrators can view logs to monitor the RAG service's operation and troubleshoot issues.
1.  Access the administrative interface.
2.  Navigate to the "System Logs" section.
3.  Logs can typically be filtered by level (INFO, WARNING, ERROR, DEBUG) and time range.
