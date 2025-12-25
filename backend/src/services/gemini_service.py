import os
import google.generativeai as genai
from typing import List, Optional
from backend.src.utils.logger import logger

class GeminiService:
    """
    Service for interacting with the Google Gemini API for embeddings and LLM.
    """
    def __init__(self):
        self.gemini_api_key = os.getenv("GEMINI_API_KEY")

        if not self.gemini_api_key:
            logger.error("GEMINI_API_KEY environment variable must be set.")
            raise ValueError(
                "GEMINI_API_KEY environment variable must be set."
            )
        try:
            genai.configure(api_key=self.gemini_api_key)
            self.embedding_model = genai.get_model("models/embedding-001")
            self.generative_model = genai.GenerativeModel("gemini-pro") # Or another suitable model
            logger.info("GeminiService initialized successfully.")
        except Exception as e:
            logger.critical(f"Failed to initialize Gemini client: {e}")
            raise

    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generates an embedding vector for the given text using Gemini's embedding model.
        """
        if not text:
            logger.warning("Attempted to generate embedding for empty text.")
            return []
        try:
            response = genai.embed_content(
                model="models/embedding-001",
                content=text,
                task_type="RETRIEVAL_DOCUMENT" # or RETRIEVAL_QUERY depending on usage
            )
            embedding = response['embedding']
            logger.debug(f"Generated embedding of size {len(embedding)} for text.")
            return embedding
        except Exception as e:
            logger.error(f"Failed to generate embedding: {e}")
            return []

    async def generate_text(self, prompt: str,  context: Optional[List[str]] = None) -> str:
        """
        Generates text using Gemini's generative model based on the prompt and optional context.
        """
        full_prompt = ""
        if context:
            full_prompt += "Context:\n" + "\n".join(context) + "\n\n"
        full_prompt += f"Question: {prompt}"

        logger.debug(f"Generating text for prompt: {prompt[:100]}...") # Log first 100 chars
        try:
            response = self.generative_model.generate_content(full_prompt)
            if response.text:
                logger.info("Text generation successful.")
                return response.text
            else:
                logger.warning("Gemini API returned an empty response text.")
                return ""
        except Exception as e:
            logger.error(f"Failed to generate text: {e}")
            return ""

    async def get_client(self):
        """
        Returns the initialized Gemini generative model client.
        """
        return self.generative_model
