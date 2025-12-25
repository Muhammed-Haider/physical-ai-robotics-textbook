import os
import google.generativeai as genai
from typing import List, Optional

class GeminiService:
    """
    Service for interacting with the Google Gemini API for embeddings and LLM.
    """
    def __init__(self):
        self.gemini_api_key = os.getenv("GEMINI_API_KEY")

        if not self.gemini_api_key:
            raise ValueError(
                "GEMINI_API_KEY environment variable must be set."
            )

        genai.configure(api_key=self.gemini_api_key)
        self.embedding_model = genai.get_model("models/embedding-001")
        self.generative_model = genai.GenerativeModel("gemini-pro") # Or another suitable model

    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generates an embedding vector for the given text using Gemini's embedding model.
        """
        if not text:
            return []
        response = genai.embed_content(
            model="models/embedding-001",
            content=text,
            task_type="RETRIEVAL_DOCUMENT" # or RETRIEVAL_QUERY depending on usage
        )
        return response['embedding']

    async def generate_text(self, prompt: str,  context: Optional[List[str]] = None) -> str:
        """
        Generates text using Gemini's generative model based on the prompt and optional context.
        """
        full_prompt = ""
        if context:
            full_prompt += "Context:\n" + "\n".join(context) + "\n\n"
        full_prompt += f"Question: {prompt}"

        response = self.generative_model.generate_content(full_prompt)
        return response.text

    async def get_client(self):
        """
        Returns the initialized Gemini generative model client.
        """
        return self.generative_model
