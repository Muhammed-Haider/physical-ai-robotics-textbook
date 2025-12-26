from fastapi import FastAPI
from src.api import admin, chat

app = FastAPI()

app.include_router(admin.router)
app.include_router(chat.router)

@app.get("/")
def read_root():
    return {"message": "Welcome to the RAG Chatbot API"}
