from langchain_ollama import ChatOllama
from .agent_tools import get_tools

model = "llama3.1"
model_temperature = 0.0

def get_llm():
    return ChatOllama(model=model, 
                      temperature=model_temperature,
                      ).bind_tools(get_tools())


