import time 
from langchain_ollama import ChatOllama
from langchain.agents import AgentExecutor
from langchain.agents.format_scratchpad.openai_tools import format_to_openai_tool_messages
from langchain.agents.output_parsers.openai_tools import OpenAIToolsAgentOutputParser
from langchain_core.messages import HumanMessage, AIMessage
from langchain.prompts import ChatPromptTemplate, MessagesPlaceholder
from .agent_prompts import system_prompts
from .agent_tools import get_tools
from typing import Optional


class KinovaAgent:
    """
    KinovaAgent is a ROS2 agent designed specifically to interact with
    the Kinova Robot using MoveIt2
        
    Args:
        tools (list): A list of tools defined to use with the agent.
        prompts: Prompts defining how the agent should respond.
        llm: A language model used for generating responses.
        streaming (bool): Decides if we want to stream the output of the agent. Defaults to True.
        verbose (bool): Decides if we want to build the verbose output. Defaults to False.
    """
    def __init__(
        self, 
        tools: Optional[list] = None, 
        prompts: Optional[ChatPromptTemplate] = None , 
        llm: Optional[ChatOllama] = None, 
        streaming: bool = True,
        record_chat_history: bool = True
    ):
        self.llm = llm
        self.tools = get_tools()
        self.prompts = self.get_prompts()
        self.streaming = streaming
        self.executor = self.get_executor()
        self.chat_history = []
        self.record_chat_history = record_chat_history
        
    def get_agent (self):
        """
        Create and return an Agent to process user input
        """
        
        agent = (
            {
                "input": lambda x: x["input"],
                "agent_scratchpad": lambda x: format_to_openai_tool_messages (x["intermediate_steps"]),
                "chat_history": lambda x: x["chat_history"]
            } | self.prompts | self.llm | OpenAIToolsAgentOutputParser ()
        )

        return agent
    
    def get_executor(self) -> AgentExecutor:
        """
        Creates and returns an executor to process the user commands and generate a response.

        Returns:
            AgentExecutor: Creates and returns an AgentExecutor Instance
        """
        agent = self.get_agent()
        executor = AgentExecutor(agent=agent, tools=self.tools)
        return executor
    
    
    def invoke (self, query: str) -> str:
        """
        Invoke the agent with the user query and return the agent's response

        Args:
            query (str): Users query to be processed

        Returns:
            str: Response generated by the agent to the user's query
        """
        try:
            commands = query.split(" then ")
            responses = []
            for command in commands:
                result = self.executor.invoke(
                    input={"input": command.strip(), "chat_history": self.chat_history}
                )
                
                # if result:
                #     self.accumulate_chat_history(query, result["output"])
                    
                response = result["output"]
                responses.append(str(response))
                print(f"\033[92mProcessed command: '{command.strip()}' -> Response: {response}\033[0m")
                final_response = "\n".join(responses)
            return final_response

        except Exception as e:
            return f"An Error occured while invoking the Agent: {e}"

            
    def get_prompts(self) -> ChatPromptTemplate:
        """
        Return a chat prompt template for the agent.

        Returns:
            ChatPromptTemplate: Template specifying the agent's response format.
        """
        prompts = ChatPromptTemplate.from_messages(
            system_prompts +
            [
                MessagesPlaceholder(variable_name="chat_history"),
                ("user", "{input}"),
                MessagesPlaceholder(variable_name="agent_scratchpad"),
            ]
        )
        return prompts
        
        
    def accumulate_chat_history(self, query: str, response: str):
        """
        Record the interaction between user and agent

        Args:
            query (str): User Query
            response (str): Agent Response
        """
        if self.record_chat_history:
            self.chat_history.extend([HumanMessage(content=query), AIMessage(content=response)])
        