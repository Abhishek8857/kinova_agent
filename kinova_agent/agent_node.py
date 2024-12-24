import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String
from .llm import get_llm
from .agent import KinovaAgent


class Agent(Node):
    def __init__(self):
        super().__init__("Kinova_Agent")
        qos_profile = QoSProfile(depth=1, durability=QoSDurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(String, "tool_query", qos_profile)
        self.subscription = self.create_subscription(String, "transcription_text", self.agent_callback , qos_profile)

        # Instantiate the Agent
        self.agent = KinovaAgent(llm=get_llm())
        
        self.get_logger().info("Kinova Agent initialised and waiting for messages ...")
        self.message_recieved = False

        
    def agent_callback (self, msg):
        """Callback function to process recieved transcription text"""
        self.get_logger().info("Waiting for message to be published ...")
        
        # Wait for message to be published
        if not self.message_recieved:
            self.message_recieved = True
        
        # Logger
        self.get_logger().info(f"Recieved message: {msg.data}")
        
        # Invoke the agent and handle the response
        try:
            self.get_logger().info(f"Invoking Agent with the query: [{msg.data}]")
            self.publisher.publish(msg)
            response = self.agent.invoke([msg.data])
            self.get_logger().info(f"Agent Response: {response}")
        except Exception as e:
            self.get_logger().error(f"Error executing tool: {e}")
        finally:
            self.message_recieved = False
            self.get_logger().info("Waiting for the next message...\n")


def main ():
    rclpy.init()
    kinova_node = Agent()
    try:
        rclpy.spin(kinova_node)
    except KeyboardInterrupt:
        kinova_node.get_logger().info("Keyboard interrupt received, shutting down...")
    finally:
        kinova_node.destroy_node()
        rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
