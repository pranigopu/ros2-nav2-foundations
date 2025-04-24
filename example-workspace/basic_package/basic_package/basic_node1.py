import rclpy
from rclpy.node import Node

class BasicNode1(Node):
    '''I initialise and then die due to shutdown.'''

    def __init__(self):
        super().__init__("basic_node1")
        self.get_logger().info("Initialised! YEAH!!")

def main():
    rclpy.init()
    node = BasicNode1()
    rclpy.shutdown()