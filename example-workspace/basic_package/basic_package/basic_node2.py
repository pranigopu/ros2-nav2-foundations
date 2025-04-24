import rclpy
from rclpy.node import Node

class BasicNode2(Node):
    '''I initialise and stay alive thanks to `rclpy.spin`, until you kill me.'''

    def __init__(self):
        super().__init__("basic_node2")
        self.get_logger().info("Initialised! YEAH!!")

def main():
    rclpy.init()
    node = BasicNode2()
    rclpy.spin(node) # Keeps the given node alive (until termination) and runs all its callbacks
    rclpy.shutdown()