import rclpy
from rclpy.node import Node

class BasicNode3(Node):
    '''I initialise and stay alive thanks to `rclpy.spin` until you kill me, all the while executing my timer.'''

    def __init__(self):
        super().__init__("basic_node3")
        self.get_logger().info("Initialised! YEAH!!")
        self.create_timer(1.0, self.simple_callback)
        self.i = 0
    
    def simple_callback(self):
        self.get_logger().info(f"{self.i}")
        self.i += 1

def main():
    rclpy.init()
    node = BasicNode3()
    rclpy.spin(node)
    rclpy.shutdown()