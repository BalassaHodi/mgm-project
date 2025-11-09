import rclpy
from rclpy.node import Node


class TestNode(Node):

    def __init__(self):
        super().__init__("test_node")

        # Debug message
        self.get_logger().info("TestNode has been started.")


def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
