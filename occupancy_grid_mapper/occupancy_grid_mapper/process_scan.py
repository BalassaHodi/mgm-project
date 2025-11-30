import rclpy
from rclpy.node import Node


class ProcessScan(Node):  

    def __init__(self):
        super().__init__("process_scan")

        self.get_logger().info("mukodik a processszken")


def main(args=None):
    rclpy.init(args=args)
    node = ProcessScan() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
