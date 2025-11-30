"""
This node reads the position of the robot, and from that
it calculates the maximum and minimum x and y values of its path.

The maximum and minimum values are displayed in the terminal.
"""

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point


class GetExtremeValuesNode(Node):

    def __init__(self):
        super().__init__("get_ext_val")
        self.get_logger().info("GetExtremeValuesNode has been started.")

        # variables
        self.max_point = Point(x=-9999.0, y=-9999.0)
        self.min_point = Point(x=9999.0, y=9999.0)
        self.prev_vals = [
            self.max_point.x,
            self.max_point.y,
            self.min_point.x,
            self.min_point.y,
        ]

        # subscription
        self.create_subscription(Odometry, "/odom", self.odom_cb, 1)

    def odom_cb(self, msg: Odometry):
        self.max_point.x = max(self.max_point.x, msg.pose.pose.position.x)
        self.max_point.y = max(self.max_point.y, msg.pose.pose.position.y)
        self.min_point.x = min(self.min_point.x, msg.pose.pose.position.x)
        self.min_point.y = min(self.min_point.y, msg.pose.pose.position.y)

        act_vals = [
            self.max_point.x,
            self.max_point.y,
            self.min_point.x,
            self.min_point.y,
        ]

        if self.prev_vals != act_vals:
            # self.get_logger().info(
            #     f"Max: [{self.max_point.x:.2f}, {self.max_point.y:.2f}]\tMin: [{self.min_point.x:.2f}, {self.min_point.y:.2f}]"
            # )

            self.prev_vals = [
                self.max_point.x,
                self.max_point.y,
                self.min_point.x,
                self.min_point.y,
            ]


def main(args=None):
    rclpy.init(args=args)
    node = GetExtremeValuesNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
