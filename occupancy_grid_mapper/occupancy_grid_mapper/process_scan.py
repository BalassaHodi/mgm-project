import numpy as np

from local.OccupancyGridMap import OccupancyGridMap

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
from message_filters import Subscriber, ApproximateTimeSynchronizer


class ProcessScanNode(Node):

    def __init__(self):
        super().__init__("process_scan")
        self.get_logger().info("ProcessScanNode has been started.")

        # create the OccupancyGridMap object from the local module
        self.localGridMap = OccupancyGridMap(8.0, 8.0, 0.1)

        # create OccupancyGrid object from the ros message
        self.rosGridMap = OccupancyGrid()
        self.rosGridMap.header.frame_id = "odom"
        self.rosGridMap.info.resolution = self.localGridMap._resolution
        self.rosGridMap.info.width = self.localGridMap._gridCols
        self.rosGridMap.info.height = self.localGridMap._gridRows

        # the cell (0,0) is at the bottom left:
        self.rosGridMap.info.origin.position.x = -self.localGridMap._mapWidth / 2
        self.rosGridMap.info.origin.position.y = -self.localGridMap._mapHeight / 2
        self.rosGridMap.info.origin.position.z = 0.0
        self.rosGridMap.info.origin.orientation.w = 1.0

        # create message_filter subscriptions to LaserScan and Odometry messages
        self.scan_sub = Subscriber(self, LaserScan, "/scan")
        self.odom_sub = Subscriber(self, Odometry, "/odom")

        # create ApproximateTimeSynchronizer to synchronize the messages
        self.ts = ApproximateTimeSynchronizer(
            [self.scan_sub, self.odom_sub],
            queue_size=10,
            slop=0.1,
        )

        # create the callback for synchronized messages
        self.ts.registerCallback(self.synced_cb)

        # create publisher for OccupancyGrid messages
        self.gridmap_pub = self.create_publisher(OccupancyGrid, "/gridmap", 10)

    def synced_cb(self, scan_msg: LaserScan, odom_msg: Odometry):
        # get robot pose from odometry
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        q = [
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w,
        ]
        yaw = euler_from_quaternion(q)[2]

        robot_pose = [x, y, np.rad2deg(yaw)]

        # get ranges from the lidar scan
        for i, distance in enumerate(scan_msg.ranges):
            if scan_msg.range_min < distance < scan_msg.range_max:
                angle = scan_msg.angle_min + i * scan_msg.angle_increment

                lidar_data = [np.rad2deg(angle), distance]

                # process the scan
                self.localGridMap.process_scan(robot_pose, lidar_data)

        # update the OccupancyGrid message to publish
        self.rosGridMap.header.stamp = self.get_clock().now().to_msg()
        flat_gridmap = self.localGridMap.gridMap.flatten()
        self.rosGridMap.data = flat_gridmap.tolist()

        # publish the OccupancyGrid message
        self.gridmap_pub.publish(self.rosGridMap)

        # save the occupancy grid map to a txt file
        want_to_save = False  # change to True to save the map
        if want_to_save:
            with open(
                "/workspace/src/mgm-project/occupancy_grid_mapper/data/occupancy_grid_map.csv",
                "w",
            ) as f:
                for row in self.localGridMap.gridMap:
                    row_str = " ".join(str(cell) for cell in row)
                    f.write(row_str + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = ProcessScanNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
