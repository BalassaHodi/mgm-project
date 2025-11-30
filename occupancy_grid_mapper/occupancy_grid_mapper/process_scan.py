import rclpy
import math
import numpy as np
from local.OccupancyGridMap import OccupancyGridMap
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from tf_transformations import euler_from_quaternion


class ProcessScan(Node):  

    def __init__(self):
        super().__init__("process_scan")

        self.get_logger().info("mukodik a processszken")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ogm = OccupancyGridMap(16.0, 16.0, 0.1)

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.ogm_pub = self.create_publisher(OccupancyGrid, '/ogm', 1)

    def scan_callback(self, scan_msg: LaserScan):
        laser_frame = scan_msg.header.frame_id

        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'odom',                     
                laser_frame,                
                scan_msg.header.stamp,      
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(f'Nem sikerült TF-et lekérdezni: {e}')
            return

        # pozíció
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y

        # orientáció kvaternióból yaw
        q = transform.transform.rotation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        yaw_in_degrees = math.degrees(yaw)

        #lidar mérés feldolgozása (angle, ranges)
        for i in range(len(scan_msg.ranges)):
            
            if ((scan_msg.range_min < scan_msg.ranges[i]) and (scan_msg.ranges[i] < scan_msg.range_max)):
                angle = scan_msg.angle_min + scan_msg.angle_increment * i
                angle_in_degrees = math.degrees(angle)
                scan_distance = scan_msg.ranges[i]
                
                self.ogm.process_scan([tx, ty, yaw_in_degrees],[angle_in_degrees, scan_distance])
        
        grid = OccupancyGrid()
        grid.header = scan_msg.header
        grid.info.map_load_time = rclpy.time.Time().to_msg() #fingom nincs
        grid.info.resolution = self.ogm._resolution
        grid.info.width = self.ogm._gridCols
        grid.info.height = self.ogm._gridRows
        grid.info.origin.position.x = -(self.ogm._mapWidth/2) #bal alsó sarok (reméljük)
        grid.info.origin.position.y = -(self.ogm._mapHeight/2) #same in english
        grid.info.origin.position.z = 0
        grid.info.origin.orientation.w = 1.0 #asszem
        
        grid.data = self.ogm.gridMap.astype(np.int8).flatten(order="C").tolist()


        self.ogm_pub.publish(grid)        


def main(args=None):
    rclpy.init(args=args)
    node = ProcessScan() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
