import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped, Quaternion, Pose, Point, Vector3
from visualization_msgs.msg import Marker
from x_core2 import pose_strip, open_world_data
from custom_msgs.msg import WorldMarkers, MarkerLoc
import tf2_ros
import math
import subprocess
import multiprocessing
import numpy as np
from math import cos, sin, radians


class OdomSimplifier(Node):
    package_name = 'world_gen'
    def __init__(self, marker_name):
        self.refresh_rate = 5 #Hz
        super().__init__("odom_simplifier")

        # Create a subscriber to listen to the "odom" topic
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.last_received_time = self.get_clock().now()


    def odom_callback(self, msg):

        
        current_time = self.get_clock().now()
        
        if (current_time - self.last_received_time).nanoseconds >= 1e9/self.refresh_rate:  # 1 second/Hz refresh rate
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z

            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w

            pose = [x,y,z]
            orient = [qx,qy,qz,qw]
            print('-------------------------------------------------------------')

            print(f'Pose [X,Y,Z]: {pose}')
            print(f'Orientation [X,Y,Z,W]: {orient}')

def main(args=None):
    rclpy.init(args=args)
    odom_transformer = OdomSimplifier()
    rclpy.spin(odom_transformer)
    odom_transformer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()