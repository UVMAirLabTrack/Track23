import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped, Quaternion, Pose, Point, Vector3
from visualization_msgs.msg import Marker
from x_core2 import pose_strip, open_world_data,formulas
import tf2_ros
import math
import subprocess
import numpy as np
from math import cos, sin, radians
#import pynput as keyboard

class OdomTransformer(Node):
    def __init__(self):
        self.refresh_rate = 5 #Hz
        super().__init__('odom_transformer')

        # Create a subscriber to listen to the "odom" topic
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        self.reset_subscriber = self.create_subscription(Bool,"reset_car",self.reset_callback,10)
        self.odom_cap = False
        self.last_received_time = self.get_clock().now()
        self.transformed_odom = Odometry()
        self.saved_odom = Odometry()

        # Create a publisher to publish the transformed odometry data to "odom_map"
        self.odom_map_publisher = self.create_publisher(Odometry, 'odom_map', 10)

        # Create a publisher to publish the car model mesh
        self.car_mesh_publisher = self.create_publisher(Marker, 'car_mesh', 10)

        # Create a tf2_ros.TransformBroadcaster for publishing the transform
        self.transform_broadcaster = tf2_ros.TransformBroadcaster(self)

    def reset_callback(self,msg):
        if msg.data == True:
            self.odom_cap = True
            print(f'Reset Active')
        else:
            self.odom_cap = False
            print(f'Reset Finished')
            

    def odom_callback(self, msg):
        # Perform the transformation from "base_link" to "map"
        
        current_time = self.get_clock().now()
        
        if (current_time - self.last_received_time).nanoseconds >= 1e9/self.refresh_rate:  # 1 second/Hz refresh rate
            if self.odom_cap==True:
                self.saved_odom = msg
            transformed_odom = self.transform_odom(msg)
            self.odom_map_publisher.publish(transformed_odom)
            self.last_received_time = current_time

        # Publish the transformed odometry data to "odom_map"
        

        # Publish the car model mesh in the transformed frame
            self.publish_car_mesh(transformed_odom)

    def transform_odom(self, odom_msg):
        world_yaw = 0.0  #no X and Y angles.  Need to pull these frames from the custom pose stuff
        world_x = 0.0
        world_y = 0.0
        world_z = 0.0

        car_x = odom_msg.pose.pose.position.x
        car_y = odom_msg.pose.pose.position.y
        car_z = odom_msg.pose.pose.position.z
        car_euler = pose_strip.strip_return_euler_odom(odom_msg)

        ref_x = self.saved_odom.pose.pose.position.x
        ref_y = self.saved_odom.pose.pose.position.y
        ref_z = self.saved_odom.pose.pose.position.z
        ref_euler = pose_strip.strip_return_euler_odom(self.saved_odom)

        translation_vector = np.array([world_x - car_x-ref_x, world_y - car_y-ref_y])

        theta = radians(world_yaw)

        rotation_matrix = np.array([[cos(theta), -sin(theta)],
                                     [sin(theta), cos(theta)]])
        
        transform = np.dot(rotation_matrix, translation_vector)

        car_coord_x = transform[0]+ world_x
        car_coord_y = transform[1] + world_y

        E=[0,0,0]

        E[0] = car_euler[0]-ref_euler[0]
        E[1] = car_euler[1]-ref_euler[1]
        E[2] = car_euler[2]-ref_euler[2]

        car_quat = pose_strip.compute_quat(E)
        Q= car_quat


        transformed_odom = Odometry()
        transformed_odom.header = odom_msg.header
        transformed_odom.child_frame_id = 'base_footprint'  # Set to the appropriate child frame id

        # Perform the transformation from "base_link" to "map"
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_footprint'


        transform.transform.translation.x = car_coord_x
        transform.transform.translation.y = car_coord_y
        transform.transform.translation.z = world_z
        
        # Publish the transform
        #self.transform_broadcaster.sendTransform(transform)

        # Transform the odometry data
        transformed_odom.pose.pose.position.x = transform.transform.translation.x
        transformed_odom.pose.pose.position.y = transform.transform.translation.y
        transformed_odom.pose.pose.position.z = transform.transform.translation.z

        

        transformed_odom.pose.pose.orientation.x = Q[0]
        transformed_odom.pose.pose.orientation.y = Q[1]
        transformed_odom.pose.pose.orientation.z = Q[2]
        transformed_odom.pose.pose.orientation.w = Q[3]
        #print(f'Euler Ref: {a}')
        #print(f'Euler CT: {b}')
        #print(f'Euler Comb: {c}')

        #print(f'Quaternion: {Q[0]} {Q[1]} {Q[2]} {Q[3]}')

        return transformed_odom

    
    def publish_car_mesh(self, odom_msg):
        # Publish the car model mesh in the transformed frame
        car_mesh = Marker()
        car_mesh.header.frame_id = 'map' # 'base_footprint'  # Set to the transformed frame
        car_mesh.header.stamp = self.get_clock().now().to_msg()
        car_mesh.ns = 'car_mesh'
        car_mesh.id = 0
        car_mesh.type = Marker.MESH_RESOURCE
        car_mesh.action = Marker.ADD

        # Set the position as needed
        #car_mesh.pose.position.x = 0.0 #old version that worked, somehow?
        #car_mesh.pose.position.y = 0.0
        #car_mesh.pose.position.z = 0.0

        car_mesh.pose.position.x = odom_msg.pose.pose.position.x
        car_mesh.pose.position.y = odom_msg.pose.pose.position.y
        car_mesh.pose.position.z = odom_msg.pose.pose.position.z

        # Set the orientation as needed
        car_mesh.pose.orientation = odom_msg.pose.pose.orientation

        # Set the scale as needed
        car_mesh.scale.x = 1.0
        car_mesh.scale.y = 1.0
        car_mesh.scale.z = 1.0

        car_mesh.color.a = 1.0
        car_mesh.color.r = 1.0
        car_mesh.color.g = 0.0
        car_mesh.color.b = 0.0

        # Set the mesh resource file path
        full_file_path = f'file:///{subprocess.check_output(["ros2", "pkg", "prefix", "map_transforms"]).decode("utf-8").strip()}/share/map_transforms/markers/car.stl'
        #marker.mesh_resource = 'package://testing_pubs/markers/car.stl'
        car_mesh.mesh_resource = full_file_path

        #car_mesh.mesh_resource = 'package://map_transforms/markers/car.stl'  # Update with your mesh file path

        self.car_mesh_publisher.publish(car_mesh)


def main(args=None):
    rclpy.init(args=args)
    odom_transformer = OdomTransformer()
    rclpy.spin(odom_transformer)
    odom_transformer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()