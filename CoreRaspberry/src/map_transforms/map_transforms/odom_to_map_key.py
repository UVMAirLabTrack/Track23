import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Pose, Point, Vector3
from visualization_msgs.msg import Marker
import tf2_ros
import math
import subprocess
import threading
from pynput import keyboard

class OdomTransformer(Node):
    def __init__(self):
        super().__init__('odom_transformer')

        # Create a subscriber to listen to the "odom" topic
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        self.offset_position = [0.0, 0.0, 0.0]
        self.offset_orientation = Quaternion()

        # Create a publisher to publish the transformed odometry data to "odom_map"
        self.odom_map_publisher = self.create_publisher(Odometry, 'odom_map', 10)

        # Create a publisher to publish the car model mesh
        self.car_mesh_publisher = self.create_publisher(Marker, 'car_mesh', 10)

        # Create a tf2_ros.TransformBroadcaster for publishing the transform
        self.transform_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Add a flag to check if the offset has been set
        self.offset_set = False

        key_listener_thread = threading.Thread(target=self.key_listener_thread)
        key_listener_thread.daemon = True  # Terminate thread when the main program exits
        key_listener_thread.start()

    def transform_odom(self, odom_msg):
        # Assuming you have the transformation logic here
        # In this example, we simply copy the original odometry message

        if not self.offset_set:
            # Set the car's start position and pose as zeros
            self.capture_offset(odom_msg)
            print("Car's start position and pose set to zeros.")

        transformed_odom = Odometry()
        transformed_odom.header = odom_msg.header
        transformed_odom.child_frame_id = 'base_link'  # Set to the appropriate child frame id

        # Perform the transformation from "base_link" to "map"
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'

        # Set the translation
        transform.transform.translation.x = odom_msg.pose.pose.position.x
        transform.transform.translation.y = odom_msg.pose.pose.position.y
        transform.transform.translation.z = odom_msg.pose.pose.position.z

        # Set the orientation (quaternion)
        transform.transform.rotation = self.angle_to_quaternion(0.0)  # Adjust as needed

        # Publish the transform
        self.transform_broadcaster.sendTransform(transform)

        # Transform the odometry data
        transformed_odom.pose.pose.position = odom_msg.pose.pose.position
        transformed_odom.pose.pose.orientation = transform.transform.rotation

        return transformed_odom

    
    def publish_car_mesh(self, odom_msg):
        # Publish the car model mesh in the transformed frame
        car_mesh = Marker()
        car_mesh.header.frame_id = 'base_link'  # Set to the transformed frame
        car_mesh.header.stamp = self.get_clock().now().to_msg()
        car_mesh.ns = 'car_mesh'
        car_mesh.id = 0
        car_mesh.type = Marker.MESH_RESOURCE
        car_mesh.action = Marker.ADD

        # Set the position as needed
        car_mesh.pose.position.x = 0.0
        car_mesh.pose.position.y = 0.0
        car_mesh.pose.position.z = 0.0

        # Set the orientation as needed
        car_mesh.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

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

    def angle_to_quaternion(self, angle):
        # Convert angle to quaternion
        return Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(angle / 2.0),
            w=math.cos(angle / 2.0)
        )
    def odom_callback(self, msg):
        # Check if the offset has been set
        if not self.offset_set:
            # Set the car's start position and pose as zeros
            self.set_offset(msg)
            print("Car's start position and pose set to zeros.")

        # Perform the transformation from "base_link" to "map"
        transformed_odom = self.transform_odom(msg)

        # Publish the transformed odometry data to "odom_map"
        self.odom_map_publisher.publish(transformed_odom)

        # Publish the car model mesh in the transformed frame
        self.publish_car_mesh(transformed_odom)

    def capture_offset(self, odom_msg):
        # Capture the car's current pose as the offset
        self.offset_position = [
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z
        ]
        self.offset_orientation = odom_msg.pose.pose.orientation

        # Set the offset_set flag to True
        self.offset_set = True

    def odom_callback(self, msg):
        # Check if the offset has been set
        if not self.offset_set:
            # Capture the car's current pose as the offset
            self.capture_offset(msg)
            print("Offset captured.")
        transformed_odom = self.transform_odom(msg)

    def on_key_press(self, key):
        # Check for the space key
        if key == keyboard.Key.space:
            # Reset the offset in a thread-safe way
            with self.offset_lock:
                self.offset_set = False
            print("Offset reset.")

    def key_listener_thread(self):
        with keyboard.Listener(on_press=self.on_key_press) as listener:
            listener.join()
        
def main(args=None):
    rclpy.init(args=args)
    odom_transformer = OdomTransformer()
    rclpy.spin(odom_transformer)
    odom_transformer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()