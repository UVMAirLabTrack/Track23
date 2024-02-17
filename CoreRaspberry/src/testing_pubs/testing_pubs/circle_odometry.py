import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Vector3, TransformStamped
from visualization_msgs.msg import Marker
import tf2_ros
import math
import subprocess

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.marker_publisher = self.create_publisher(Marker, 'car_marker', 10)
        self.transform_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.2, self.publish_odom)
        self.time_sec = 0.0
        self.angle = 0.0
        self.radius = 1.0  # 1 meter circle

    def publish_odom(self):
        self.time_sec += 0.2
        self.angle += 0.03  # Adjust as needed for desired rotation speed

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'

        # Set position
        odom.pose.pose.position.x = self.radius * math.cos(self.angle)
        odom.pose.pose.position.y = self.radius * math.sin(self.angle)
        odom.pose.pose.position.z = 0.0

        # Set orientation (quaternion)
        odom.pose.pose.orientation = self.angle_to_quaternion(self.angle)

        # Set linear velocity
        odom.twist.twist.linear = Vector3(x=0.1, y=0.0, z=0.0)  # Adjust as needed

        self.odom_publisher.publish(odom)

        # Publish transform from 'odom' to 'base_link'
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = odom.pose.pose.position.z
        transform.transform.rotation = odom.pose.pose.orientation

        self.transform_broadcaster.sendTransform(transform)

        # Publish Marker for visualization
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'car'
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.pose.position = odom.pose.pose.position
        marker.pose.orientation = odom.pose.pose.orientation
        marker.scale.x = 1.0  # Scale of the mesh
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        full_file_path = f'file:///{subprocess.check_output(["ros2", "pkg", "prefix", "world_gen"]).decode("utf-8").strip()}/share/world_gen/markers/light.stl'
        marker.mesh_resource = 'package://world_gen/markers/light.stl'

    # Print the full file path for debugging
        print(f"Full file path for 'package://world_gen/markers/light.stl': {full_file_path}")

        self.marker_publisher.publish(marker)

    def angle_to_quaternion(self, angle):
        return Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(angle / 2.0),
            w=math.cos(angle / 2.0)
        )

def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdomPublisher()
    rclpy.spin(odom_publisher)
    odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()