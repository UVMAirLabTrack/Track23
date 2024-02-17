import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Vector3
import math

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
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