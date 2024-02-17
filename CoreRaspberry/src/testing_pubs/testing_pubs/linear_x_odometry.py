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
        self.linear_speed = 0.5  # Adjust linear speed as needed
        self.distance_limit = 2.0  # Adjust distance limit as needed
        self.forward = True
    def publish_odom(self):
        self.time_sec += 0.2

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'

        # Set linear velocity
        odom.twist.twist.linear.x = self.linear_speed if self.forward else -self.linear_speed
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0

        # Update position
        elapsed_time = self.time_sec % (2 * self.distance_limit / self.linear_speed)
        if elapsed_time < self.distance_limit / self.linear_speed:
            # Moving forward
            odom.pose.pose.position.x = elapsed_time * self.linear_speed
        else:
            # Turning around
            odom.pose.pose.position.x = (2 * self.distance_limit - elapsed_time * self.linear_speed)

        # Set orientation (quaternion)
        odom.pose.pose.orientation = self.angle_to_quaternion(math.pi) if not self.forward else self.angle_to_quaternion(0.0)

        self.odom_publisher.publish(odom)

        # Check if the robot has reached the distance limit
        if elapsed_time >= 2 * self.distance_limit / self.linear_speed:
            self.forward = not self.forward

    def angle_to_quaternion(self, angle):
        return Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(angle / 2.0),
            w=math.cos(angle / 2.0))

def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdomPublisher()
    rclpy.spin(odom_publisher)
    odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()