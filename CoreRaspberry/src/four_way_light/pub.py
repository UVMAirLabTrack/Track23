import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('pub')
    publisher = node.create_publisher(Int32MultiArray, 'four_way_state', 10)

    msg = Int32MultiArray()
    msg.data = [5, 8]  # Example values in the range of 1-10

    while rclpy.ok():
        node.get_logger().info('Publishing: {}'.format(msg.data))
        publisher.publish(msg)
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()