import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('pub')
    publisher = node.create_publisher(Int32MultiArray, 'four_way_state', 10)

    msg = Int32MultiArray()
    msg2 = Int32MultiArray()
    msg3 = Int32MultiArray()
    msg.data = [5, 7] 
    msg2.data = [1, 3]
    msg3.data = [4, 2]
    time = 1

    while rclpy.ok():
        node.get_logger().info('Publishing: {}'.format(msg.data))
        publisher.publish(msg)
        rclpy.spin_once(node)
        rclpy.sleep(time)

        node.get_logger().info('Publishing: {}'.format(msg.data1))
        publisher.publish(msg2)
        rclpy.spin_once(node)
        rclpy.sleep(time)

        node.get_logger().info('Publishing: {}'.format(msg.data2))
        publisher.publish(msg3)
        rclpy.spin_once(node)
        rclpy.sleep(time)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()