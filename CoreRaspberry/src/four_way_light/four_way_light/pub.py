import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('pub')
    publisher = node.create_publisher(Int32MultiArray, 'four_way_state', 10)

    msg = Int32MultiArray()
    combinations = [(1, 2), (1, 4), (5, 6),(10,10)] 
    time_interval = 1


    while rclpy.ok():
        for combination in combinations:
        msg.data = f'Combination: {combination[0]}, {combination[1]}'
        node.get_logger().info('Publishing: {}'.format(msg.data))
        publisher.publish(msg)
        serial_data = f'{combination[0]} {combination[1]}\n'
        my_serial.write(serial_data.encode('utf-8'))

        print(f"Published: {msg.data}")
        rclpy.spin_once(node)
        rclpy.sleep(time_interval)

    my_serial.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()