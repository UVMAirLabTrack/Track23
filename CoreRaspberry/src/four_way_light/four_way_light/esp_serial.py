import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial

class fourway(Node):
    def __init__(self):
        super().__init__('esp_serial')
        self.light_state_subscription = self.create_subscription(
            Int32MultiArray,
            'four_way_state',
            self.four_way_state_callback,
            10
        )
        self.serial_port = '/dev/ttyUSB0'  # Adjust this based on your serial port
        self.serial = serial.Serial(self.serial_port, 9600)

    def light_state_callback(self, msg):
        if len(msg.data) == 2:
            pair1 = msg.data[0]
            pair2 = msg.data[1]
            # Convert 1-10 range to 0-255 range
            pair1 = int((pair1 - 1) * 25.5)
            pair2 = int((pair2 - 1) * 25.5)
            # Send the values to Arduino via serial
            self.set_led_color(pair1, pair2)

    def set_led_color(self, pair1, pair2):
        # Send the data over serial
        self.serial.write(f'{pair1} {pair2}\n'.encode())
        self.get_logger().info(f'Sent values to 4 way light: Pair1={pair1}, Pair2={pair2}')

def main(args=None):
    rclpy.init(args=args)
    node = fourway()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()