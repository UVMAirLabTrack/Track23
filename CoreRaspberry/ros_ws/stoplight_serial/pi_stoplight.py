import rclpy
from rclpy.node import Node
import serial

class ArduinoPublisher(Node):
    def __init__(self):
        super().__init__('stoplight_publisher')
        self.serial_port = '/dev/ttyUSB0'  # Replace with your actual port
        self.serial = serial.Serial(self.serial_port, 9600)  # Adjust baud rate if needed
        self.publisher_ = self.create_publisher(String, 'stoplight_topic', 10)

    def publish_data(self, data):
        msg = String()
        msg.data = data
        self.publisher_.publish(msg)

    def run(self):
        while rclpy.ok():
            data = self.serial.readline().decode().strip()  # Read data from Arduino
            self.publish_data(data)

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoPublisher()
    node.run()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()