import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
from serial.tools import list_ports

class SerialSend(Node):
    def __init__(self):
        super().__init__('serial_send')
        
        self.four_way_subscription = self.create_subscription(
            Int32MultiArray,
            'four_way_state',
            self.callback_four_way,
            10
        )
        self.three_way_subscription = self.create_subscription(
            Int32MultiArray,
            'three_way_state',
            self.callback_three_way,
            10
        )
        self.train_subscription = self.create_subscription(
            Int32MultiArray,
            'train_state',
            self.callback_train,
            10
        )
        self.aux_subscription = self.create_subscription(
            Int32MultiArray,
            'aux_state',
            self.callback_aux,
            10
        )

        # Get a list of available serial ports
        self.serial_ports = self.get_available_serial_ports()
        
        self.serial_objects = [serial.Serial(port, 9600, timeout=1) for port in self.serial_ports]

        # Check if any serial ports are available
        if not self.serial_ports:
            self.get_logger().error('No serial devices found')
        else:
            self.serial_objects = [serial.Serial(port, 9600, timeout=1) for port in self.serial_ports]
            self.get_logger().info(f'Serial devices opened successfully: {self.serial_ports}')

        print("Node Activated")

    def callback_four_way(self, msg):
        self.process_state_callback(msg, 0)

    def callback_three_way(self, msg):
        self.process_state_callback(msg, 1)

    def callback_train(self, msg):
        self.process_state_callback(msg, 2)

    def callback_aux(self, msg):
        self.process_state_callback(msg, 3)

    def process_state_callback(self, msg, serial_port_index):
        # Ensure the received message has at least 2 integers
        if len(msg.data) >= 2:
            # Create an array of 8 values
            serial_data = [0, 0, 0, 0, 0, 0, 0, 0]
            
            # Copy the first 2 integers from the message into the serial_data array
            serial_data[0] = msg.data[0]
            serial_data[1] = msg.data[1]
            
            # Send the same 8-integer message to all serial ports
            self.send_to_all_serial_ports(serial_data)

    def send_to_all_serial_ports(self, serial_data):
        # Convert the list of integers to a string and send it over all specified serial ports
        serial_str = f'{serial_data[0]} {serial_data[1]} {serial_data[2]} {serial_data[3]} {serial_data[4]} {serial_data[5]} {serial_data[6]} {serial_data[7]}\n'
        for serial_object in self.serial_objects:
            serial_object.write(serial_str.encode('utf-8'))
        
        # Log the sent values
        self.get_logger().info(f'Sent values to all ports: {serial_data}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialSend()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()