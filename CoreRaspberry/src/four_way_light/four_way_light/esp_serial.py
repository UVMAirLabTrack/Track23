import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from serial.tools import list_ports
import serial

class SerialSend(Node):
    def __init__(self):
        super().__init__('serial_send')

        # Get a list of available serial ports
        self.serial_ports = self.get_available_serial_ports()

        # Check if any serial ports are available
        if not self.serial_ports:
            self.get_logger().error('No serial devices found')
        else:
            # Create serial objects for all available serial ports
            self.serial_objects = [serial.Serial(port, 9600, timeout=1) for port in self.serial_ports]
            self.get_logger().info(f'Serial devices opened successfully: {self.serial_ports}')

        # Create publishers for individual topics
        self.four_way_publisher = self.create_publisher(Int32MultiArray, 'four_way_serial_state', 10)
        self.three_way_publisher = self.create_publisher(Int32MultiArray, 'three_way_serial_state', 10)
        self.train_publisher = self.create_publisher(Int32MultiArray, 'train_serial_state', 10)
        self.aux_publisher = self.create_publisher(Int32MultiArray, 'aux_serial_state', 10)

        # Initialize last values for each state
        self.last_four_way_state = [0, 0]
        self.last_three_way_state = [0, 0]
        self.last_train_state = [0, 0]
        self.last_aux_state = [0, 0]

        print("Node Activated")

    def callback_four_way(self, msg):
        self.process_state_callback(msg, self.four_way_publisher, self.last_four_way_state)

    def callback_three_way(self, msg):
        self.process_state_callback(msg, self.three_way_publisher, self.last_three_way_state)

    def callback_train(self, msg):
        self.process_state_callback(msg, self.train_publisher, self.last_train_state)

    def callback_aux(self, msg):
        self.process_state_callback(msg, self.aux_publisher, self.last_aux_state)

    def process_state_callback(self, msg, publisher, last_state):
        # Ensure the received message has at least 2 integers
        if len(msg.data) >= 2:
            # Create an array of 8 values
            serial_data = [0, 0, 0, 0, 0, 0, 0, 0]

            # Copy the first 2 integers from the message into the serial_data array
            serial_data[0] = msg.data[0]
            serial_data[1] = msg.data[1]

            # Log the received data for debugging
            self.get_logger().info(f'Received data: {serial_data}')

            # Update the last state with the received values
            last_state[0] = serial_data[0]
            last_state[1] = serial_data[1]

            # Send the same 8-integer message to all serial ports
            self.send_to_all_serial_ports(serial_data)

            # Publish the received data on the appropriate serial state topic
            publisher.publish(Int32MultiArray(data=last_state))

    def send_to_all_serial_ports(self, serial_data):
        # Convert the list of integers to a string and send it over all specified serial ports
        serial_str = f'{serial_data[0]} {serial_data[1]} {serial_data[2]} {serial_data[3]} {serial_data[4]} {serial_data[5]} {serial_data[6]} {serial_data[7]}\n'
        for serial_object in self.serial_objects:
            serial_object.write(serial_str.encode('utf-8'))
        
        # Log the sent values
        self.get_logger().info(f'Sent values to all ports: {serial_data}')

    def get_available_serial_ports(self):
        available_ports = [port.device for port in list_ports.comports()]
        return available_ports

def main(args=None):
    rclpy.init(args=args)
    node = SerialSend()

    # Create subscriptions to the original topics
    node.create_subscription(Int32MultiArray, 'four_way_state', node.callback_four_way, 10)
    node.create_subscription(Int32MultiArray, 'three_way_state', node.callback_three_way, 10)
    node.create_subscription(Int32MultiArray, 'train_state', node.callback_train, 10)
    node.create_subscription(Int32MultiArray, 'aux_state', node.callback_aux, 10)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
