import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import os
import time

def read_light_states_from_file(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
        light_states = []
        for line in lines:
            # Assuming the data in the file is space-separated integers
            light_state = list(map(int, line.strip().split()))
            light_states.append(light_state)
    return light_states

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('pub')
    publisher = node.create_publisher(Int32MultiArray, 'four_way_state', 10)

    # Get the path of the current script
    script_path = os.path.dirname(os.path.abspath(__file__))

    # Navigate up three parent levels and then access the 'control' folder
    parent_folder = os.path.abspath(os.path.join(script_path, os.pardir, os.pardir, os.pardir))
    file_path = os.path.join(parent_folder, 'control', 'light_states.txt')  # Specify the filename

    lights = read_light_states_from_file(file_path)

    while rclpy.ok():
        for light_state in lights:
            # Ensure each line in the file has at least 3 integers
            if len(light_state) >= 3:
                msg = Int32MultiArray(data=light_state[:4]) # was 2
                node.get_logger().info('Publishing: {}'.format(msg.data))
                publisher.publish(msg)

                # Use the third integer in the line as the timeout time
                timeout_time = light_state[2]

                # Sleep for the specified timeout time
                time.sleep(timeout_time)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()