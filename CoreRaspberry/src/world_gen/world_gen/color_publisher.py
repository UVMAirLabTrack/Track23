import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8MultiArray
import time

class ColorPublisher(Node):
    color_mapping = {
        'red': 0,
        'yellow': 1,
        'green': 2,
        'white': 3,
        'blue': 4,
        'off': 5,
        'left_stop': 6,
        'right_stop': 7,
        'left_go': 8,
        'right_go': 9,
    }

    def __init__(self):
        super().__init__('color_publisher')
        self.publisher_ = self.create_publisher(String, 'four_way_state_str', 10)
        self.publisher_2 = self.create_publisher(Int8MultiArray, 'four_way_state', 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.colors = [['red', 'yellow', 'off', 'white'],
                       ['white', 'blue', 'yellow', 'green'],
                       ['green', 'white', 'red', 'yellow'],
                       ['left_go', 'right_stop', 'right_go', 'left_stop']]
        self.colors_int = [[self.color_mapping[color] for color in row] for row in self.colors]
        self.num_lights = 4
        self.current_color_index = 0

    def timer_callback(self):
        # Publish colors as a string
        msg_str = String()
        msg_str.data = ','.join(self.colors[self.current_color_index])
        self.publisher_.publish(msg_str)

        # Publish colors as Int8MultiArray
        msg_int = Int8MultiArray()
        msg_int.data = [item for sublist in self.colors_int[self.current_color_index] for item in sublist]
        self.publisher_2.publish(msg_int)

        # Update the color index for the next iteration
        self.current_color_index = (self.current_color_index + 1) % len(self.colors)

def main(args=None):
    rclpy.init(args=args)
    color_publisher = ColorPublisher()
    rclpy.spin(color_publisher)
    color_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()