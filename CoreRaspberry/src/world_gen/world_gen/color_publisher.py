import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class ColorPublisher(Node):
    def __init__(self):
        super().__init__('color_publisher')
        self.publisher_ = self.create_publisher(String, 'four_way_state', 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.colors = [['red', 'yellow', 'green', 'white'],
                       ['blue', 'red', 'white', 'green'],['green','blue', 'red', 'white']]  # Add more color sequences if needed
        self.num_lights = 4
        self.current_color_index = 0

    def timer_callback(self):
        msg = String()
        msg.data = ','.join(self.colors[self.current_color_index])
        self.publisher_.publish(msg)

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