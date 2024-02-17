import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ColorPublisher(Node):
    def __init__(self):
        super().__init__('color_publisher')
        self.publisher_ = self.create_publisher(String, 'four_way_state', 10)
        self.timer_ = self.create_timer(3.0, self.timer_callback)
        self.colors = ['red', 'yellow', 'green', 'white']
        self.num_lights = 4

    def timer_callback(self):
        for light_index in range(self.num_lights):
            msg = String()
            msg.data = self.colors[light_index]
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    color_publisher = ColorPublisher()
    rclpy.spin(color_publisher)
    color_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()