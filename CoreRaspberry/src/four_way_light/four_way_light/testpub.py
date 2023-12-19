import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import time

class StatePublisher(Node):
    def __init__(self):
        super().__init__('state_publisher')
        
        self.four_way_publisher = self.create_publisher(Int32MultiArray, 'four_way_state', 10)
        self.three_way_publisher = self.create_publisher(Int32MultiArray, 'three_way_state', 10)
        self.train_publisher = self.create_publisher(Int32MultiArray, 'train_state', 10)
        self.aux_publisher = self.create_publisher(Int32MultiArray, 'aux_state', 10)

        self.timer = self.create_timer(5.0, self.timer_callback)
        self.state_alternator = [[1, 11], [3, 4], [30, 60], [45, 90]]
        self.current_state_index = 0

    def timer_callback(self):
        # Log information about the current state index
        self.get_logger().info(f'Publishing data for state index: {self.current_state_index}')

        # Publish the current state to each topic
        self.publish_state('four_way_state', self.state_alternator[self.current_state_index][:2])
        self.publish_state('three_way_state', self.state_alternator[self.current_state_index][2:])
        self.publish_state('train_state', self.state_alternator[self.current_state_index])  # [100, 150]
        self.publish_state('aux_state', self.state_alternator[self.current_state_index + 1])  # [45, 90]

        # Update the current state index
        self.current_state_index = (self.current_state_index + 1) % len(self.state_alternator)

    def publish_state(self, topic, values):
        # Log information about the data being published
        self.get_logger().info(f'Publishing data to topic {topic}: {values}')

        # Create a message with the current state
        msg = Int32MultiArray()
        msg.data = values

        # Publish the message to the specified topic
        if topic == 'four_way_state':
            self.four_way_publisher.publish(msg)
        elif topic == 'three_way_state':
            self.three_way_publisher.publish(msg)
        elif topic == 'train_state':
            self.train_publisher.publish(msg)
        elif topic == 'aux_state':
            self.aux_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

