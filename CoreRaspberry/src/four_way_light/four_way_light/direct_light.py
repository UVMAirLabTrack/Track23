import rclpy
from std_msgs.msg import Int32

def map_state_to_integer(state):
    state_mapping = {
        "Red": 1,
        "Yellow": 2,
        "Green": 3,
        "Right": 4,
        "Left": 5,
        "All": 11
    }
    return state_mapping.get(state, 0)

def main():
    rclpy.init()

    node = rclpy.create_node('four_way_state_publisher')

    publisher = node.create_publisher(Int32, 'four_way_state', 10)

    while rclpy.ok():
        user_input = input("Enter state (Red, Yellow, Green, Right, Left, All) or an integer: ")

        try:
            # Try parsing input as an integer
            state_integer = int(user_input)
        except ValueError:
            # If it's not a valid integer, map the input to an integer state
            state_integer = map_state_to_integer(user_input)

        msg = Int32()
        msg.data = state_integer

        node.get_logger().info(f'Publishing: {state_integer}')
        publisher.publish(msg)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
