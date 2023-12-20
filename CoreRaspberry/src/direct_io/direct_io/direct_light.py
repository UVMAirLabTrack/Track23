import rclpy
from std_msgs.msg import Int32MultiArray

def map_state_to_integer(state):
    state_mapping = {
        "off": 0,
        "red": 1,
        "yellow": 2,
        "green": 3,
        "right": 4,
        "left": 5,
        "go-R": 6,
        "go-l": 7,
        "stop-r": 8,
        "stop-l": 9,
        "all": 10,
    }
    return state_mapping.get(state, 0)

def main():
    rclpy.init()

    node = rclpy.create_node('four_way_state_publisher')

    publisher = node.create_publisher(Int32MultiArray, 'four_way_state', 10)

    while rclpy.ok():
        user_input = input("Enter state 1 (Red, Yellow, Green, Right, Left, All) or an integer: ")
        user_input2 = input("Enter state 2 (Red, Yellow, Green, Right, Left, All) or an integer: ")

        try:
            # Try parsing input as an integer
            state_integer = int(user_input)
            state_integer2 = int(user_input2)
        except ValueError:
            # If it's not a valid integer, map the input to an integer state
            state_integer = map_state_to_integer(user_input)
            state_integer2 = map_state_to_integer(user_input2)

        # Create an array message with two integers
        msg = Int32MultiArray()
        msg.data = [state_integer, state_integer2]  # For simplicity, setting the second element to 0

        node.get_logger().info(f'Publishing: {msg.data}')
        publisher.publish(msg)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
