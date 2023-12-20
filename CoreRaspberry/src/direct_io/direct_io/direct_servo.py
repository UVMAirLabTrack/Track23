import rclpy
from std_msgs.msg import Int32MultiArray

def map_state_to_integer(state):
    state_mapping = {
        "stop": 0,
        "half": 45,
        "go": 100,

    }
    return state_mapping.get(state, 0)

def main():
    rclpy.init()

    node = rclpy.create_node('train_state_publisher')

    publisher = node.create_publisher(Int32MultiArray, 'train_state', 10)

    while rclpy.ok():
        user_input = input("Enter state 1 (stop, half, go) or an integer: ")
        user_input2 = input("Enter state 2 (stop, half, go) or an integer: ")

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
