import rclpy
from std_msgs.msg import Int32MultiArray

def main():
    rclpy.init()

    node = rclpy.create_node('integer_to_topic_publisher')

    topics_mapping = {
        1: "four_way_state",
        2: "three_way_state",
        3: "train_crossing_state",
        4: "aux_state"
    }

    print("Select a topic:")
    for key, value in topics_mapping.items():
        print(f"{key}: {value}")

    try:
        topic_choice = int(input("Enter the number corresponding to the topic: ([Four way, 1],[Three way,2],[Train,3],[Aux,4]) "))
        selected_topic = topics_mapping.get(topic_choice)
    except ValueError:
        print("Invalid input. Please enter a valid integer.")
        return

    if selected_topic is None:
        print("Invalid topic choice. Exiting.")
        return

    publisher = node.create_publisher(Int32MultiArray, selected_topic, 10)

    while rclpy.ok():
        try:
            user_input1 = int(input("Enter the first integer: "))
            user_input2 = int(input("Enter the second integer: "))
            user_input3 = int(input("Enter the third integer: "))
            user_input4 = int(input("Enter the fourth integer: "))
        except ValueError:
            print("Invalid input. Please enter valid integers.")
            continue

        # Create an array message with two integers
        msg = Int32MultiArray()
        msg.data = [user_input1, user_input2,user_input3,user_input4]

        node.get_logger().info(f'Publishing: {msg.data} on topic {selected_topic}')
        publisher.publish(msg)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()