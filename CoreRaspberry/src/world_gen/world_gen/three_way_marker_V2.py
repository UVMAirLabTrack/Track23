import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Int32MultiArray
from custom_msgs.msg import WorldMarkers
import core_functions

import os
from ament_index_python.packages import get_package_share_directory

import multiprocessing



class ThreeWayVisualizer(Node):
    package_name = 'world_gen'
    color_mapping = {
        'red': [1.0, 0.0, 0.0, 1.0],
        'yellow': [1.0, 1.0, 0.0, 1.0],
        'green': [0.0, 1.0, 0.0, 1.0],
        'white': [1.0, 1.0, 1.0, 1.0],
        'blue': [0.0, 0.0, 1.0, 1.0],
        'off': [0.0, 0.0, 0.0, 0.0],
        'left_stop': [0.5, 0.0, 0.5, 1.0],
        'right_stop': [1.0, 0.0, 0.5, 1.0],
        'left_go': [0.0, 0.5, 0.5, 1.0],
        'right_go': [0.0, 1.0, 1.0, 1.0],
        'all': [0.1,0.1,0.1,0.1]

    }

    numeric_to_color = {
            0: 'off',
            1: 'red',
            2: 'yellow',
            3: 'green',
            4: 'white',
            5: 'blue',
            6: 'left_stop',
            7: 'right_stop',
            8: 'left_go',
            9: 'right_go',
            10: 'all',
        }
    def __init__(self, marker_name, pose_file):
        super().__init__('three_way_marker_' + marker_name)
        self.marker_name = marker_name
        self.light_colors = {
        'three_way_marker_' + marker_name: 'white',
        }
        
        self.subscription = self.create_subscription(Int32MultiArray, 'custom_poses', self.color_callback, 10)

        self.current_pose = Pose() 
        self.current_color = [0.0, 0.0, 0.0, 0.0]  # Default black color
        
        # Create publisher for the marker
        self.publisher = self.create_publisher(Marker, 'three_way_marker' + marker_name, 10)

        # Create subscription to the 4_way_state topic
        self.subscription = self.create_subscription(Int32MultiArray, 'three_way_state', self.color_callback, 10)

        # Set a timer to publish the marker periodically
        self.timer = self.create_timer(1.0, self.publish_marker)



    def color_callback(self, msg):
        # Use the numeric values directly
        numeric_values = msg.data

        # Map numeric values to color names
        colors = [self.numeric_to_color.get(value, 'off') for value in numeric_values]

        # Update colors for each light based on the received list
        for i, light in enumerate(['light_a', 'light_b', 'light_c', 'light_d']):
            light_name = f'three_way_marker_{light}'
            if light_name in self.light_colors and colors:
                self.light_colors[light_name] = colors[i]
            else:
                self.light_colors[light_name] = 'off'
                print("length failure")
    def pose_callback(self,msg):
        pass
        


    def publish_marker(self):
        marker_msg = Marker()
        marker_msg.header.frame_id = 'map'  # Set the frame ID as needed
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.id = 0
        marker_msg.type = Marker.MESH_RESOURCE
        marker_msg.action = Marker.ADD
        marker_msg.pose = self.current_pose
        marker_msg.scale.x = 1.0
        marker_msg.scale.y = 1.0
        marker_msg.scale.z = 1.0

        color_name = self.light_colors['three_way_marker_' + self.marker_name]

    # Use the color_mapping dictionary to get the RGBA values
        rgba_values = self.color_mapping.get(color_name, [1.0, 1.0, 1.0, 1.0])

    # Assign RGBA values to the marker message
        marker_msg.color.r, marker_msg.color.g, marker_msg.color.b, marker_msg.color.a = rgba_values
       # marker_msg.color.r, marker_msg.color.g, marker_msg.color.b, marker_msg.color.a = self.current_color
       # print(rgba_values)


        marker_msg.mesh_resource = 'package://world_gen/markers/light.stl'#os.path.join(get_package_share_directory(self.package_name),  'markers', 'light.dae')

        self.publisher.publish(marker_msg)


def run_marker(marker_name, pose_file):
    #rclpy.init()
    node = ThreeWayVisualizer(marker_name, pose_file)
    rclpy.spin(node)
    rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    # Read ROS parameters for the pose files and set default values
    pose_files = {
        'light_a': '4_way_poses_light_a.txt',
        'light_b': '4_way_poses_light_b.txt',
        'light_c': '4_way_poses_light_c.txt',
        'light_d': '4_way_poses_light_d.txt',
    }

    # Create processes for each marker
    process_a = multiprocessing.Process(target=run_marker, args=('light_a', pose_files['light_a']))
    process_b = multiprocessing.Process(target=run_marker, args=('light_b', pose_files['light_b']))
    process_c = multiprocessing.Process(target=run_marker, args=('light_c', pose_files['light_c']))
    process_d = multiprocessing.Process(target=run_marker, args=('light_d', pose_files['light_d']))

    # Start the processes
    process_a.start()
    process_b.start()
    process_c.start()
    process_d.start()

    
    process_a.join()
    process_b.join()
    process_c.join()
    process_d.join()

    rclpy.shutdown()

if __name__ == '__main__':
    main()