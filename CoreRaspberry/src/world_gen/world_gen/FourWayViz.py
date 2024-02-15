import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
import tf2_ros
import tf2_geometry_msgs
import tf2_py
import os
from ament_index_python.packages import get_package_share_directory

class FourWayVisualizer(Node):
    package_name = 'world_gen'
    def __init__(self, marker_name, pose_files):
        super().__init__('four_way_visualizer_' + marker_name)
        self.marker_name = marker_name
        self.possible_poses = self.read_poses_from_files(pose_files)
        self.current_pose = self.possible_poses[0]
        self.current_color = [0.0, 0.0, 0.0, 0.0]  # Default black color
        

        # Create publisher for the marker
        self.publisher = self.create_publisher(Marker, 'marker_topic_' + marker_name, 10)

        # Create subscription to the 4_way_state topic
        self.subscription = self.create_subscription(String, 'four_way_state', self.color_callback, 10)

        # Set a timer to publish the marker periodically
        self.timer = self.create_timer(0.1, self.publish_marker)

    def read_poses_from_files(self, pose_files):
        poses = {}
        for light, pose_file in pose_files.items():
            share_path = get_package_share_directory(self.package_name)
            worlds_folder_path = os.path.join(share_path, '..', '..', 'CoreRaspberry', 'worlds')
            pose_file = os.path.join(worlds_folder_path, pose_file)

            poses[light] = self.read_pose_from_file(pose_file)

        return poses

    def read_pose_from_file(self, pose_file):
        poses = []
        try:
            with open(pose_file, 'r') as file:
                for line in file:
                    x, y, z, roll, pitch, yaw = map(float, line.split())
                    pose = Pose()
                    pose.position.x, pose.position.y, pose.position.z = x, y, z
                    quaternion = tf2_py.Quaternion()
                    quaternion.setRPY(roll, pitch, yaw)
                    pose.orientation = tf2_geometry_msgs.msg.Quaternion()
                    pose.orientation.x = quaternion[0]
                    pose.orientation.y = quaternion[1]
                    pose.orientation.z = quaternion[2]
                    pose.orientation.w = quaternion[3]
                    poses.append(pose)
        except Exception as e:
            self.get_logger().error(f"Failed to read pose from file: {e}")
        return poses

    def color_callback(self, msg):
        # Set the marker color based on the received string
        color_mapping = {
            'red': [1.0, 0.0, 0.0, 1.0],
            'yellow': [1.0, 1.0, 0.0, 1.0],
            'green': [0.0, 1.0, 0.0, 1.0],
            'white': [1.0, 1.0, 1.0, 1.0],
            'blue': [0.0, 0.0, 1.0, 1.0],
        }
        self.current_color = color_mapping.get(msg.data, [1.0, 1.0, 1.0, 1.0])

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
        marker_msg.color.r, marker_msg.color.g, marker_msg.color.b, marker_msg.color.a = self.current_color

        share_path = get_package_share_directory(self.package_name)
        worlds_folder_path = os.path.join(share_path, '..', '..', 'CoreRaspberry', 'worlds')
        marker_msg.mesh_resource = os.path.join(worlds_folder_path,'worlds' 'markers', 'light.stl')  # Update with your mesh file

        self.publisher.publish(marker_msg)

def main(args=None):
    rclpy.init(args=args)

    # Read ROS parameters for the pose files and set default values
    pose_files = {
        'light_a': '4_way_poses_light_a.txt',
        'light_b': '4_way_poses_light_b.txt',
        'light_c': '4_way_poses_light_c.txt',
        'light_d': '4_way_poses_light_d.txt',
    }

    # Create instances of FourWayVisualizer for each light
    marker_a = FourWayVisualizer('light_a', pose_files)
    marker_b = FourWayVisualizer('light_b', pose_files)
    marker_c = FourWayVisualizer('light_c', pose_files)
    marker_d = FourWayVisualizer('light_d', pose_files)

    # Spin the nodes
    rclpy.spin(marker_a)
    rclpy.spin(marker_b)
    rclpy.spin(marker_c)
    rclpy.spin(marker_d)

    # Shutdown
    marker_a.destroy_node()
    marker_b.destroy_node()
    marker_c.destroy_node()
    marker_d.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()