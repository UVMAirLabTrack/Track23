import rclpy
from rclpy.node import Node
from custom_msgs.msg import WorldMarkers
from std_msgs.msg import Header
import os

class PoseParserNode(Node):
    def __init__(self):
        super().__init__('pose_parser_node')
        self.publisher = self.create_publisher(WorldMarkers, 'custom_poses', 10)
        self.timer = self.create_timer(1.0, self.publish_poses)
        self.pose_filename = "signpose.txt"

        # Read poses from file
        self.filepath = self.move_to_world_path(self.pose_filename)
        self.poses = self.read_poses_from_file(self.filepath)
        self.pose_index = 0

    def read_poses_from_file(self, filename):
        poses = []
        with open(filename, 'r') as file:
            for line in file:
                entries = line.strip().split()
                index = int(entries[0])
                entry1 = entries[1]
                entry2 = entries[2]
                pose_values = list(map(float, entries[3:]))
                poses.append((index, entry1, entry2, *pose_values))
        return poses
    
    def move_to_world_path(self, pose_filename):
            script_path = os.path.dirname(os.path.abspath(__file__))
            parent_folder = os.path.abspath(os.path.join(script_path, os.pardir, os.pardir, os.pardir))
            #world_ctrl = os.path.join(parent_folder, 'worlds', 'world_select.txt')
            world_file = pose_filename
            path = os.path.join(parent_folder, 'worlds', f'{world_file}')
            #world_pose_path = os.path.join(parent_folder, 'worlds', f'{world_pose}')
            return(path)

    def publish_poses(self):
        if self.pose_index < len(self.poses):
            pose_values = self.poses[self.pose_index]
            pose_msg = WorldMarkers()
            pose_msg.header = Header()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.index = [pose_values[0]]
            pose_msg.entry1 = [pose_values[1]]
            pose_msg.entry2 = [pose_values[2]]
            pose_msg.x = [pose_values[3]]
            pose_msg.y = [pose_values[4]]
            pose_msg.z = [pose_values[5]]
            pose_msg.qx = [pose_values[6]]
            pose_msg.qy = [pose_values[7]]
            pose_msg.qz = [pose_values[8]]
            #pose_msg.qw = [pose_values[9]]

            self.get_logger().info(f"Publishing Pose {self.pose_index + 1}: {pose_values}")
            self.publisher.publish(pose_msg)
            self.pose_index += 1
        else:
            self.get_logger().info('Finished publishing poses')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    pose_parser_node = PoseParserNode()
    rclpy.spin(pose_parser_node)
    pose_parser_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
