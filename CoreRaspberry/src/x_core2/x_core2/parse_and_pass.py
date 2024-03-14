import rclpy
from rclpy.node import Node
from custom_msgs.msg import WorldMarkers,MarkerLoc
from std_msgs.msg import Header
import os
import numpy as np
from x_core2 import open_world_data

class PoseParserNode(Node):
    def __init__(self):
        super().__init__('pose_parser_node')
        self.publisher = self.create_publisher(WorldMarkers, 'custom_poses', 10)
        self.publisher2 = self.create_publisher(MarkerLoc, 'marker_loc', 10)
        self.timer = self.create_timer(1.0, self.publish_poses)
        self.pose_filename = "signpose.txt" #unused

        # Read poses from file
        #self.filepath = self.move_to_world_path(self.pose_filename)
        self.filepath = open_world_data.find_marker_loc_path()
        self.poses = self.read_poses_from_file(self.filepath)
        self.pose_index = 0

    def read_poses_from_file(self, filename):
        poses = []
        with open(filename, 'r') as file:
            for line in file:
                entries = line.strip().split()
                index = int(entries[0])
                title = entries[1]
                entry1 = entries[2]
                entry2 = entries[3]
                pose_values = list(map(float, entries[4:]))
                poses.append((index, title, entry1, entry2, *pose_values))
        return poses
    


    def publish_poses(self):
        pose_msg = WorldMarkers()
        pose_msg.index = []
        pose_msg.title = []
        pose_msg.entry1 = []
        pose_msg.entry2 = []
        pose_msg.x = []
        pose_msg.y = []
        pose_msg.z = []
        pose_msg.qx = []
        pose_msg.qy = []
        pose_msg.qz = []

        for pose_values in self.poses:
            pose_msg.index.append(pose_values[0])
            pose_msg.title.append(pose_values[1])
            pose_msg.entry1.append(pose_values[2])
            pose_msg.entry2.append(pose_values[3])
            pose_msg.x.append(pose_values[4])
            pose_msg.y.append(pose_values[5])
            pose_msg.z.append(pose_values[6])
            pose_msg.qx.append(pose_values[7])
            pose_msg.qy.append(pose_values[8])
            pose_msg.qz.append(pose_values[9])

        self.get_logger().info(f"Publishing {len(self.poses)} Poses")
        self.publisher.publish(pose_msg)
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
