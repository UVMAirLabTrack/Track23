import rclpy
from rclpy.node import Node
from custom_msgs.msg import WorldMarkers, MarkerLoc
from std_msgs.msg import Header
import os
import numpy as np
from x_core2 import open_world_data


class PoseParserNode(Node):
    def __init__(self):
        super().__init__('pose_parser_node')
        self.publisher = self.create_publisher(WorldMarkers, 'custom_poses', 10)
        self.publisher2 = self.create_publisher(MarkerLoc, 'marker_loc', 10)
        self.timer2 = self.create_timer(5.0, self.publish_indexes)
        self.timer = self.create_timer(5.0, self.publish_poses)
        

        self.filepath = open_world_data.find_marker_loc_path()
        self.markerpath = open_world_data.find_marker_path()
        self.poses_tmp = self.read_poses_from_file(self.filepath)
        self.markers = self.read_indexes_from_file(self.markerpath)
        self.poses = self.find_zero_loc(self.poses_tmp)
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
    
    def find_zero_loc(self, poses):
    # Find the index of the ZPM pose
        zpm_index = None
        for i, pose in enumerate(poses):
            if pose[1] == "ZPM":  # Check the title
                zpm_index = i
                break

        if zpm_index is None:
            print("ZPM pose not found.")
            zpm_x,zpm_y,zpm_z=[0,0,0]
            return None

        # Get the x, y, z values of the ZPM pose
        else:
            zpm_x, zpm_y, zpm_z, *_ = poses[zpm_index][4:]  # Assuming x, y, z values start from index 4

        # Adjust other poses based on the difference between their x, y, and z values and those of the ZPM pose
        adjusted_poses = []
        for pose in poses:
            index, title, entry1, entry2, x, y, z, qx, qy, qz, qw = pose
            if title == "ZPM":
                # Skip the ZPM pose
                continue
            # Calculate the differences
            dx = x - zpm_x
            dy = y - zpm_y
            dz = z - zpm_z
            # Adjust the pose values
            adjusted_pose = (index, title, entry1, entry2, dx, dy, dz, qx, qy, qz, qw)
            adjusted_poses.append(adjusted_pose)


        return adjusted_poses
        

    def read_indexes_from_file(self,filename):
        indexes = []
        with open(filename, 'r') as file:
            for line in file:
                entries = line.strip().split()
  
                title = entries[0]
                entry1 = entries[1]
                entry2 = entries[2]
                indexes.append((title,entry1,entry2))
        return indexes
    


    def publish_indexes(self):
        marker_msg = MarkerLoc()
        
        marker_msg.title = []
        marker_msg.entry1 = []
        marker_msg.entry2 = []


        for marker_values in self.markers:
            
            marker_msg.title.append(marker_values[0])
            marker_msg.entry1.append(marker_values[1])
            marker_msg.entry2.append(marker_values[2])


        self.get_logger().info(f"Publishing {len(self.markers)} Markers")
        self.publisher2.publish(marker_msg)
        #self.get_logger().info('Finished publishing Markers')



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
        pose_msg.qw = []

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
            pose_msg.qw.append(pose_values[10])

        self.get_logger().info(f"Publishing {len(self.poses)} Poses")
        self.publisher.publish(pose_msg)
        #self.get_logger().info('Finished publishing poses')



def main(args=None):
    rclpy.init(args=args)
    pose_parser_node = PoseParserNode()
    rclpy.spin(pose_parser_node)
    pose_parser_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
