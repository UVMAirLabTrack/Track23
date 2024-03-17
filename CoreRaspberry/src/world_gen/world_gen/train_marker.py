import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Quaternion
from custom_msgs.msg import MarkerLoc, WorldMarkers
from x_core2 import pose_strip, open_world_data,formulas
from math import sin, cos, radians
import os
from ament_index_python.packages import get_package_share_directory
import multiprocessing

class TrainVisualizer(Node):
    package_name = 'world_gen'
    node_title = "train_"

    def __init__(self, marker_name):
        super().__init__(self.node_title + marker_name)
        self.marker_name = marker_name
        self.marker_title = 'train_barrier'
        self.marker_path = f'package://world_gen/markers/{self.marker_title}.dae'
        self.marker_data = pose_strip.read_marker_param(self.marker_title)
        self.marker_pose = pose_strip.strip_marker_pose(self.marker_data)
        self.current_angle = 0.0  # Initialize current angle

        # Create publisher for the marker
        self.publisher = self.create_publisher(Marker, self.node_title + marker_name, 10)

        # Create subscription to the train_state topic
        self.subscription = self.create_subscription(Int32MultiArray, 'train_crossing_state', self.angle_callback, 10)

        # Set a timer to publish the marker periodically
        self.timer = self.create_timer(1.0, self.publish_marker)

        # Initialize marker pose
        self.pose = Pose()
        self.pose.orientation = Quaternion()  # Identity quaternion
        self.eulers = [0,0,0]

        # Subscribe to marker_loc topic for marker location information
        self.subscription2 = self.create_subscription(MarkerLoc, 'marker_loc', self.loc_call, 10)
        # Subscribe to custom_poses topic for custom marker poses
        self.subscription3 = self.create_subscription(WorldMarkers, 'custom_poses', self.pose_call, 10)

        self.marker = self.node_title+marker_name #set for testing, use later in other classes.



    def loc_call(self, msg):
        self.zone, self.loc = pose_strip.strip_marker_loc(msg, self.marker)

    def pose_call(self, msg):
        temp_pose = pose_strip.strip_pose(msg, self.zone, self.loc)
        self.pose = pose_strip.pose_xyz_shift(temp_pose, self.marker_pose)
        self.eulers = formulas.quat_to_euler(pose_strip.strip_pose_quat(msg, self.zone, self.loc))

    def angle_callback(self, msg):
        # Assuming the received message contains angles for all markers
        if len(msg.data) >= int(self.marker_name):
            self.current_angle = msg.data[int(self.marker_name) - 1]  # Index starts from 0
        else:
            self.get_logger().warn("Insufficient data in angle message.")

    def publish_marker(self):
        # Update orientation based on the current angle
        new_y = self.eulers[1]+self.current_angle
        q = formulas.euler_to_quat(self.eulers[0],new_y,self.eulers[2])
        self.pose.orientation = q
        

        # Publish marker
        marker_msg = Marker()
        marker_msg.header.frame_id = 'map'  # Set the frame ID as needed
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.id = 0
        marker_msg.type = Marker.MESH_RESOURCE
        marker_msg.action = Marker.ADD
        marker_msg.pose = self.pose
        marker_msg.mesh_resource = self.marker_path

        marker_msg.scale.x = self.marker_data.get('Scale_x', 1.0)
        marker_msg.scale.y = self.marker_data.get('Scale_y', 1.0)
        marker_msg.scale.z = self.marker_data.get('Scale_z', 1.0)
        change_color = self.marker_data.get('Change_Color', False)
        if change_color:
            marker_msg.color.r = self.marker_data.get('Color_r', 1.0)
            marker_msg.color.g = self.marker_data.get('Color_g', 1.0)
            marker_msg.color.b = self.marker_data.get('Color_b', 1.0)
            marker_msg.color.a = self.marker_data.get('Color_a', 1.0)
            
        marker_msg.lifetime.sec = int(self.marker_data.get('Lifetime_sec', 1))
        marker_msg.frame_locked = bool(self.marker_data.get('Frame_locked', False))
        marker_msg.mesh_use_embedded_materials = bool(self.marker_data.get('Mesh_use_embedded_materials', False))
        marker_msg.header.frame_id = self.marker_data.get('Frame_id', "map")


    # Assign RGBA values to the marker message


        self.publisher.publish(marker_msg)


def run_marker(marker_name):
    #rclpy.init()
    node = TrainVisualizer(marker_name)
    rclpy.spin(node)
    rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    # Create processes for each marker
    process_a = multiprocessing.Process(target=run_marker, args=('1',))
    process_b = multiprocessing.Process(target=run_marker, args=('2',))
    process_c = multiprocessing.Process(target=run_marker, args=('3',))
    process_d = multiprocessing.Process(target=run_marker, args=('4',))

    # Start the processes
    process_a.start()
    process_b.start()
    process_c.start()
    process_d.start()

    # Join the processes
    process_a.join()
    process_b.join()
    process_c.join()
    process_d.join()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
