import rclpy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Quaternion
import os
from .core_functions import open_world_data


def read_world_pose(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    pose_data = {}
    for line in lines:
        key, value = line.strip().split()
        try:
            pose_data[key] = float(value)
        except ValueError:
            # Handle non-numeric values, for example, by keeping them as strings
            pose_data[key] = value

    return pose_data



def main():
    rclpy.init()
    node = rclpy.create_node('world_publisher')

    marker_publisher = node.create_publisher(Marker, '/world_map', 10)

    marker = Marker()
    marker.type = Marker.MESH_RESOURCE

    world_pose_path = open_world_data.find_pose_path()
    world_path = open_world_data.find_world_path()
    pose_data = read_world_pose(world_pose_path)


    # Set marker properties using the extracted pose data
    marker.pose.position.x = pose_data.get('X_position', 0.0)
    marker.pose.position.y = pose_data.get('Y_position', 1.0)
    marker.pose.position.z = pose_data.get('Z_position', 1.0)
    marker.scale.x = pose_data.get('Scale_x', 1.0)
    marker.scale.y = pose_data.get('Scale_y', 1.0)
    marker.scale.z = pose_data.get('Scale_z', 1.0)
    change_color = pose_data.get('Change_Color', False)
    if change_color:
        marker.color.r = pose_data.get('Color_r', 1.0)
        marker.color.g = pose_data.get('Color_g', 1.0)
        marker.color.b = pose_data.get('Color_b', 1.0)
        marker.color.a = pose_data.get('Color_a', 1.0)
    quaternion = Quaternion()
    quaternion.x = pose_data.get('Quaternion_x', 0.0)
    quaternion.y = pose_data.get('Quaternion_y', 0.0)
    quaternion.z = pose_data.get('Quaternion_z', 0.0)
    quaternion.w = pose_data.get('Quaternion_w', 1.0)
    marker.pose.orientation = quaternion
    marker.lifetime.sec = int(pose_data.get('Lifetime_sec', 1))
    marker.frame_locked = bool(pose_data.get('Frame_locked', False))
    marker.mesh_use_embedded_materials = bool(pose_data.get('Mesh_use_embedded_materials', False))
    marker.header.frame_id = pose_data.get('Frame_id', "map")


    if not os.path.exists(world_path):
        node.get_logger().error(f"Error: Mesh file not found at {world_path}")
        node.destroy_node()
        rclpy.shutdown()
        return


    marker.mesh_resource = f"file://{world_path}"


 

    def publish_marker_callback():
        nonlocal marker
        node.get_logger().info(f'Publishing world from path: {world_path}')
        node.get_logger().info("Marker properties set:")
        node.get_logger().info(f"Pose: {marker.pose}")
        node.get_logger().info(f"Scale: {marker.scale}")
        node.get_logger().info(f"Change Color?: {change_color}")
        node.get_logger().info(f"Color: {marker.color}")
        node.get_logger().info(f"Lifetime_sec: {marker.lifetime.sec}")
        node.get_logger().info(f"Frame_locked: {marker.frame_locked}")
        node.get_logger().info(f"Mesh_use_embedded_materials: {marker.mesh_use_embedded_materials}")
        node.get_logger().info(f"Frame_id: {marker.header.frame_id}")
        marker_publisher.publish(marker)

    timer_period = 10.0  # seconds
    timer = node.create_timer(timer_period, publish_marker_callback)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()