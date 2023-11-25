import rclpy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
import os

def world_select(file_path):
    selected_world = None
    world_pose = None

    with open(file_path, 'r') as file:
        lines = file.readlines()
        for line in lines:
            parts = line.strip().split()

            if len(parts) > 1 and 'y' in parts[1]:
                selected_world = f"{parts[0]}.dae"
                world_pose = f"{parts[0]}.txt"
                break  # Exit the loop after the first match

    return selected_world, world_pose

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

    marker_publisher = node.create_publisher(Marker, '/visualization_marker', 10)

    marker = Marker()
    marker.type = Marker.MESH_RESOURCE
    

    script_path = os.path.dirname(os.path.abspath(__file__))
    parent_folder = os.path.abspath(os.path.join(script_path, os.pardir, os.pardir, os.pardir))
    world_ctrl = os.path.join(parent_folder, 'worlds', 'world_select.txt')
    world_file, world_pose = world_select(world_ctrl)
    world_path = os.path.join(parent_folder, 'worlds', f'{world_file}')
    world_pose_path = os.path.join(parent_folder, 'worlds', f'{world_pose}')


    pose_data = read_world_pose(world_pose_path)


    # Set marker properties using the extracted pose data
    marker.pose.position.x = pose_data.get('X_position', 0.0)
    marker.pose.position.y = pose_data.get('Y_position', 1.0)
    marker.pose.position.z = 5 #pose_data.get('Z_position', 1.0)
    marker.scale.x = pose_data.get('Scale_x', 1.0)
    marker.scale.y = pose_data.get('Scale_y', 1.0)
    marker.scale.z = pose_data.get('Scale_z', 1.0)
    marker.color.r = pose_data.get('Color_r', 1.0)
    marker.color.g = pose_data.get('Color_g', 1.0)
    marker.color.b = pose_data.get('Color_b', 1.0)
    marker.color.a = pose_data.get('Color_a', 1.0)
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
    marker.pose = Pose()

 

    while rclpy.ok():
        node.get_logger().info(f'Publishing world from path: {world_path}')
        node.get_logger().info("Marker properties set:")
        node.get_logger().info(f"Pose: {marker.pose}")
        node.get_logger().info(f"Pose y: {marker.pose.position.y}")
        node.get_logger().info(f"Scale: {marker.scale}")
        node.get_logger().info(f"Color: {marker.color}")
        node.get_logger().info(f"Lifetime_sec: {marker.lifetime.sec}")
        node.get_logger().info(f"Frame_locked: {marker.frame_locked}")
        node.get_logger().info(f"Mesh_use_embedded_materials: {marker.mesh_use_embedded_materials}")
        node.get_logger().info(f"Frame_id: {marker.header.frame_id}")
        marker_publisher.publish(marker)
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()