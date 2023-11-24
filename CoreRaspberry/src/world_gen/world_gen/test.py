import rclpy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
import os

def main():
    rclpy.init()
    node = rclpy.create_node('world_publisher')

    marker_publisher = node.create_publisher(Marker, '/visualization_marker', 10)

    marker = Marker()
    marker.type = Marker.MESH_RESOURCE
    #world_path = "/home/dev/RaspberryCore/CoreRaspberry/worlds/test.dae"

    script_path = os.path.dirname(os.path.abspath(__file__))
    parent_folder = os.path.abspath(os.path.join(script_path, os.pardir, os.pardir, os.pardir))
    world_path = os.path.join(parent_folder, 'worlds', 'test.dae')

    marker.pose.position.x = 1.0
    marker.pose.position.y = 2.0
    marker.pose.position.z = 0.0
    

    
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    marker.lifetime.sec = 0  # Set a non-zero duration
    marker.frame_locked = False
    marker.mesh_use_embedded_materials = True
    marker.header.frame_id = "map"

    if not os.path.exists(world_path):
        node.get_logger().error(f"Error: Mesh file not found at {world_path}")
        node.destroy_node()
        rclpy.shutdown()
        return


    marker.mesh_resource = f"file://{world_path}"
    marker.pose = Pose()

 

    while rclpy.ok():
        node.get_logger().info('Publishing marker...')
        
        marker_publisher.publish(marker)
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()