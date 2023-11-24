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
    world_path = "package://world_gen/test.dae"

    if not os.path.exists(world_path):
        node.get_logger().error(f"Error: Mesh file not found at {world_path}")
        node.destroy_node()
        rclpy.shutdown()
        return


    marker.mesh_resource = world_path
    marker.pose = Pose()

 

    while rclpy.ok():
        node.get_logger().info('Publishing marker...')
        node.get_logger().info(marker.meshresr)
        marker_publisher.publish(marker)
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()