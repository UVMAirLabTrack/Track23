import rclpy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose

def main():
    rclpy.init()
    node = rclpy.create_node('world_publisher')

    marker_publisher = node.create_publisher(Marker, '/visualization_marker', 10)

    marker = Marker()
    marker.type = Marker.MESH_RESOURCE
    marker.mesh_resource = "package://world_gen/worlds/test.dae"
    marker.pose = Pose()

    while rclpy.ok():
        node.get_logger().info('Publishing marker...')
        marker_publisher.publish(marker)
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()