from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_share_directory = get_package_share_directory('world_gen')
    rviz_directory = os.path.join(package_share_directory, 'rviz')
    file = 'config_lights.rviz'
    path = os.path.join(rviz_directory,file)
    print(path)

    # Declare a launch argument for the RViz configuration file parameter


    # Get the path to the RViz configuration file from the launch argument

    # Launch RViz with the specified configuration file
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d',path],
    )

    return LaunchDescription([

            Node(
    package='map_transforms',
    
    executable='odom_to_map',
    name='map_transform'
),
    ])