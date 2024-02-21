from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
   package_share_directory = get_package_share_directory('your_package')
   return LaunchDescription([
      
    Node(
    package='world_gen',
    
    executable='gen',
    name='world_map'
),

    Node(
    package='world_gen',
    
    executable='four_way_marker',
    name='Four_markers'
),


    Node(
    package='map_transforms',
    
    executable='odom_to_map',
    name='odom_transform'
),
    Node(
   
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'rviz_config_file': package_share_directory + '/path/to/your/rviz/config/file.rviz'}],
        ),

   ])