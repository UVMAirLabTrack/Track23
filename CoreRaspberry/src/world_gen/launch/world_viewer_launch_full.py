from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
   package_share_directory = get_package_share_directory('world_gen')
   return LaunchDescription([
      
    Node(
    package='world_gen',
    
    executable='gen_v2',
    name='world_map'
),


    Node(
    package='x_core2',
    
    executable='pub_all_pose',
    name='marker_parser'
),

    Node(
    package='world_gen',
    
    executable='four_way_marker',
    name='four_markers'
),
    Node(
    package='world_gen',
    
    executable='three_way_marker',
    name='three_markers'
),
    Node(
    package='map_transforms',
    
    executable='odom_to_map',
    name='map_transform'
),
    Node(
   
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'4marker_rviz': package_share_directory + 'world_gen/rviz/4marker_rviz.rviz'}],
        ),

   ])