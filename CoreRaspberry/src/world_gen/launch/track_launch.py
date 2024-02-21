from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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
    package='four_way_light',
    
    executable='pub',
    name='four_way_control'
),

    Node(
    package='map_transforms',
    
    executable='odom_to_map',
    name='odom_transform'
),

   ])