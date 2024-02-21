from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
      


    Node(
    package='four_way_light',
    
    executable='pub',
    name='four_way_control')


   ])