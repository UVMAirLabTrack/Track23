from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
      
    Node(
    package='testing_pubs',
    
    executable='circle_odometry',
    name='drive_in_circles'
),


   ])