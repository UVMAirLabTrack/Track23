from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
      


    Node(
    package='four_way_light',
    
    executable='pub',
    name='four_way_control'),

      Node(
    package='three_way_light',
    
    executable='pub',
    name='three_way_control'),

   Node(
    package='direct_io',
    
    executable='esp_serial',
    name='serial_converter')


   ])