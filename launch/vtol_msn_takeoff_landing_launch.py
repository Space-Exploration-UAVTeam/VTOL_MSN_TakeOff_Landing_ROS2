
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
   config = os.path.join(get_package_share_directory('vtol_msn_takeoff_landing'), 'config', 'params.yaml')

   return LaunchDescription([
      Node(
         package='vtol_msn_takeoff_landing',
         executable='vtol_msn_takeoff_landing',
         name='vtol_msn_takeoff_landing',
         parameters=[config]
      )
   ])