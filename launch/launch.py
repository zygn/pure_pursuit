import os
import sys 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():

    ld = LaunchDescription()
    
    ## this code will open {YOUR_WS}/pure_pursuit/share/pure_pursuit/config/params.yaml
    # config = os.path.join(get_package_share_directory('pure_pursuit'),'config','params.yaml')
   
    ## or you can open source folder directly 
    config = os.path.abspath("src/pure_pursuit/config/params.yaml")

    ld.add_action(
        Node(
            package='pure_pursuit',
            executable='pursuit',
            name='pure_pursuit',
            output='screen',
            parameters=[config]
        )
    )

    return ld