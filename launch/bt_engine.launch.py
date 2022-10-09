import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory 


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='behavior_tree_sample',
            executable='bt_engine',
            parameters=[
                {'xml_filename': 'hello_world.xml'},
                {'plugin_lib_names': ['hello_world_action_bt_node']}
            ],
            output='screen'
        ),
    ])
