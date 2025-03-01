from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='group16_final',  # Replace with your package name
            executable='listen',  # Replace with your executable name
            name='listen',
            parameters=[{'/home/hv/final_ws/src/group16_final/config/waypoint_params.yaml'}]  # Replace with the path to your param.yaml
        )
    ])
