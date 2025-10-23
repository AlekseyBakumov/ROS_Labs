from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('3turtles1carrot'), 'launch', 'base_demo.launch.py']),
            launch_arguments={'target_frame': 'carrot1'}.items(),
        ),
        Node(
            package='3turtles1carrot',
            executable='target_switcher',
            name='dynamic_broadcaster',
            parameters = [
              {'radius': 1.2},
              {'direction': -1},
              {'switch_threshold': 1.0f}
            ],
        ),
    ])
