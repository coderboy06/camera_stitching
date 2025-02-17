from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cam_side_view',
            executable='cam_node_left',
            name='cam_node_left',
            output='screen'
        ),
        Node(
            package='cam_side_view',
            executable='cam_node_right',
            name='cam_node_right',
            output='screen'
        ),
        # Node(
        #     package='cam_side_view',
        #     executable='stitch_node',
        #     name='stitch_node',
        #     output='screen'
        # ),
        Node(
            package='cam_side_view',
            executable='cam_node_back',
            name='cam_node_back',
            output='screen'
        ),
        Node(
            package='cam_side_view',
            executable='cam_node_front',
            name='cam_node_front',
            output='screen'
        ),
    ])
