from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    camera_viewer = Node(
        package='camera_pub_sub',
        executable='camera_viewer',
        name='camera1_view',
        output='screen',
        parameters=[
            {'camera_name': 'camera1'},
            {'window_width': 960},
            {'window_height': 540}
        ],
    )

    return LaunchDescription([
        camera_viewer
    ])

