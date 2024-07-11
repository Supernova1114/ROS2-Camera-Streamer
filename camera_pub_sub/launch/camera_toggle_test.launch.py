from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    camera_toggle_test = Node(
        package='camera_pub_sub',
        executable='camera_toggle_test',
        name='camera_toggle_test',
        output='screen',
        parameters=[
            {"camera_name": "camera1"},
            {"toggle_camera": True},
        ],
    )

    return LaunchDescription([
        camera_toggle_test,
    ])

