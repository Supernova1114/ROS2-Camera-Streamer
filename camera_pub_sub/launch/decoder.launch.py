from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    camera_decoder = Node(
        package='camera_pub_sub',
        executable='camera_decoder',
        name='camera1_dec',
        output='screen',
        parameters=[
            {"camera_name": "camera1"},
            {"image_transport": "compressed"},
        ],
    )

    return LaunchDescription([
        camera_decoder,
    ])

