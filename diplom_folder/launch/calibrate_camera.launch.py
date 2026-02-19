from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_calibration',
            executable='cameracalibrator',
            name='cameracalibrator',
            arguments=['--size', '8x6', '--square', '0.024'],
            remappings=[
                ('image', '/image_raw'),
                ('/camera/set_camera_info', '/v4l2_camera/set_camera_info')
            ]
        )
    ])