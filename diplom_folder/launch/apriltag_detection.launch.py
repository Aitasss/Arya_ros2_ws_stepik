from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='tag_detector',
            name='apriltag_detector',
            remappings=[
                ('image', '/logi_webcam/image_raw'),
                ('camera_info', '/logi_webcam/camera_info')
            ],
            parameters=[{
                'family': '36h11',
                'size': 0.15,
                'max_hamming': 0,
                'publish_tf': True,
                'image_transport': 'raw'
            }]
        )
    ])