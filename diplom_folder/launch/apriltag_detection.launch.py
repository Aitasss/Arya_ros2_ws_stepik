
# Что делает: Запускает детектор AprilTag.
# Зачем: Чтобы находить тег на изображении.
# Ключевой момент: Детектор публикует camera_link → tag36h11:3 в /tf (динамика).

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
                'size': 0.2,
                'max_hamming': 0,
                'publish_tf': True,
                'image_transport': 'raw'
            }]
        )
    ])