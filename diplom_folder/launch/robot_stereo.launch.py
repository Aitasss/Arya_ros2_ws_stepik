import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('diplom_folder')
    
    # URDF файл
    urdf_path = os.path.join(pkg_dir, 'urdf', 'my_robot.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()
    
    return LaunchDescription([
        # 1. Публикация трансформаций робота (URDF → TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # 2. Запуск камеры
        ExecuteProcess(
            cmd=['ros2', 'launch', 'diplom_folder', 'camera.launch.py'],
            output='screen'
        ),
        
        # 3. Запуск детектора AprilTag
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'apriltag_ros', 'tag_detector',
                '--ros-args',
                '-r', 'image:=/logi_webcam/image_raw',
                '-r', 'camera_info:=/logi_webcam/camera_info',
                '-p', 'image_transport:=raw',
                '--params-file', os.path.join(pkg_dir, 'config', 'apriltag_config.yaml')
            ],
            output='screen'
        ),
        
        # Публикация статической трансформации: map → tag
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '--x', '2.0',        # тег в 2 метрах по X от угла комнаты
                '--y', '1.5',        # тег в 1.5 метрах по Y
                '--z', '1.0',        # тег на высоте 1 метр
                '--yaw', '0',
                '--pitch', '0',
                '--roll', '0',
                '--frame-id', 'map',
                '--child-frame-id', 'tag36h11:3'   # ID твоего тега
            ],
            output='screen'
        )

        # 4. Rviz2 для визуализации
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
