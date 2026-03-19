import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('diplom_folder')
    
    # ===== 1. Чтение URDF файла =====
    # Описание робота: где находится камера относительно base_link
    urdf_path = os.path.join(pkg_dir, 'urdf', 'my_robot.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()
    
    return LaunchDescription([
        
        # ===== 2. Публикация URDF (статическая трансформация base_link → camera_link) =====
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # ===== 3. Запуск камеры =====
        # Публикует /logi_webcam/image_raw и /logi_webcam/camera_info
        ExecuteProcess(
            cmd=['ros2', 'launch', 'diplom_folder', 'camera.launch.py'],
            output='screen'
        ),
        
        # ===== 4. Запуск детектора AprilTag =====
        # Публикует детекции в /apriltag_detections
        # НЕ публикует трансформации (из-за бага)
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

        # ===== 5. Визуализация =====
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
        
        # ===== 6. Запись детекций в YAML =====
        # Подписывается на /apriltag_detections
        # Сохраняет координаты в tag_positions.yaml (история)
        Node(
            package='diplom_folder',
            executable='tag_to_yaml',
            name='tag_to_yaml',
            output='screen'
        ),
        
        # ===== 7. Публикация статической map → base_link =====
        # Вычисляет положение робота от AprilTag
        # Node(
        #    package='diplom_folder',
        #    executable='map_to_base_publisher',
        #    name='map_to_base_publisher',
        #    output='screen'
        # ),
        
        
        # ===== 8. Публикация статической map → tag_fixed =====
        # Читает tag_fixed.yaml
        # Публикует map → tag_fixed в /tf_static
        Node(
            package='diplom_folder',
            executable='map_to_tag_publisher',
            name='map_to_tag_publisher',
            output='screen'
        ),

        # ===== 9. Вычисление второй ветки: tag_fixed → camera2 → base_link2 =====
        # Использует:
        #   - map → tag_fixed
        #   - map → base_link
        #   - base_link → camera_link
        #   - camera_link → tag_dynamic
        # Публикует:
        #   - tag_fixed → camera2
        #   - camera2 → base_link2
        Node(
            package='diplom_folder',
            executable='tag_to_camera2_base2',
            name='tag_to_camera2_base2',
            output='screen'
        ),
    ])