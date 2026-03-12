import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('diplom_folder')
    
    urdf_path = os.path.join(pkg_dir, 'urdf', 'my_robot.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        ExecuteProcess(
            cmd=['ros2', 'launch', 'diplom_folder', 'camera.launch.py'],
            output='screen'
        ),
        
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
        
        # ЕДИНСТВЕННАЯ статика: tag → map
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '--x', '0',
                '--y', '0',
                '--z', '-1.0',
                '--yaw', '0',
                '--pitch', '0',
                '--roll', '0',
                '--frame-id', 'tag36h11:3',
                '--child-frame-id', 'map'
            ],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
        
        Node(
            package='diplom_folder',
            executable='tag_to_yaml',
            name='tag_to_yaml',
            output='screen'
        ),
        
        # Узел для вычисления map → base_link (с инверсией)
        #Node(
        #    package='diplom_folder',
        #    executable='map_to_base_publisher',
        #    name='map_to_base_publisher',
        #    output='screen'
        #),
        Node(
            package='diplom_folder',
            executable='map_to_tag_publisher',
            name='map_to_tag_publisher',
            output='screen'
        ),
    ])