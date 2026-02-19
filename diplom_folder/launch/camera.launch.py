from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('diplom_folder')
    
    calibration_file = os.path.join(pkg_share, 'config', 'camera_calibration.yaml')
    
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            namespace='logi_webcam',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 1920,
                'image_height': 1080,
                'framerate': 30.0,
                'pixel_format': 'mjpeg2rgb',  
                'camera_frame_id': 'camera_link',
                'io_method': 'mmap',
                'camera_info_url': f'file://{calibration_file}',  
                'camera_name': 'default_cam'
            }]
        )
    ])

