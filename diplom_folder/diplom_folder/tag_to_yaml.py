#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from tf_transformations import euler_from_quaternion
import math

class TagToYaml(Node):
    def __init__(self):
        super().__init__('tag_to_yaml')
        
        pkg_dir = get_package_share_directory('diplom_folder')
        self.yaml_file = os.path.join(pkg_dir, 'config', 'tag_positions.yaml')
        self.get_logger().info(f'YAML file path: {self.yaml_file}')
        
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag_detections',
            self.detection_callback,
            10)
        
        self.get_logger().info('Tag to YAML node started')

    def quaternion_to_euler(self, x, y, z, w):
        (roll, pitch, yaw) = euler_from_quaternion([x, y, z, w])
        roll_deg = roll * 180.0 / math.pi
        pitch_deg = pitch * 180.0 / math.pi
        yaw_deg = yaw * 180.0 / math.pi
        return roll_deg, pitch_deg, yaw_deg

    def detection_callback(self, msg):
        if not msg.detections:
            return
            
        detection = msg.detections[0]
        
        qx = detection.pose.pose.pose.orientation.x
        qy = detection.pose.pose.pose.orientation.y
        qz = detection.pose.pose.pose.orientation.z
        qw = detection.pose.pose.pose.orientation.w
        
        roll_deg, pitch_deg, yaw_deg = self.quaternion_to_euler(qx, qy, qz, qw)
        
        tag_data = {
            'tag': {
                'id': detection.id,
                'family': detection.family,
                'position': {
                    'x': detection.pose.pose.pose.position.x,
                    'y': detection.pose.pose.pose.position.y,
                    'z': detection.pose.pose.pose.position.z
                },
                'orientation': {
                    'roll_deg': roll_deg,
                    'pitch_deg': pitch_deg,
                    'yaw_deg': yaw_deg
                }
            }
        }
        
        try:
            with open(self.yaml_file, 'w') as f:
                yaml.dump(tag_data, f, default_flow_style=False)
            self.get_logger().info(f'Tag {detection.id} saved to YAML')
        except Exception as e:
            self.get_logger().error(f'Failed to write YAML: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TagToYaml()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()