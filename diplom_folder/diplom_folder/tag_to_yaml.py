#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
import yaml
import os
from datetime import datetime

class TagToYaml(Node):
    def __init__(self):
        super().__init__('tag_to_yaml')
        
        # Подписываемся на детекции
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag_detections',
            self.detection_callback,
            10)
        
        # Файл для записи
        self.yaml_file = '/home/igsp-01/ros2_ws_arya/src/Arya_ros2_ws_stepik/diplom_folder/config/tag_positions.yaml'
        
        self.get_logger().info('Tag to YAML node started')

    def detection_callback(self, msg):
        if not msg.detections:
            return
            
        # Берём первое обнаружение (или можно обрабатывать все)
        detection = msg.detections[0]
        
        # Формируем данные для YAML
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
                    'x': detection.pose.pose.pose.orientation.x,
                    'y': detection.pose.pose.pose.orientation.y,
                    'z': detection.pose.pose.pose.orientation.z,
                    'w': detection.pose.pose.pose.orientation.w
                },
                'timestamp': datetime.now().isoformat()
            }
        }
        
        # Записываем в YAML
        with open(self.yaml_file, 'w') as f:
            yaml.dump(tag_data, f, default_flow_style=False)
        
        self.get_logger().info(f'Tag {detection.id} saved to YAML')

def main(args=None):
    rclpy.init(args=args)
    node = TagToYaml()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()