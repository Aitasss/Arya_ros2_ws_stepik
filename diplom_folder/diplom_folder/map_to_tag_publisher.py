#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import yaml
from tf_transformations import quaternion_from_euler
import os
from ament_index_python.packages import get_package_share_directory
import math

class MapToTagPublisher(Node):
    def __init__(self):
        super().__init__('map_to_tag_publisher')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        pkg_dir = get_package_share_directory('diplom_folder')
        self.yaml_file = os.path.join(pkg_dir, 'config', 'tag_positions.yaml')
        self.get_logger().info(f'MapToTagPublisher started, reading from {self.yaml_file}')
        
        self.timer = self.create_timer(0.1, self.publish_transform)

    def read_tag_yaml(self):
        try:
            with open(self.yaml_file, 'r') as f:
                data = yaml.safe_load(f)
                if data and 'tag' in data:
                    return data['tag']
        except Exception as e:
            self.get_logger().error(f'Error reading YAML: {e}')
            return None
        return None

    def publish_transform(self):
        tag_data = self.read_tag_yaml()
        if not tag_data:
            return
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'tag36h11:3'
        
        pos = tag_data['position']
        t.transform.translation.x = float(pos['x'])
        t.transform.translation.y = float(pos['y'])
        t.transform.translation.z = float(pos['z'])
        
        orient = tag_data['orientation']
        roll = float(orient['roll_deg']) * math.pi / 180.0
        pitch = float(orient['pitch_deg']) * math.pi / 180.0
        yaw = float(orient['yaw_deg']) * math.pi / 180.0
        
        q = quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f'Published map → tag at ({pos["x"]}, {pos["y"]}, {pos["z"]})')

def main(args=None):
    rclpy.init(args=args)
    node = MapToTagPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()