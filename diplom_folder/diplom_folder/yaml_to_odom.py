#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import yaml
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
import os
from ament_index_python.packages import get_package_share_directory

class YamlToOdom(Node):
    def __init__(self):
        super().__init__('yaml_to_odom')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Путь к YAML
        pkg_dir = get_package_share_directory('diplom_folder')
        self.yaml_file = os.path.join(pkg_dir, 'config', 'tag_positions.yaml')
        
        # Подписка на одометрию
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        self.current_odom = None
        
        # Таймер для публикации трансформации (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_transform)
        
        self.get_logger().info('YAML to Odom node started')

    def odom_callback(self, msg):
        self.current_odom = msg

    def read_tag_yaml(self):
        try:
            with open(self.yaml_file, 'r') as f:
                data = yaml.safe_load(f)
                if data and 'tag' in data:
                    return data['tag']
        except:
            return None
        return None

    def publish_transform(self):
        # Читаем положение тега из YAML
        tag = self.read_tag_yaml()
        if not tag:
            return
            
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        
        # Позиция
        pos = tag['position']
        t.transform.translation.x = float(pos['x'])
        t.transform.translation.y = float(pos['y'])
        t.transform.translation.z = float(pos['z'])
        
        # Ориентация (углы Эйлера в градусах → радианы → кватернион)
        orient = tag['orientation']
        roll = float(orient['roll_deg']) * 3.14159 / 180.0
        pitch = float(orient['pitch_deg']) * 3.14159 / 180.0
        yaw = float(orient['yaw_deg']) * 3.14159 / 180.0
        
        q = quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = YamlToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()