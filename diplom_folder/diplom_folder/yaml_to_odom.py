#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import yaml
import os
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler

class YamlToOdom(Node):
    def __init__(self):
        super().__init__('yaml_to_odom')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.yaml_file = '/home/igsp-01/ros2_ws_arya/src/Arya_ros2_ws_stepik/diplom_folder/config/tag_positions.yaml'
        
        # Подписываемся на одометрию (чтобы знать положение робота)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        self.current_odom = None
        self.tag_position = None
        
        # Таймер для публикации трансформации
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
        if not tag or not self.current_odom:
            return
            
        # Вычисляем map → odom на основе положения тега и одометрии
        # Это сложная часть - нужно учитывать, где тег в мире
        # Пока публикуем фиксированную трансформацию для теста

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        
        # Берём позицию из YAML
        pos = tag['position']
        t.transform.translation.x = pos['x']
        t.transform.translation.y = pos['y']
        t.transform.translation.z = pos['z']
        
        # Углы Эйлера (roll, pitch, yaw) в радианах
        roll = 0.0    # крен
        pitch = 0.0   # тангаж
        yaw = 0.0     # рысканье

        # Берём углы Эйлера из YAML (в градусах)
        orient = tag['orientation']
        roll_deg = orient['roll_deg']
        pitch_deg = orient['pitch_deg']
        yaw_deg = orient['yaw_deg']

        # Переводим в радианы
        roll = roll_deg * 3.14159 / 180.0
        pitch = pitch_deg * 3.14159 / 180.0
        yaw = yaw_deg * 3.14159 / 180.0

        # Преобразуем в кватернион
        q = quaternion_from_euler(roll, pitch, yaw)
        
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().debug('Published map → odom transform')

def main(args=None):
    rclpy.init(args=args)
    node = YamlToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()