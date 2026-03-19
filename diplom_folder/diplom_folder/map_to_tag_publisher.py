
#Статический тег
# Что делает: Читает tag_fixed.yaml и публикует статическую трансформацию map → tag_fixed.
# Зачем: Чтобы тег всегда был известен в координатах карты.
# Ключевой момент: Публикует в /tf_static, живёт вечно.

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
from tf2_ros import StaticTransformBroadcaster

class MapToTagPublisher(Node):
    def __init__(self):
        super().__init__('map_to_tag_publisher')
        
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Создаём фрейм map (публикуем identity-трансформацию)
        # t_map = TransformStamped()
        # t_map.header.stamp = self.get_clock().now().to_msg()
        # t_map.header.frame_id = 'map'
        # t_map.child_frame_id = 'map'
        # t_map.transform.translation.x = 0.0
        # t_map.transform.translation.y = 0.0
        # t_map.transform.translation.z = 0.0
        # t_map.transform.rotation.w = 1.0
        # self.static_broadcaster.sendTransform(t_map)
        # self.get_logger().info('Created map frame')

        # ищет папку пакета diplom_folder.
        pkg_dir = get_package_share_directory('diplom_folder')
        # Путь к YAML
        self.yaml_file = os.path.join(pkg_dir, 'config', 'tag_fixed.yaml')
        self.get_logger().info(f'MapToTagPublisher started, reading from {self.yaml_file}')
    
        self.publish_transform()   

    # Чтение данных 
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

    # Преобразование координат
    def publish_transform(self):
        tag_data = self.read_tag_yaml()
        if not tag_data:
            return
            
        # Создаём трансформацию map → tag
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'tag_fixed'
        
        # Берём позицию из YAML
        pos = tag_data['position']
        t.transform.translation.x = float(pos['x'])
        t.transform.translation.y = float(pos['y'])
        t.transform.translation.z = float(pos['z'])
        
        # Берём углы из YAML и преобразуем в кватернион
        orient = tag_data['orientation']
        roll = float(orient['roll_deg']) * math.pi / 180.0
        pitch = float(orient['pitch_deg']) * math.pi / 180.0
        yaw = float(orient['yaw_deg']) * math.pi / 180.0
        
        q = quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        #Публикация
        self.static_broadcaster.sendTransform(t)
        self.get_logger().info(f'Published map → tag at ({pos["x"]}, {pos["y"]}, {pos["z"]})')

def main(args=None):
    rclpy.init(args=args)
    node = MapToTagPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()