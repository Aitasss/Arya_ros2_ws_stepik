#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import tf_transformations
import math

class MapToBasePublisher(Node):
    def __init__(self):
        super().__init__('map_to_base_publisher')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        pkg_dir = get_package_share_directory('diplom_folder')
        self.yaml_file = os.path.join(pkg_dir, 'config', 'tag_positions.yaml')
        
        self.timer = self.create_timer(0.1, self.publish_map_to_base)
        self.get_logger().info('MapToBasePublisher started')

    def matrix_from_transform(self, t):
        trans = [t.translation.x, t.translation.y, t.translation.z]
        rot = [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
        mat = tf_transformations.quaternion_matrix(rot)
        mat[0][3] = trans[0]
        mat[1][3] = trans[1]
        mat[2][3] = trans[2]
        return mat

    def transform_from_matrix(self, mat):
        t = TransformStamped().transform
        t.translation.x = mat[0][3]
        t.translation.y = mat[1][3]
        t.translation.z = mat[2][3]
        quat = tf_transformations.quaternion_from_matrix(mat)
        t.rotation.x = quat[0]
        t.rotation.y = quat[1]
        t.rotation.z = quat[2]
        t.rotation.w = quat[3]
        return t

    def publish_map_to_base(self):
        try:
            # 1. Получаем camera_link → tag (из детектора)
            camera_to_tag = self.tf_buffer.lookup_transform(
                'camera_link',
                'tag36h11:3',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            self.get_logger().debug('Got camera → tag')
            
            # 2. Получаем base_link → camera_link (из URDF)
            base_to_camera = self.tf_buffer.lookup_transform(
                'base_link',
                'camera_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            self.get_logger().debug('Got base → camera')
            
            # 3. Получаем tag → map (из static_transform)
            tag_to_map = self.tf_buffer.lookup_transform(
                'tag36h11:3',
                'map',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            self.get_logger().debug('Got tag → map')
            
            # Компонуем: map → base = (tag → map)^-1 * (camera → tag) * (base → camera)^-1
            # Но проще: инвертируем всё и перемножаем
            map_to_tag = tag_to_map
            map_to_tag_mat = self.matrix_from_transform(map_to_tag.transform)
            
            camera_to_tag_mat = self.matrix_from_transform(camera_to_tag.transform)
            base_to_camera_mat = self.matrix_from_transform(base_to_camera.transform)
            
            # Вычисляем map → base
            map_to_base_mat = tf_transformations.concatenate_matrices(
                map_to_tag_mat,
                camera_to_tag_mat,
                tf_transformations.inverse_matrix(base_to_camera_mat)
            )
            
            map_to_base_transform = self.transform_from_matrix(map_to_base_mat)
            
            # Публикуем
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'base_link'
            t.transform = map_to_base_transform
            
            self.tf_broadcaster.sendTransform(t)
            self.get_logger().info(f'Published map → base_link')
            
        except Exception as e:
            self.get_logger().debug(f'Waiting for transforms: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MapToBasePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()