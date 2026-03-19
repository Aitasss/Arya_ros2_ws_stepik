
# Что делает: Создаёт альтернативную ветку от tag_fixed через виртуальную камеру к расчётной базе.
# Зачем: Для экспериментов, сравнения, резервирования.
# Ключевой момент: Не влияет на основную логику, но даёт вторую точку зрения.

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations

class TagToCamera2Base2(Node):
    def __init__(self):
        super().__init__('tag_to_camera2_base2')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish)

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

    def publish(self):
        try:
            # Получаем camera → tag_dynamic (от детектора)
            cam_to_tag = self.tf_buffer.lookup_transform('camera_link', 'tag36h11:3', rclpy.time.Time())
            # Инвертируем в tag → camera
            mat = self.matrix_from_transform(cam_to_tag.transform)
            inv = tf_transformations.inverse_matrix(mat)
            tag_to_cam = self.transform_from_matrix(inv)

            # Получаем base_link → camera (URDF)
            base_to_cam = self.tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())
            # Инвертируем в camera → base
            mat = self.matrix_from_transform(base_to_cam.transform)
            inv = tf_transformations.inverse_matrix(mat)
            cam_to_base = self.transform_from_matrix(inv)

            # Получаем map → tag_fixed
            map_to_tag = self.tf_buffer.lookup_transform('map', 'tag_fixed', rclpy.time.Time())
            mat_map_tag = self.matrix_from_transform(map_to_tag.transform)

            # Вычисляем tag_fixed → camera2
            # camera2 — это виртуальная камера, привязанная к tag_fixed
            # Мы хотим, чтобы camera2 была в том же месте, где реальная камера
            # то есть tag_fixed → camera2 = tag_fixed → map * map → tag * tag → camera
            # Но проще: tag_fixed → camera2 = tag_fixed → map * map → camera
            # где map → camera = map → base_link * base_link → camera

            # Получаем map → base_link
            map_to_base = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            mat_map_base = self.matrix_from_transform(map_to_base.transform)

            # Получаем base_link → camera
            base_to_cam = self.tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())
            mat_base_cam = self.matrix_from_transform(base_to_cam.transform)

            # map → camera
            mat_map_cam = tf_transformations.concatenate_matrices(mat_map_base, mat_base_cam)

            # tag_fixed → map
            mat_tag_map = tf_transformations.inverse_matrix(mat_map_tag)

            # tag_fixed → camera2
            mat_tag_cam2 = tf_transformations.concatenate_matrices(mat_tag_map, mat_map_cam)

            # Публикуем tag_fixed → camera2
            t1 = TransformStamped()
            t1.header.stamp = self.get_clock().now().to_msg()
            t1.header.frame_id = 'tag_fixed'
            t1.child_frame_id = 'camera2'
            t1.transform = self.transform_from_matrix(mat_tag_cam2)
            self.tf_broadcaster.sendTransform(t1)

            # camera2 → base_link2 = (base_link → camera)⁻¹
            # то есть camera → base, но от camera2
            mat_cam_base = tf_transformations.inverse_matrix(mat_base_cam)
            t2 = TransformStamped()
            t2.header.stamp = self.get_clock().now().to_msg()
            t2.header.frame_id = 'camera2'
            t2.child_frame_id = 'base_link2'
            t2.transform = self.transform_from_matrix(mat_cam_base)
            self.tf_broadcaster.sendTransform(t2)

        except Exception as e:
            pass

def main():
    rclpy.init()
    node = TagToCamera2Base2()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()