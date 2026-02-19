#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from my_service_interfaces.srv import SetVelocity

class JoyToVelocity(Node):
    def __init__(self):
        super().__init__('joy_to_velocity')
        
        # Подписка на джойстик
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        self.get_logger().info('Subscribed to /joy topic')
        
        # Клиент для твоего сервиса
        self.client = self.create_client(SetVelocity, 'set_velocity')
        
        # Ждём сервер
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for velocity_service_server...')
        self.get_logger().info('Connected to velocity_service_server')
        
        # Настройки управления
        self.linear_scale = 2.0    # макс линейная скорость
        self.angular_scale = 3.0   # макс угловая скорость
        self.deadzone = 0.1        # мёртвая зона
        
        self.get_logger().info('Joy to Velocity node started!')
        self.get_logger().info('Use left stick: up/down = linear, left/right = angular')
    
    def joy_callback(self, msg):
        """Вызывается при каждом новом сообщении с джойстика"""
        
        if len(msg.axes) < 2:
            return
        
        # Получаем значения с левого стика
        RT = msg.axes[5]   # вертикаль вперед
        LT = msg.axes[2]   # вертикаль назад

        raw_angular = msg.axes[0]    # горизонталь
        
        linear = (RT - LT) * self.linear_scale

        angular = raw_angular * self.angular_scale
        
        # Мёртвая зона
        if abs(linear) < self.deadzone:
            linear = 0.0
        if abs(angular) < self.deadzone:
            angular = 0.0
        
        # Масштабируем
        
        # Логируем если есть движение
        if abs(linear) > 0.01 or abs(angular) > 0.01:
            self.get_logger().info(f'Joy -> RT: {RT:.2f}, LT: {LT:.2f}, linear: {linear:.2f} angular: {angular:.2f}')
        
        # Вызываем сервис
        self.call_velocity_service(linear, angular)
    
    def call_velocity_service(self, linear, angular):
        """Вызов сервиса /set_velocity"""
        
        request = SetVelocity.Request()
        request.linear = float(linear)
        request.angular = float(angular)
        
        future = self.client.call_async(request)
        future.add_done_callback(self.service_callback)
    
    def service_callback(self, future):
        """Ответ от сервера"""
        try:
            response = future.result()
            if not response.success:
                self.get_logger().warn(f'Server rejected: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = JoyToVelocity()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down joy_to_velocity')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
