#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# Импортируем сгенерированный модуль из пакета интерфейсов
from my_service_interfaces.srv import SetVelocity
from geometry_msgs.msg import Twist


class VelocityServiceServer(Node):

    def __init__(self):
        super().__init__('velocity_service_server')
        # Создаём сервис, который будет «слушать» на /set_velocity
        self.srv = self.create_service(
            SetVelocity, 
            'set_velocity',
            self.handle_set_velocity
        )
        self.get_logger().info('Service /set_velocity is up and running!')
        #ADD publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def handle_set_velocity(self, request, response):
        """
        Этот метод вызывается при получении запроса.
        :param request: SetVelocity.Request
        :param response: SetVelocity.Response
        """
        linear = request.linear
        angular = request.angular

        self.get_logger().info(f'Received velocity request: linear={linear}, angular={angular}')

        # Формируем Twist
        twist_msg = Twist()
        twist_msg.linear.x = float(linear)
        twist_msg.angular.z = float(angular)

        # Публикуем в топик /turtle1/cmd_vel
        self.cmd_vel_publisher.publish(twist_msg)

        # Считаем, что сервер «задаёт» эту скорость (пока здесь просто логика-заглушка)
        success = True
        message = f'Velocity is set to linear={linear:.2f}, angular={angular:.2f}'

        # Заполняем поля ответа
        response.success = success
        response.message = message
        return response


def main(args=None):
    rclpy.init(args=args)
    node = VelocityServiceServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
