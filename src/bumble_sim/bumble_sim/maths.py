import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from scipy.integrate import cumulative_trapezoid
import numpy as np


class MathNode(Node):
    def __init__(self):
        super().__init__('math_node')
        self.start_time, self.current_time, self.time_list  = 0.0, None, []
        self.velocity, self.velocity_left, self.velocity_right  = 0.0, 0.0, 0.0
        self.angular_velocity, self.angular_velocity_list = 0.0, []
        self.theta, self.theta_0 = 0.0, 0.0
        self.d = 1.1  # Расстояние между гусеницами/колесами
        self.x_0, self.y_0, self.x, self.y = 0.0, 0.0, 0.0, 0.0

        # QoS для подписки
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.cmd_sub = self.create_subscription(TwistStamped, '/cmd_vel', self.cmd_callback, qos_profile)

    def cmd_callback(self, msg):
        # Получаем линейную и угловую скорости из сообщения
        self.velocity = msg.twist.linear.x
        self.angular_velocity = msg.twist.angular.z

        # Получаем текущее время из сообщения
        self.current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.start_time is None:
            # Первое сообщение, задаём стартовое время
            self.start_time = self.current_time
            relative_time = 0.0  # Время отсчитывается от старта
        else:
            # Рассчитываем относительное время
            relative_time = self.current_time - self.start_time

        # Добавляем угловую скорость и время
        self.time_list.append(relative_time)
        self.angular_velocity_list.append(self.angular_velocity)

        # Проверка на синхронизацию списков (опционально)
        if len(self.angular_velocity_list) != len(self.time_list):
            self.get_logger().warning(f"Mismatch in list lengths: time_list len: {len(self.time_list)}, angular_velocity_list len: {len(self.angular_velocity_list)}")

        # Обновляем скорости и азимут
        self.calculate_velocities()
        self.calculate_azimuth()

    def calculate_velocities(self):
        # Вычисляем скорости для левого и правого колёс
        self.velocity_left = self.velocity - (self.angular_velocity * self.d) / 2
        self.velocity_right = self.velocity + (self.angular_velocity * self.d) / 2

    def calculate_azimuth(self):
        # Интегрируем угловую скорость для получения азимута
        if len(self.angular_velocity_list) > 1 and len(self.time_list) > 1:
            try:
                # Интеграция угловой скорости методом трапеций
                self.theta = cumulative_trapezoid(self.angular_velocity_list, self.time_list, initial=self.theta_0)
                self.x = self.x_0 + self.velocity * self.time_list[-1] * np.cos(self.theta)
                self.y = self.y_0 + self.velocity * self.time_list[-1] * np.sin(self.theta)
                self.get_logger().info(f"Theta (азимут): {self.theta[-1]}")

            except ValueError:
                # Ловим ошибки интегрирования, если они возникли
                self.get_logger().warning("Ошибка при интегрировании")
                self.get_logger().info(f"W list len: {len(self.angular_velocity_list)}")
                self.get_logger().info(f"Time list len: {len(self.time_list)}")
        else:
            self.get_logger().warning("Недостаточно данных для интегрирования")


def main(args=None):
    rclpy.init(args=args)
    node = MathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
