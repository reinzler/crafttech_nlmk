import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String


class ModeOrchestrator(Node):
    def __init__(self):
        super().__init__('mode_orchestrator')
        self.current_mode = "manual_keyboard"  # Начальный режим
        self.last_source = None  # Последний источник команды

        # Приоритеты
        self.priorities = {
            "manual_rc": 1,  # Самый высокий приоритет
            "manual_keyboard": 2,
            "auto": 3  # Самый низкий приоритет
        }

        # Подписки на различные источники команд
        self.keyboard_sub = self.create_subscription(Twist, 'keyboard_cmd_vel', self.cmd_callback, 10)
        self.rc_sub = self.create_subscription(Twist, 'rc_cmd_vel', self.cmd_callback, 10)
        self.auto_sub = self.create_subscription(Twist, 'auto_cmd_vel', self.cmd_callback, 10)

        # Подписка на топик смены режимов
        self.mode_sub = self.create_subscription(String, 'mode_change', self.mode_callback, 10)

        # Публикация в общий топик управления роботом
        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        # Топик для публикации предупреждений
        self.warning_pub = self.create_publisher(String, 'warning', 10)

    def mode_callback(self, msg):
        self.get_logger().info(f"Switching mode to: {msg.data}")
        self.current_mode = msg.data
        self.last_source = None  # Сброс последнего источника

        # Обнуляем команду при смене режима, чтобы исключить выполнение старых команд
        self.stop_robot()

    def cmd_callback(self, msg):
        # Определяем, какой топик прислал команду
        topic = self.get_current_subscription().topic_name

        # Определяем источник команды на основе топика
        if topic == 'rc_cmd_vel':
            source = "manual_rc"
        elif topic == 'keyboard_cmd_vel':
            source = "manual_keyboard"
        elif topic == 'auto_cmd_vel':
            source = "auto"
        else:
            return

        # Проверяем приоритет текущей команды и сравниваем с последним источником
        if self.last_source is None or self.priorities[source] < self.priorities[self.last_source]:
            self.last_source = source
            command_message = TwistStamped()
            command_message.twist.linear.x = msg.linear.x
            command_message.twist.linear.y = msg.linear.y
            command_message.twist.linear.z = msg.linear.z
            command_message.twist.angular.z = msg.angular.z
            self.cmd_pub.publish(command_message)

        elif self.priorities[source] == self.priorities[self.last_source]:
            # Это условие не нужно, так как приоритеты уникальны
            return
        else:
            return

    def stop_robot(self):
        stop_message = Twist()
        stop_message.twist.linear.x = 0.0
        stop_message.twist.linear.y = 0.0
        stop_message.twist.linear.z = 0.0
        stop_message.twist.angular.z = 0.0
        self.cmd_pub.publish(stop_message)


def main(args=None):
    rclpy.init(args=args)
    node = ModeOrchestrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
