import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String


class ModeOrchestrator(Node):
    def __init__(self):
        super().__init__('mode_orchestrator')
        self.current_mode = "manual_keyboard"  # Начальный режим
        self.last_source = None  # Последний источник команды

        # Приоритеты
        self.priorities = {
            "manual_rc": 1,
            "manual_keyboard": 2,
            "auto": 3
        }

        # Подписки на различные источники команд
        self.keyboard_sub = self.create_subscription(TwistStamped, 'keyboard_cmd_vel', self.cmd_callback, 10)
        self.rc_sub = self.create_subscription(TwistStamped, 'rc_cmd_vel', self.cmd_callback, 10)
        self.auto_sub = self.create_subscription(TwistStamped, 'auto_cmd_vel', self.cmd_callback, 10)

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
        if self.last_source and self.last_source != source:
            if self.priorities[source] < self.priorities[self.last_source]:
                self.last_source = source
            elif self.priorities[source] == self.priorities[self.last_source]:
                # Если команды от разных источников с одинаковым приоритетом, останавливаем робота и предупреждаем
                self.stop_robot()
                warning_msg = String()
                warning_msg.data = "Conflicting commands from multiple sources. Robot stopped."
                self.warning_pub.publish(warning_msg)
                return
            else:
                return

        self.last_source = source
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        stop_message = TwistStamped()
        stop_message.twist.linear.x = 0.0
        stop_message.twist.linear.y = 0.0
        stop_message.twist.linear.z = 0.0
        stop_message.twist.angular.x = 0.0
        stop_message.twist.angular.y = 0.0
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
