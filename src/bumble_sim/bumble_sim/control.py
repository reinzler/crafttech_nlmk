#!/usr/bin/env python3
import sys
import rclpy
import geometry_msgs.msg
import std_msgs.msg
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import termios
import tty

msg = """
Control Your Rover!
---------------------------
Moving around:
   W
A  S  D
W/S : move forward/backward
A/D : turn left/right
Spacebar: stop the rover
CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0),   # Move forward
    's': (-1, 0),  # Move backward
    'a': (0.1, 1),   # Turn left
    'd': (0.1, -1),  # Turn right
}


def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    settings = saveTerminalSettings()
    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')

    # Configure QoS profile for publishing and subscribing
    qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1
    )

    # Create publishers
    pub = node.create_publisher(geometry_msgs.msg.Twist, '/cmd_vel', qos_profile)
    teleop_speed = node.create_publisher(std_msgs.msg.Float32MultiArray, '/rover_velocity', qos_profile)
    speed = 0.5  # Linear speed
    turn = 1.0   # Angular speed

    try:
        print(msg)
        while True:
            key = getKey(settings)
            if key in moveBindings:
                x = moveBindings[key][0]
                th = moveBindings[key][1]
            elif key == ' ':
                x = 0
                th = 0

            else:
                x = 0
                th = 0
                if key == '\x03':  # CTRL-C
                    break

            twist = geometry_msgs.msg.Twist()
            twist.linear.x = x * speed
            twist.angular.z = th * turn
            pub.publish(twist)

            velocity = std_msgs.msg.Float32MultiArray()
            velocity.data = [x * speed, 0.0, 0.0, th * turn]
            teleop_speed.publish(velocity)

    except Exception as e:
        node.get_logger().error(f'Error occurred: {e}')

    finally:
        # Send a stop command before exiting
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        velocity = std_msgs.msg.Float32MultiArray()
        velocity.data = [x * speed, 0.0, 0.0, th * turn]
        teleop_speed.publish(velocity)

        restoreTerminalSettings(settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

### Variant 2
###########################################
#!/usr/bin/env python3
# import sys
# import rclpy
# import geometry_msgs.msg
# import std_msgs.msg
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# import termios
# import tty
# import time
# import math
#
# msg = """
# Control Your Rover!
# ---------------------------
# Moving around:
#    W
# A  S  D
# W/S : move forward/backward
# A/D : turn left/right
# R : rotate by a specific angle
# Spacebar: stop the rover
# CTRL-C to quit
# """
#
# moveBindings = {
#     'w': (1, 0),  # Move forward
#     's': (-1, 0),  # Move backward (will be modified to rotate 180 degrees)
#     'a': (0, 1),  # Turn left
#     'd': (0, -1),  # Turn right
# }
#
#
# def getKey(settings):
#     tty.setraw(sys.stdin.fileno())
#     key = sys.stdin.read(1)
#     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
#     return key
#
#
# def saveTerminalSettings():
#     return termios.tcgetattr(sys.stdin)
#
#
# def restoreTerminalSettings(old_settings):
#     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
#
#
# def rotate(pub, angle_degrees, angular_speed):
#     """ Функция для поворота ровера на заданный угол """
#     twist = geometry_msgs.msg.Twist()
#
#     # Устанавливаем угловую скорость
#     twist.angular.z = math.copysign(angular_speed, angle_degrees)
#     pub.publish(twist)
#
#     # Подсчитываем время, необходимое для поворота на заданный угол
#     time_required = abs(angle_degrees) / angular_speed
#
#     # Ожидание завершения поворота
#     time.sleep(time_required)
#
#     # Остановка после поворота
#     twist.angular.z = 0.0
#     pub.publish(twist)
#
#
# def stop(pub):
#     """ Функция для полной остановки ровера """
#     twist = geometry_msgs.msg.Twist()
#     twist.linear.x = 0.0
#     twist.angular.z = 0.0
#     pub.publish(twist)
#
#
# def main():
#     settings = saveTerminalSettings()
#     rclpy.init()
#
#     node = rclpy.create_node('teleop_twist_keyboard')
#
#     # Configure QoS profile for publishing and subscribing
#     qos_profile = QoSProfile(
#         reliability=ReliabilityPolicy.BEST_EFFORT,
#         durability=DurabilityPolicy.TRANSIENT_LOCAL,
#         history=HistoryPolicy.KEEP_LAST,
#         depth=1
#     )
#
#     # Create publishers
#     pub = node.create_publisher(geometry_msgs.msg.Twist, '/cmd_vel', qos_profile)
#     teleop_speed = node.create_publisher(std_msgs.msg.Float32MultiArray, '/rover_velocity', qos_profile)
#     speed = 0.5  # Linear speed
#     angular_speed = 1.0  # Angular speed (radians per second)
#
#     try:
#         print(msg)
#         while True:
#             key = getKey(settings)
#             if key in moveBindings:
#                 x = moveBindings[key][0]
#                 th = moveBindings[key][1]
#
#                 if key == 's':
#                     # Разворот на 180 градусов перед движением назад
#                     rotate(pub, 180, angular_speed)
#                     stop(pub)
#                     x = 1  # Теперь двигаться вперед, так как ровер развернут
#                     th = 0
#
#             elif key == 'r':
#                 # Запрашиваем угол поворота
#                 restoreTerminalSettings(settings)
#                 angle_degrees = float(input("Enter the angle to rotate (in degrees): "))
#                 rotate(pub, angle_degrees, angular_speed)
#                 stop(pub)
#                 settings = saveTerminalSettings()
#                 continue  # Переход к следующему циклу без движения
#
#             elif key == ' ':
#                 x = 0
#                 th = 0
#             else:
#                 x = 0
#                 th = 0
#                 if key == '\x03':  # CTRL-C
#                     break
#
#             twist = geometry_msgs.msg.Twist()
#             twist.linear.x = x * speed
#             twist.angular.z = th * angular_speed
#             pub.publish(twist)
#
#             velocity = std_msgs.msg.Float32MultiArray()
#             velocity.data = [x * speed, 0.0, 0.0, th * angular_speed]
#             teleop_speed.publish(velocity)
#
#     except Exception as e:
#         node.get_logger().error(f'Error occurred: {e}')
#
#     finally:
#         # Send a stop command before exiting
#         stop(pub)
#         restoreTerminalSettings(settings)
#         node.destroy_node()
#         rclpy.shutdown()
#
#
# if __name__ == '__main__':
#     main()

# #!/usr/bin/env python3
# import sys
# import rclpy
# from geometry_msgs.msg import Twist
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# import termios
# import tty
#
# msg = """
# Control Your Rover!
# ---------------------------
# Moving around:
#    W
# A  S  D
# W/S : move forward/backward
# A/D : turn left/right
# Spacebar: stop the rover
# CTRL-C to quit
# """
#
# moveBindings = {
#     'w': (1, 0),   # Move forward
#     's': (-1, 0),  # Move backward
#     'a': (0, 1),   # Turn left
#     'd': (0, -1),  # Turn right
# }
#
# def getKey(settings):
#     tty.setraw(sys.stdin.fileno())
#     key = sys.stdin.read(1)
#     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
#     return key
#
# def saveTerminalSettings():
#     return termios.tcgetattr(sys.stdin)
#
# def restoreTerminalSettings(old_settings):
#     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
#
# def main():
#     settings = saveTerminalSettings()
#     rclpy.init()
#
#     node = rclpy.create_node('teleop_twist_keyboard')
#
#     # Configure QoS profile for publishing and subscribing
#     qos_profile = QoSProfile(
#         reliability=ReliabilityPolicy.BEST_EFFORT,
#         durability=DurabilityPolicy.TRANSIENT_LOCAL,
#         history=HistoryPolicy.KEEP_LAST,
#         depth=1
#     )
#
#     # Create publisher
#     pub = node.create_publisher(Twist, '/cmd_vel', qos_profile)
#     speed = 0.5  # Linear speed
#     turn = 1.0   # Angular speed
#
#     try:
#         print(msg)
#         while True:
#             key = getKey(settings)
#             if key in moveBindings:
#                 x = moveBindings[key][0]
#                 th = moveBindings[key][1]
#             elif key == ' ':
#                 x = 0
#                 th = 0
#             else:
#                 x = 0
#                 th = 0
#                 if key == '\x03':  # CTRL-C
#                     break
#
#             twist = Twist()
#             twist.linear.x = x * speed
#             twist.angular.z = th * turn
#             pub.publish(twist)
#
#     except Exception as e:
#         node.get_logger().error(f'Error occurred: {e}')
#
#     finally:
#         # Send a stop command before exiting
#         twist = Twist()
#         twist.linear.x = 0.0
#         twist.angular.z = 0.0
#         pub.publish(twist)
#
#         restoreTerminalSettings(settings)
#         node.destroy_node()
#         rclpy.shutdown()
#
# if __name__ == '__main__':
#     main()




