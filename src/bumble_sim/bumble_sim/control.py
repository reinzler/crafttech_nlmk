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
    'a': (0, 1),   # Turn left
    'd': (0, -1),  # Turn right
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

    # Define QoS profiles
    qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10
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
            velocity.data = [speed, 0.0]
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
        velocity.data = [0.0, 0.0]
        teleop_speed.publish(velocity)

        restoreTerminalSettings(settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
