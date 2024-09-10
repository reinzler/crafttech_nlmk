#!/usr/bin/env python
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, TwistStamped
import sys, select, termios, tty
import time

settings = termios.tcgetattr(sys.stdin)

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   q    w    e
   a    s    d

---------------------------

anything else : stop

Up/Down : increase/decrease max speeds by 10%
Right/Left : increase/decrease only linear speed by 10%
z/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0, 0, 0),
    's': (-1, 0, 0, 0),
    'a': (0, 0, 0, -1),
    'd': (0, 0, 0, 1),
    'e': (1, 0, 0, -1),
    'q': (1, 0, 0, 1),
    ' ': (0, 0, 0, 0),
}

speedBindings = {
    '\x1b[A': (1.1, 1.0),  # Вверх
    '\x1b[B': (0.9, 1.0),  # Вниз
    '\x1b[D': (1.0, 0.9),  # Влево
    '\x1b[C': (1.0, 1.1),  # Вправо
    'z': (1.0, 0.9),
    'c': (1.0, 1.1),
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':  # Обработка начала последовательности стрелок
            key += sys.stdin.read(2)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

def main(args=None):

    if args is None:
        args = sys.argv

    rclpy.init()
    node = rclpy.create_node('teleop_twist_keyboard')

    qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1
    )

    pub = node.create_publisher(TwistStamped, 'cmd_vel', qos_profile)

    speed = 0.5
    turn = 1.0
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed, turn))
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if key == '\x03':  # CTRL-C для выхода
                    break

            twist = TwistStamped()
            current_time = time.time()
            # Разделяем на секунды и наносекунды
            twist.header.stamp.sec = int(current_time)  # Секунды
            twist.header.stamp.nanosec = int((current_time - twist.header.stamp.sec) * 1e9)  # Наносекунды
            twist.header.stamp.sec = int(time.time())
            twist.twist.linear.x = x * speed
            twist.twist.linear.y = y * speed
            twist.twist.linear.z = z * speed
            twist.twist.angular.x = 0.0
            twist.twist.angular.y = 0.0
            twist.twist.angular.z = th * turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = TwistStamped()
        current_time = time.time()
        # Разделяем на секунды и наносекунды
        twist.header.stamp.sec = int(current_time)  # Секунды
        twist.header.stamp.nanosec = int((current_time - twist.header.stamp.sec) * 1e9)  # Наносекунды
        twist.twist.linear.x = 0.0
        twist.twist.linear.y = 0.0
        twist.twist.linear.z = 0.0
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
