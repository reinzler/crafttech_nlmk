import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
import numpy as np
from scipy.integrate import cumulative_trapezoid
import time


class MathNode(Node):
    def __init__(self):
        super().__init__('math_node')
        self.current_time, self.time_list  = 0.0, [0.0]
        self.velocity = 0.0
        self.velocity_left, self.velocity_right = 0.0, 0.0
        self.angular_velocity, self.angular_velocity_list = 0.0, []
        self.theta, self.theta_0, self.theta_list = 0.0, 0.0, []
        self.current_x, self.current_y = 0.0, 0.0

        # Расстояние между гусеницами/колесами поперечное
        self.d = 1.1
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.cmd_sub = self.create_subscription(TwistStamped, '/cmd_vel', self.cmd_callback, qos_profile)



    def cmd_callback(self, msg):
        self.velocity = msg.twist.linear.x
        self.angular_velocity = msg.twist.angular.z
        self.angular_velocity_list.append(self.angular_velocity)

        self.current_time = msg.header.stamp.sec
        self.get_logger().info(f"Time: {self.current_time}")
        self.time_list.append(self.current_time)

        self.calculate_velocities(), self.calculate_azimuth()

    def calculate_velocities(self):
        self.velocity_left = self.velocity - (self.angular_velocity * self.d) / 2
        self.velocity_right = self.velocity + (self.angular_velocity * self.d) / 2

    def calculate_azimuth(self):
        try:
            self.theta = cumulative_trapezoid(self.angular_velocity_list, self.time_list, initial=self.theta_0)
        except ValueError:
            self.theta = 0
        self.get_logger().info(f"Theta: {self.theta}")


def main(args=None):
    rclpy.init(args=args)
    node = MathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
