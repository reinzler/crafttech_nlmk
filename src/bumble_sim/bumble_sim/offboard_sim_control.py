#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus
from std_msgs.msg import Float32MultiArray
from colorama import Fore


class OffboardControl(Node):
    """Node for controlling a rover in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_rover')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        self.vehicle_teleop_speed = self.create_subscription(Float32MultiArray, '/rover_velocity', self.vehicle_vel_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_status = VehicleStatus()

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.velocity_vx,  self.velocity_vy = 0.0, 0.0
        self.ready_flag = False

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)
        self.get_logger().info(f"{Fore.YELLOW}Publishing Offboard Control Mode{Fore.RESET}")

    def publish_velocity_setpoint(self, vx: float, vy: float):
        """Publish the trajectory setpoint for velocity control."""
        msg = TrajectorySetpoint()
        msg.velocity = [vx, vy, 0.0]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"{Fore.CYAN}Publishing velocity setpoints {[vx, vy, 0.0]}{Fore.RESET}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
        self.get_logger().info(f"Publishing Vehicle Command: {command}")

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 5:
            self.engage_offboard_mode()
            self.arm()
            self.ready_flag = True

        if self.ready_flag:
            # pass
            self.publish_velocity_setpoint(self.velocity_vx, self.velocity_vy)  # Adjust velocity as needed

        if self.offboard_setpoint_counter < 6:
            self.offboard_setpoint_counter += 1

    def vehicle_vel_callback(self, msg):
        if self.ready_flag:
            self.velocity_vx, self.velocity_vy = msg.data[0], msg.data[1]
            # self.publish_velocity_setpoint(msg.data[0], msg.data[1])


def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)


# #!/usr/bin/env python3
#
# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus
# from std_msgs.msg import Bool, Float32
# from colorama import Fore
#
#
# class OffboardControl(Node):
#     """Node for controlling a rover in offboard mode."""
#
#     def __init__(self) -> None:
#         super().__init__('offboard_control_rover')
#
#         # Configure QoS profile for publishing and subscribing
#         qos_profile = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             durability=DurabilityPolicy.TRANSIENT_LOCAL,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=1
#         )
#
#         # Create publishers
#         self.offboard_control_mode_publisher = self.create_publisher(
#             OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
#         self.trajectory_setpoint_publisher = self.create_publisher(
#             TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
#         self.vehicle_command_publisher = self.create_publisher(
#             VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
#
#         # Create subscribers
#         self.vehicle_status_subscriber = self.create_subscription(
#             VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
#         self.teleop_control_subscriber = self.create_subscription(
#             Bool, '/teleop_control', self.teleop_control_callback, qos_profile)
#         self.teleop_velocity_subscriber = self.create_subscription(
#             Float32, '/teleop_velocity', self.teleop_velocity_callback, qos_profile)
#
#         # Initialize variables
#         self.offboard_setpoint_counter = 0
#         self.vehicle_status = VehicleStatus()
#         self.ready_flag = False
#         self.arm_flag = False
#
#         # Create a timer to publish control commands
#         self.timer = self.create_timer(0.1, self.timer_callback)
#
#     def vehicle_status_callback(self, vehicle_status):
#         """Callback function for vehicle_status topic subscriber."""
#         self.vehicle_status = vehicle_status
#
#     def teleop_control_callback(self, msg):
#         """Callback function for teleop control commands."""
#         if msg.data:
#             if not self.arm_flag:
#                 self.arm()
#                 self.arm_flag = True
#             else:
#                 self.disarm()
#                 self.arm_flag = False
#
#     def teleop_velocity_callback(self, msg):
#         """Callback function for teleop velocity commands."""
#         if self.ready_flag:
#             vx = msg.data
#             vy = 0.0  # Assuming we are only controlling vx for simplicity
#             self.publish_velocity_setpoint(vx, vy)
#
#     def arm(self):
#         """Send an arm command to the vehicle."""
#         self.publish_vehicle_command(
#             VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
#         self.get_logger().info('Arm command sent')
#
#     def disarm(self):
#         """Send a disarm command to the vehicle."""
#         self.publish_vehicle_command(
#             VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
#         self.get_logger().info('Disarm command sent')
#
#     def engage_offboard_mode(self):
#         """Switch to offboard mode."""
#         self.publish_vehicle_command(
#             VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
#         self.get_logger().info("Switching to offboard mode")
#
#     def publish_offboard_control_heartbeat_signal(self):
#         """Publish the offboard control mode."""
#         msg = OffboardControlMode()
#         msg.position = False
#         msg.velocity = True
#         msg.acceleration = False
#         msg.attitude = False
#         msg.body_rate = False
#         msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
#         self.offboard_control_mode_publisher.publish(msg)
#         self.get_logger().info(f"{Fore.YELLOW}Publishing Offboard Control Mode{Fore.RESET}")
#
#     def publish_velocity_setpoint(self, vx: float, vy: float):
#         """Publish the trajectory setpoint for velocity control."""
#         msg = TrajectorySetpoint()
#         msg.velocity = [vx, vy, 0.0]
#         msg.yaw = 0.0
#         msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
#         self.trajectory_setpoint_publisher.publish(msg)
#         self.get_logger().info(f"{Fore.CYAN}Publishing velocity setpoints {[vx, vy, 0.0]}{Fore.RESET}")
#
#     def publish_vehicle_command(self, command, **params) -> None:
#         """Publish a vehicle command."""
#         msg = VehicleCommand()
#         msg.command = command
#         msg.param1 = params.get("param1", 0.0)
#         msg.param2 = params.get("param2", 0.0)
#         msg.param3 = params.get("param3", 0.0)
#         msg.param4 = params.get("param4", 0.0)
#         msg.param5 = params.get("param5", 0.0)
#         msg.param6 = params.get("param6", 0.0)
#         msg.param7 = params.get("param7", 0.0)
#         msg.target_system = 1
#         msg.target_component = 1
#         msg.source_system = 1
#         msg.source_component = 1
#         msg.from_external = True
#         msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
#         self.vehicle_command_publisher.publish(msg)
#         self.get_logger().info(f"Publishing Vehicle Command: {command}")
#
#     def timer_callback(self) -> None:
#         """Callback function for the timer."""
#         self.publish_offboard_control_heartbeat_signal()
#
#         if self.offboard_setpoint_counter == 5:
#             self.engage_offboard_mode()
#             self.ready_flag = True
#
#         if self.ready_flag:
#             # Placeholder for any additional periodic actions
#             pass
#
#         if self.offboard_setpoint_counter < 6:
#             self.offboard_setpoint_counter += 1
#
#
# def main(args=None) -> None:
#     print('Starting offboard control node...')
#     rclpy.init(args=args)
#     offboard_control = OffboardControl()
#     rclpy.spin(offboard_control)
#     offboard_control.destroy_node()
#     rclpy.shutdown()
#
#
# if __name__ == '__main__':
#     try:
#         main()
#     except Exception as e:
#         print(e)
