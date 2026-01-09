#!/usr/bin/env python3
"""
Takeoff Test Script
------------------------------
Author: Jess Mathews
GitHub: https://github.com/jessmathews

Mission Steps: 

"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped
import time


class TakeoffTest(Node):
    def __init__(self):
        super().__init__("TakeoffTest")

        # QoS profile for MAVROS topics (BEST_EFFORT reliability)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.state_sub = self.create_subscription(
            State,
            "/mavros/state",
            self.state_callback,
            qos_profile
        )

        self.local_pos_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.local_pos_callback,
            qos_profile
        )

        # Publisher for setpoints (will only be used AFTER takeoff)
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            "/mavros/setpoint_position/local",
            10
        )

        # Service clients
        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.takeoff_client = self.create_client(CommandTOL, "/mavros/cmd/takeoff")

        # State variables
        self.current_state = State()
        self.current_pose = PoseStamped()

        # Wait for services
        self.get_logger().info("Waiting for services...")
        self.arming_client.wait_for_service(timeout_sec=10.0)
        self.set_mode_client.wait_for_service(timeout_sec=10.0)
        self.takeoff_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info("Services ready!\n")

    def state_callback(self, msg):
        self.current_state = msg

    def local_pos_callback(self, msg):
        self.current_pose = msg

    def set_mode(self, mode):
        """Set flight mode"""
        self.get_logger().info(f"Setting {mode} mode...")
        req = SetMode.Request()
        req.custom_mode = mode

        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().mode_sent:
            self.get_logger().info(f"{mode} mode set")
            return True
        else:
            self.get_logger().info(f"Failed to set {mode} mode")
            return False

    def arm(self):
        """Arm the vehicle"""
        self.get_logger().info("Arming throttle...")
        req = CommandBool.Request()
        req.value = True

        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().success:
            self.get_logger().info("Armed")
            return True
        else:
            self.get_logger().error("Failed to arm")
            return False

    def takeoff(self, altitude:float) -> bool:
        """Takeoff to specified altitude"""
        self.get_logger().info(f"Taking off to {altitude}m...")

        req = CommandTOL.Request()
        req.altitude = altitude

        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().success:
            self.get_logger().info(f"Takeoff command sent")
            return True
        else:
            self.get_logger().error("Takeoff command failed")
            return False

    def wait_for_altitude(self, target_alt, tolerance=0.3):
        """Wait until altitude is reached"""
        self.get_logger().info(f"Waiting to reach {target_alt}m altitude...")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            current_alt = self.current_pose.pose.position.z

            print(f"  Altitude: {current_alt:.2f}m / {target_alt}m", end="\r")

            if abs(current_alt - target_alt) < tolerance:
                self.get_logger().info(f"\n✓ Reached altitude {current_alt:.2f}m\n")
                return True

            time.sleep(0.2)

    def run_test(self):
        """Execute the mission"""
        print("="*50)
        print("MISSION: Takeoff -> RTL")
        print("="*50)
        print()

        # Wait for connection
        self.get_logger().info("Waiting for FCU connection...")
        while not self.current_state.connected and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        self.get_logger().info("Connected to FCU\n")

        # Step 1: Set GUIDED mode
        self.get_logger().info("[1] Set GUIDED mode")
        if not self.set_mode("GUIDED"):
            return
        time.sleep(1)

        # Step 2: Arm
        self.get_logger().info("[2] Arm throttle")
        if not self.arm():
            return
        time.sleep(2)

        # Step 3: Takeoff to 10m
        self.get_logger().info("[3] Takeoff to 2m")
        if not self.takeoff(2.0):
            return

        # Wait for takeoff to complete
        self.wait_for_altitude(2.0)
        time.sleep(2)

        # Step 5: Wait 3 seconds
        self.get_logger().info("[4] Wait 3 seconds")
        for i in range(3, 0, -1):
            print(f"  {i}...", end="\r")
            time.sleep(1)
        self.get_logger().info("Wait complete\n")

        # Step 6: RTL
        self.get_logger().info("[5] Return to Launch (RTL)")
        self.set_mode("RTL")

        print("="*50)
        print("MISSION COMPLETE!")
        print("="*50)
        print("\nMonitoring RTL... (Ctrl+C to exit)")

        # Monitor while returning
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.5)

        except KeyboardInterrupt:
            pass


def main():
    rclpy.init()
    node = TakeoffTest()

    try:
        node.run_test()
    except KeyboardInterrupt:
        print("\n\nMission interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()