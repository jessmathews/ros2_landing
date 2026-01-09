#!/usr/bin/env python3
"""
Arming Test Script
------------------------------
Author: Jess Mathews
GitHub: https://github.com/jessmathews

Function:
Arms Quadcopter to check connection
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
import time


class ArmTest(Node):
    def __init__(self):
        super().__init__("Arm_Test_Node")
        qos_profile = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability = DurabilityPolicy.VOLATILE,
            history = HistoryPolicy.KEEP_LAST,
            depth = 10
        )

        # Subscribers 
        self.state_sub = self.create_subscription(State, "/mavros/state",self.state_callback,qos_profile)


        # Service Clients
        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")

        
        self.get_logger().info("Waiting for services...")
        self.arming_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info("Services ready!\n")

        # State Variables
        self.current_state = State()
    
    def state_callback(self,msg):
        self.current_state = msg

    def set_mode(self,mode:str)->bool:
        """
        Set Flight Mode        
        :param self: Description
        :param mode: Description
        """

        req = SetMode.Request()
        req.custom_mode = mode

        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self,future,timeout_sec=5.0)

        if future.result() and future.result().mode_sent:
            self.get_logger().info(f"{mode} SET\n")
            return True
        else:
            self.get_logger().error(f"Failed to set {mode}")
            return False

    def arm(self)->bool:
        """
        ARM the vechicle
        """
        self.get_logger().info("ARMING Vechicle...")

        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self,future,timeout_sec=5.0)

        if future.result() and future.result().success:
            self.get_logger().info("ARMED successfully.")
            return True
        else:
            self.get_logger().error("FAILED to ARM")
            return False
    def run_test(self):
        """
        Run the Test        
        :param self: Description
        """
        print("="*50)
        print("TEST: ARMING")
        print("="*50)
        print()
        self.get_logger().info("Waiting for FCU connection...")
        while not self.current_state.connected and rclpy.ok():
            rclpy.spin_once(self,timeout_sec=0.1)
            time.sleep(0.1)
        self.get_logger().info("Connected to FCU\n")

        self.get_logger().info("Starting Test...")
        self.get_logger().info("[1] Setting GUIDED mode...")

        if not self.set_mode("GUIDED"):
            return
        self.get_logger().info("GUIDED mode SET")        
        self.get_logger().info("Trying to arm...")
        if not self.arm():
            return
        self.get_logger().info("Vehicle ARMED Successfully!")
        time.sleep(5)
        print("byeeeee....")
            
def main():
    rclpy.init()
    node = ArmTest()
    
    try:
        node.run_test()
    except KeyboardInterrupt:
        print("Mission interrupted by user\n")

    finally:
        node.destroy_node()
        rclpy.shutdown()
        exit()

if __name__ == "__main__":
    main()