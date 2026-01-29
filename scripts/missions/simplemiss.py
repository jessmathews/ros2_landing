#!/usr/bin/env python3
"""
GPS Land and RTL Mission Script
------------------------------
Author: Jess Mathews
GitHub: https://github.com/jessmathews

Function:
Navigate to GPS coordinates, land without disarming, and return to launch
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import HomePosition, GlobalPositionTarget
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
import time
import math
import sys

class GPSLandRTL(Node):
    def __init__(self, lat, lon):
        super().__init__("gps_land_rtl_node")
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.raw_global_pub = self.create_publisher(GlobalPositionTarget, '/mavros/setpoint_raw/global', 10)


        # Subscribers
        self.state_sub = self.create_subscription(State, "/mavros/state", self.state_callback, qos_profile)
        self.pose_sub = self.create_subscription(PoseStamped, "/mavros/local_position/pose", self.pose_callback, qos_profile)
        self.home_sub= self.create_subscription(HomePosition, "/mavros/home_position/home", self.home_callback, qos_profile)
        self.global_sub = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.global_callback, qos_profile)

        # Service Clients
        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.takeoff_client = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        
        print("Waiting for services...")
        self.set_mode_client.wait_for_service(timeout_sec=10.0)
        self.arming_client.wait_for_service(timeout_sec=10.0)
        self.takeoff_client.wait_for_service(timeout_sec=10.0)
        print("Services ready!\n")
        
        # Mission Parameters
        self.target_latitude = lat
        self.target_longitude = lon
        self.rtl_altitude = 4.0
        
        # State variables
        self.final_land_triggered = False
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.mission_complete = False
        self.rtl_initiated = False
        self.home_position= None
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_amsl = 0.0

        self.get_logger().info("GPS Land RTL Mission Node Started.")
    
    def state_callback(self, msg):
        self.current_state = msg

    def pose_callback(self, msg):
        self.current_pose = msg
    def home_callback(self, msg):
        """Callback for home position"""
        if self.home_position is not None:
            return
        self.home_position = msg
        if self.home_position:
            self.get_logger().info(
                f"Home position received: "
                f"lat={msg.geo.latitude:.6f}, "
                f"lon={msg.geo.longitude:.6f}, "
                f"alt={msg.geo.altitude:.2f}m"
            )
    def global_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_amsl = msg.altitude
    
    def get_distance_metres(self, lat1, lon1, lat2, lon2):
        """
        Returns the distance in metres between two points (lat, lon).
        """
        dlat = (lat2 - lat1) * math.pi / 180
        dlon = (lon2 - lon1) * math.pi / 180
        a = math.sin(dlat/2) * math.sin(dlat/2) + \
            math.cos(lat1 * math.pi / 180) * math.cos(lat2 * math.pi / 180) * \
            math.sin(dlon/2) * math.sin(dlon/2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return 6371000 * c
    

    def arm(self) -> bool:
        """Arm the vehicle"""
        self.get_logger().info("Arming throttle...")
        req = CommandBool.Request()
        req.value = True

        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().success:
            self.get_logger().info("Armed\n")
            return True
        else:
            print("Failed to arm\n")
            return False
    
    def takeoff(self, altitude: float) -> bool:
        """Takeoff to specific altitude"""
        self.get_logger().info(f"Taking off to {altitude}m...")

        req = CommandTOL.Request()
        req.altitude = altitude
        
        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().success:
            self.get_logger().info(f"Takeoff command sent\n")
            return True
        else:
            self.get_logger().error("Takeoff command failed\n")
            return False
    
    def wait_for_altitude(self, target_alt: float, tolerance: float = 0.2) -> bool:
        """Wait for altitude to be reached"""
        print(f"Waiting to reach {target_alt}m...")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            current_alt = self.current_pose.pose.position.z
            print(f"Altitude: {current_alt:.2f}m / {target_alt}m ", end="\r")

            if abs(current_alt - target_alt) < tolerance:
                self.get_logger().info(f"Reached Altitude {current_alt:.2f}m ")
                return True
            
            time.sleep(0.2)

    def goto_gps(self, lat: float, lon: float, altitude: float, timeout: float = 30.0):
        target = GlobalPositionTarget()
        target.coordinate_frame = 6 # REL_ALT
        target.type_mask = 4088 
        target.latitude = lat
        target.longitude = lon
        target.altitude = altitude 
        
        self.get_logger().info(f"Navigating to {lat:.7f}, {lon:.7f} at {altitude}m")

        start_time = self.get_clock().now()
        while rclpy.ok():
            # 1. Publish the setpoint
            target.header.stamp = self.get_clock().now().to_msg()
            self.raw_global_pub.publish(target)
            
            # 2. Spin to get latest GPS updates from subscribers
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # 3. Check distance
            # NOTE: Ensure you have a self.current_lat/lon updated in global_callback!
            dist = self.get_distance_metres(self.current_lat, self.current_lon, lat, lon)
            
            print(f"Distance to target: {dist:.2f} m", end='\r')

            if dist < 1.0: # Reach within 1 meter
                self.get_logger().info("\nTarget reached!")
                break
                
            # 4. Safety Timeout
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout:
                self.get_logger().warn("\nNavigation timeout reached!")
                break
                
            time.sleep(0.1)
        return True

    def set_mode(self, mode: str) -> bool:
        """Set flight mode"""
        self.get_logger().info(f"Setting {mode} mode")
        req = SetMode.Request()
        req.custom_mode = mode

        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().mode_sent:
            self.get_logger().info(f"{mode} set\n")
            return True
        else:
            self.get_logger().error("Failed to set mode")
            return False

    def trigger_final_land(self):
        """Trigger final landing sequence"""
        if self.final_land_triggered:
            return
        self.final_land_triggered = True

    def run_mission(self):
        """Mission Execution"""
        print("="*70)
        print("MISSION: TAKEOFF → NAVIGATE GPS → LAND → RTL")
        print("="*70)
        print()
        
        print("Waiting for FCU connection...")
        while not self.current_state.connected and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        print("Connected to FCU\n")
        
        print("Starting timer...")
        start_time = time.time()

        print("[1] Set GUIDED mode")
        if not self.set_mode("GUIDED"):
            return
        time.sleep(1)

        print("[2] ARM throttle")
        if not self.arm():
            return
        time.sleep(2)
        relative_target_height = self.current_pose.pose.position.z + self.rtl_altitude

        print("Waiting for home position...")
        while self.home_position is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        print(f"Home position locked: lat={self.home_position.geo.latitude:.6f}, "
              f"lon={self.home_position.geo.longitude:.6f}\n")
        

        print(f"[3] Takeoff to {self.rtl_altitude}m")
        if not self.takeoff(self.rtl_altitude):
            return
        
        self.wait_for_altitude(relative_target_height)
        time.sleep(1)
        print()
        
        print(f"[4] Navigate to GPS Coordinates ({self.target_latitude}, {self.target_longitude})")
        if not self.goto_gps(lat=self.target_latitude, lon=self.target_longitude,altitude=self.rtl_altitude):
            return
        
        print()
        print("[5] Landing at target location")
        if not self.set_mode("LAND"):
            return
        
        self.trigger_final_land()
        
        # Wait for landing to complete
        print("Waiting for landing to complete...")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            current_alt = self.current_pose.pose.position.z
            print(f"Altitude: {current_alt:.2f}m ", end="\r")
            
            if current_alt < 0.5:  # Close to ground
                break
            time.sleep(0.2)
        
        time.sleep(2)  # Stabilize on ground
        print()
        self.get_logger().info("Landing complete")
        print()
        
        print("[8] Switch to GUIDED mode for takeoff")
        if not self.set_mode("GUIDED"):
            return
        time.sleep(1)
        relative_target_height = self.current_pose.pose.position.z + self.rtl_altitude
        print()
        print("ARM throttle")
        if not self.arm():
            return
        time.sleep(2)

        
        
        print(f"[9] Takeoff to {self.rtl_altitude}m")
        if not self.takeoff(relative_target_height):
            return
        
        self.wait_for_altitude(relative_target_height)
        time.sleep(1)
        print()

        print(f" Navigate back to home position")
        if self.home_position:
            print(f"    Home: lat={self.home_position.geo.latitude:.6f}, "
                  f"lon={self.home_position.geo.longitude:.6f}, "
                  f"alt={self.rtl_altitude}m")
            
            if not self.goto_gps(
                lat=self.home_position.geo.latitude,
                lon=self.home_position.geo.longitude,
                altitude=self.rtl_altitude,
                timeout=20.0
            ):
                return
        else:
            self.get_logger().warn("No home position available, skipping navigation")
        
        time.sleep(1)
        print()
        
        print("[10] Initiating Land (Landing at home point)")
        if self.set_mode("LAND"):
            self.rtl_initiated = True
            self.mission_complete = True
            self.get_logger().info("Land activated at launch point")

        print()
        print(f"Mission duration: {((time.time() - start_time)/60.0):.2f} minutes")
        print("="*70)
        print("MISSION COMPLETE!")
        print("="*70)


def main():
    rclpy.init()
    lat, lon = input("Enter Coordinates: ").split(',')
    lat = float(lat)
    lon = float(lon)

    print(f"CHECK: Latitude: {lat}, Longitude: {lon}")
    dec = input(f"Do you want to go to {lat}, {lon} (Y/N): ")

    if dec.lower() == "n":
        raise Exception("ABORT")
    
    node = GPSLandRTL(lat, lon)
    
    try:    
        node.run_mission()
        
        # Keep node alive for RTL to complete
        while rclpy.ok() and node.rtl_initiated:
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
