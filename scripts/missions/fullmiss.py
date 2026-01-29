#!/usr/bin/env python3
"""
QR code mission test script
------------------------------
Author: Jess Mathews
GitHub: https://github.com/jessmathews

Function:
Aligns centre of camera frame and centre of qr code using cmd_vel commands with a proportional controller
Then drops payload via servo and returns to launch
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, NavSatFix
from geometry_msgs.msg import PoseStamped, Twist
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.msg import HomePosition, GlobalPositionTarget
from std_msgs.msg import Float64

from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, CommandLong
from cv_bridge import CvBridge
import cv2
import time
from qrdet import QRDetector
import math
from pyzbar.pyzbar import decode



class QRTest(Node):
    def __init__(self, lat, lon):
        super().__init__("FULL_MISSION")
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.detector = QRDetector(model_size="n")
        self.bridge = CvBridge()

        # Goto locations
        self.dest_lat = lat
        self.dest_lon = lon
        
        # Publishers
        # self.vel_pub = self.create_publisher(TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10)
        self.vel_pub = self.create_publisher(Twist,"/mavros/setpoint_velocity/cmd_vel_unstamped",10)
        self.global_setpoint_pub = self.create_publisher(GeoPoseStamped,"/mavros/setpoint_position/global",10)
        self.raw_global_pub = self.create_publisher(GlobalPositionTarget, '/mavros/setpoint_raw/global', 10)


        # Subscribers
        self.state_sub = self.create_subscription(State, "/mavros/state", self.state_callback, qos_profile)
        self.image_sub = self.create_subscription(Image, "/camera/image_raw", self.image_callback, qos_profile)
        self.pose_sub = self.create_subscription(PoseStamped, "/mavros/local_position/pose", self.pose_callback, qos_profile)
        self.home_sub= self.create_subscription(HomePosition, "/mavros/home_position/home", self.home_callback, qos_profile)
        self.global_sub = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.global_callback, qos_profile)
        self.rel_alt_sub = self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.rel_alt_callback, qos_profile)
        
        # Service Clients
        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.takeoff_client = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.command_client = self.create_client(CommandLong, "/mavros/cmd/command")

        
        
        print("Waiting for services...")
        self.set_mode_client.wait_for_service(timeout_sec=10.0)
        self.arming_client.wait_for_service(timeout_sec=10.0)
        self.takeoff_client.wait_for_service(timeout_sec=10.0)
        self.command_client.wait_for_service(timeout_sec=10.0)
        print("Services ready!\n")
        
        # Parameters
        self.kp = 0.05
        self.max_vel = 1.0
        self.land_threshold = 10.0  # center within 10 pixels
        self.required_stable_time = 2.0 
        self.descend_speed = -0.3     # m/s
        self.ascend_speed  =  0.4     # m/s
        self.z_hold_speed  =  0.0

        self.min_land_altitude  = 0.5     # trigger land below this
        self.max_search_alt = 6.0    # don't climb forever

        # Servo parameters
        self.servo_channel = 9        # Servo output channel (AUX1 = 9, AUX2 = 10, etc.)
        self.servo_drop_pwm = 2000    # PWM value to drop egg (adjust based on your servo)
        self.servo_hold_pwm = 1000    # PWM value to hold egg (adjust based on your servo)
        self.rtl_altitude = 5.0       # Defautl Altitude
                
        # State variables
        self.centered_start_time = None
        self.landing_phase = False 
        self.final_land_triggered = False
        self.qr_detection = False 
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.prev_time = None
        self.prev_err_x = 0.0
        self.prev_err_y = 0.0
        self.err_x = 0.0
        self.err_y = 0.0
        self.mission_complete = False
        self.payload_dropped = False
        self.rtl_initiated = False
        self.home_position= None
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_amsl = 0.0
        self.current_rel_alt = 0.0
        self.qr_data_value = None



        self.get_logger().info("Mission Test Node Started.")
            
    
    def state_callback(self, msg):
        self.current_state = msg

    def global_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_amsl = msg.altitude
    
    def rel_alt_callback(self, msg):
        # msg.data is the actual altitude value
        self.current_rel_alt = msg.data
    
    def pose_callback(self, msg):
        # Update current altitude from Local Position (Z)
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
        self.get_logger().info(f"Taking off to {altitude:.2f}m...")

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
    
    def control_servo(self, pwm_value: int) -> bool:
        """
        Control servo using MAVLink DO_SET_SERVO command
        :param pwm_value: PWM value (typically 1000-2000)
        """
        self.get_logger().info(f"Setting servo channel {self.servo_channel} to {pwm_value} PWM")
        
        req = CommandLong.Request()
        req.command = 183  # MAV_CMD_DO_SET_SERVO
        req.param1 = float(self.servo_channel)  # Servo channel
        req.param2 = float(pwm_value)           # PWM value
        
        future = self.command_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            self.get_logger().info(f"Servo command sent successfully\n")
            return True
        else:
            self.get_logger().error("Failed to send servo command\n")
            return False
    
    def drop_payload(self) -> bool:
        """Drop the payload by activating servo"""
        if self.payload_dropped:
            return True
            
        self.get_logger().info("=" * 50)
        self.get_logger().info("DROPPING PAYLOAD!")
        self.get_logger().info("=" * 50)
        
        # Activate servo to drop
        if self.control_servo(self.servo_drop_pwm):
            self.payload_dropped = True
            time.sleep(2)  # Wait for drop to complete
            
            # Return servo to hold position
            self.control_servo(self.servo_hold_pwm)
            return True
        
        return False
          
    def wait_for_altitude(self, target_alt: float, tolerance: float = 0.2) -> bool:
        """
        Wait for altitude to be reached
        
        :param target_alt: Target altitude to be reached
        """
        print(f"Waiting to reach {target_alt:.2f}m...")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            current_alt = self.current_pose.pose.position.z
            print(f"Altitude: {current_alt:.2f}m / {target_alt:.2f}m ", end="\r")

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
            
            print(f"Distance to target: {dist:.2f}m", end='\r')

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



    def set_mode(self, mode):
        """Set flight mode"""
        if self.mission_complete:
            return
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


        
    
    def image_callback(self, msg):
        if self.final_land_triggered:
            return 
        if not self.qr_detection:
            return
        # print(type(msg))
        #  IMAGE 
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        img = cv2.resize(cv_img, (320, 320))
        h, w = img.shape[:2]
        cx_img, cy_img = w // 2, h // 2
        # convert to grayscale only if necessary qrdet is said to work on "BGR color profiles better" and change is_bgr to False
        # img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        detections = self.detector.detect(image=img, is_bgr=True)

        #  VELOCITY MSG 
        # vel_cmd = TwistStamped()
        # vel_cmd.header.stamp = self.get_clock().now().to_msg()
        # vel_cmd.header.frame_id = ""
        vel_cmd = Twist()

        # current_alt = self.current_pose.pose.position.z
        current_alt = self.current_rel_alt

        #  PARAMETERS 
        kp_xy = 0.004
        kd_xy = 0.008

        xy_deadband = 10          # pixels
        max_xy_vel = 0.8

        descend_speed = -0.3
        

        #  DEFAULT 
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0

        now = time.time()
        if self.prev_time is None:
            self.prev_time = now
            return

        dt = max(now - self.prev_time, 1e-3)
        self.prev_time = now

        #  QR DETECTED 
        if detections:
            box = detections[0]["bbox_xyxy"]
            cx = (box[0] + box[2]) / 2
            cy = (box[1] + box[3]) / 2

            self.err_x = cx - cx_img
            self.err_y = cy - cy_img

            kp_yaw = 0.005
            max_yaw_rate = 0.4
            yaw_deadband = 20  # pixels

            # Yaw control
            if abs(self.err_x) > yaw_deadband:
                yaw_rate = -kp_yaw * self.err_x
                yaw_rate = max(min(yaw_rate, max_yaw_rate), -max_yaw_rate)
            else:
                yaw_rate = 0.0
            
            vel_cmd.angular.z = yaw_rate


            # Deadband
            if abs(self.err_x) < xy_deadband:
                self.err_x = 0.0
            if abs(self.err_y) < xy_deadband:
                self.err_y = 0.0

            # Derivative
            derr_x = (self.err_x - self.prev_err_x) / dt
            derr_y = (self.err_y - self.prev_err_y) / dt

            self.prev_err_x = self.err_x
            self.prev_err_y = self.err_y

            vx = 0
            vy = 0

            yaw_aligned = abs(self.err_x) < yaw_deadband

            vx = 0.0
            vy = 0.0

            if yaw_aligned:
                if abs(self.err_x) > self.land_threshold:
                    vy = (kp_xy * self.err_x + kd_xy * derr_x)
                elif abs(self.err_y) > self.land_threshold:
                    vx = (kp_xy * self.err_y + kd_xy * derr_y)
            else:
                print("aligning yaw")
                # freeze XY until yaw aligned
                vx = 0.0
                vy = 0.0

            # QR size scaling
            qr_area = (box[2] - box[0]) * (box[3] - box[1])
            scale = max(0.2, min(1.0, 15000.0 / max(qr_area, 1.0)))
            vx *= scale
            vy *= scale

            # Velocity limit
            vel_cmd.linear.x = max(min(vx, max_xy_vel), -max_xy_vel)
            vel_cmd.linear.y = max(min(vy, max_xy_vel), -max_xy_vel)

            qr_centered = (
                abs(self.err_x) < self.land_threshold and
                abs(self.err_y) < self.land_threshold
            )
            
            if qr_centered:
                try:
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    self.qr_data_value = decode(img)[0].data.decode('utf-8')
                    print(self.qr_data_value)
                except:
                    print()
            
            #  Z CONTROL 
            if qr_centered and current_alt <= self.min_land_altitude:
                self.get_logger().info("QR centered & low altitude → LAND")
                self.qr_detection = False
                print(self.qr_data_value)
                self.set_mode("LAND")
                self.final_land_triggered = True
                return

            if qr_centered:
                vel_cmd.linear.z = descend_speed
                # Freeze XY during descent
                vel_cmd.linear.x = 0.0
                vel_cmd.linear.y = 0.0

            else:
                vel_cmd.linear.z = 0.0

        #  QR LOST 
        else:
            if self.final_land_triggered:
                return
  
            self.get_logger().info("Entering RECOVERY!")
            # P control
            vx = -(kp_xy * self.err_x)
            vy = (kp_xy * self.err_y)

            # fix weakest axis first 
            if abs(self.err_x) > abs(self.err_y):
                vx = 0.0
            else:
                vy = 0.0

            # Start moving towards last seen position
            print("moving to qr!", end="\r")
            vel_cmd.linear.x = max(min(vx, max_xy_vel), -max_xy_vel)
            vel_cmd.linear.y = max(min(vy, max_xy_vel), -max_xy_vel)


        #  PUBLISH 
        self.vel_pub.publish(vel_cmd)

        #  DEBUG (optional) 
        print(f"ALT={current_alt:.3f} QR={'YES' if detections else 'NO'} "
              f"V=({vel_cmd.linear.x:.2f},"
              f"{vel_cmd.linear.y:.2f},"
              f"{vel_cmd.linear.z:.2f},"
              f"ERR=({self.err_x:.2f},{self.err_y:.2f})", end='\r')
    
    def run_mission(self):
        """Mission Execution"""
        print("="*70)
        print("MISSION: TAKEOFF → GPS → QR LANDING → DROP PAYLOAD → RTL")
        print("="*70)
        print()
        
        print("Waiting for FCU connection...")
        while not self.current_state.connected and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        print("Connected to FCU\n")
        print("starting timer...")
        start_time = time.time()


        print("[1] Set GUIDED mode")
        if not self.set_mode("GUIDED"):
            return
        time.sleep(1)

        print("[2] ARM throttle")
        if not self.arm():
            return
        time.sleep(2)

        # calculate relative target altitude
        relative_target_height = self.current_pose.pose.position.z + self.rtl_altitude
        
        print("Waiting for home position...")
        while self.home_position is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        print(f"Home position locked: lat={self.home_position.geo.latitude:.6f}, "
              f"lon={self.home_position.geo.longitude:.6f}\n")
        



        print(f"[3] Takeoff to {self.rtl_altitude}")
        if not self.takeoff(self.rtl_altitude):
            return
        
        self.wait_for_altitude(relative_target_height)
        time.sleep(2)
        print()
        
        print("[4] Navigate to GPS Coordinates")
        #latitude=8.543853, longitude=76.904514
        if not self.goto_gps(lat=self.dest_lat, lon=self.dest_lon, altitude=self.rtl_altitude):
            return
        self.qr_detection = True
        
        print()
        print("[5] Land on QR")
        print("Starting qr precision landing...")

        while not self.final_land_triggered and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
        
        print()
        print("[6] Waiting for landing to complete...")
        # Wait for vehicle to land (check altitude near 0)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            current_alt = self.current_pose.pose.position.z
            if current_alt < 0.5:  # Close to ground
                break
            time.sleep(0.2)
        
        time.sleep(2)  # Stabilize on ground
        print()
        
        print("[7] Drop Payload via Servo")
        if not self.drop_payload():
            self.get_logger().error("Failed to drop payload!")
        time.sleep(1)
        print()
        
        
        
        print("[8] Switch to GUIDED mode for takeoff")
        if not self.set_mode("GUIDED"):
            return
        time.sleep(1)
        print()
        print("ARM throttle")
        if not self.arm():
            return
        time.sleep(1)

        relative_target_height = self.current_pose.pose.position.z + self.rtl_altitude
        
        print(f"[9] Takeoff to {self.rtl_altitude}m")
        if not self.takeoff(self.rtl_altitude):
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
            self.get_logger().info("Land activated - Returning to launch point")
        
        print()
        print(f"Mission duration: {((time.time() - start_time)/60.0):.2f} minutes")
        print("="*70)
        print("MISSION COMPLETE!")
        print("="*70)




def main():
    rclpy.init()
    # 8.544100,76.904188
    lat, lon = input("Enter Coordinates: ").split(',')
    lat = float(lat)
    lon = float(lon)

    print(f"CHECK: Latitude: {lat}, Longitude: {lon}")
    dec = input(f"Do you want to go to {lat}, {lon} (Y/N): ")

    if dec.lower() == "n":
        raise Exception("ABORT")
    
    print (f"Go to {lat}, {lon}")
    node = QRTest(lat, lon)

    try:
        # rclpy.spin(node)
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
