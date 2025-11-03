#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import PointStamped
import math
import time
import traceback
from sensor_msgs.msg import NavSatFix
from datetime import datetime
from pino_msgs.srv import Text   # custom service
from pathlib import Path
from pino_msgs.msg import AudioMSG   # ✅ custom message
from math import radians, sin, cos, sqrt, atan2
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

def haversine_distance(lat1, lon1, lat2, lon2):
    """Compute great-circle distance in meters between two GPS points."""
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2)
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))
class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')

        # States
        self.INIT = 0
        self.PERFORMING = 1
        self.NAVIGATION = 2
        self.WANDERING = 3

        self.timer_group = MutuallyExclusiveCallbackGroup()  # for periodic + scheduled timers

        self.state = self.INIT
        self.get_logger().info("Starting in INIT state")

        # Subscribers
        self.create_subscription(Int32, '/motion_cmd', self.cmd_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/gps_raw', self.gps_callback, 10)
        self.goal_reached_sub = self.create_subscription(Bool, 'goal_reached', self.goal_reached_callback, 10)

        # GPS state
        self.current_lat = None
        self.current_lon = None
        self.last_gps_time = None
        self.last_gps_lat = None
        self.last_gps_lon = None
        self.gps_stable = False

        self.voice = 'zf_xiaoyi'

        # Publishers
        self.waypoint_pub = self.create_publisher(PointStamped, '/way_point', 10)
        self.gps_status_pub = self.create_publisher(Bool, "/gps_status", 10)  # ✅ Bool publisher
        # self.stop_pub = self.create_publisher(Int32, '/stop_cmd', 10)   # stop nav/wandering
        self.mode_pub = self.create_publisher(Int32, '/behavior_mode', 10)
        self.audio_pub = self.create_publisher(AudioMSG, "audio_cmd", 10)
        
        self.create_timer(0.2, self.safe_wrapper(self.timer_callback), callback_group=self.timer_group)
        
    # -------- Error Wrapper -------- #
    def safe_wrapper(self, func):
        """Wraps any callback/timer to avoid crashing the node."""
        def wrapped(*args, **kwargs):
            try:
                return func(*args, **kwargs)
            except Exception as e:
                self.get_logger().error(f"Error in {func.__name__}: {e}\n{traceback.format_exc()}")
        return wrapped

    def gps_callback(self, msg: NavSatFix):
        try:
            lat, lon = msg.latitude, msg.longitude
            now = time.time()

            if self.last_gps_time is not None:
                dt = now - self.last_gps_time
                freq = 1.0 / dt if dt > 0 else 0.0
                dist = haversine_distance(self.last_gps_lat, self.last_gps_lon, lat, lon)

                if freq > 0.25 and dist < 3.0:
                    self.gps_stable = True
                else:
                    self.gps_stable = False
                    self.get_logger().warn(f"⚠️ GPS unstable: freq={freq:.2f}Hz, dist={dist:.2f}m")
            else:
                self.publish_audio(text = "已接收到卫星信号！！！")
                self.get_logger().info("⏳ Waiting for GPS history...")


            # update states
            self.last_gps_time = now
            self.last_gps_lat = lat
            self.last_gps_lon = lon
            self.current_lat, self.current_lon = lat, lon

            # publish status (Bool)
            status_msg = Bool()
            status_msg.data = self.gps_stable
            self.gps_status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f"❌ Failed to parse GPS NavSatFix: {e}")

    def publish_audio(self, text: str, cmd: str = 'speak', voice: str = 'zf_xiaoyi', volume: float = 1.5, speed: float = 0.8):
        msg = AudioMSG()
        msg.cmd = cmd
        msg.text = text
        msg.voice = voice
        msg.volume = volume
        msg.speed = speed
        self.audio_pub.publish(msg)
        self.get_logger().info(f"🎙️ Published AudioMSG: {msg}")

    def timer_callback(self):
        msg = Int32()
        msg.data = self.state
        self.mode_pub.publish(msg)
        now = time.time()
        if(now - self.last_gps_time > 4.0):
            self.last_gps_time = None
            self.gps_stable = False

        # if( self.last_gps_time is None and now - self.last_audio > 60.0):
            # self.publish_audio("注意，当前GPS信号不稳定。")
        if self.state == self.NAVIGATION:
            if(  (not self.gps_stable ) or (self.last_gps_time is None) ) :
                self.publish_audio("注意，当前GPS信号不稳定，已切换至牵引模式")
                self.get_logger().warn("❌ Navigation blocked: GPS unstable")
                self.state = self.INIT

        if self.state == self.WANDERING:
            # 3 m ahead waypoint (simplified)
            wp = PointStamped()
            wp.header.stamp = self.get_clock().now().to_msg()
            wp.header.frame_id = "base_link"
            wp.point.x = 4.0
            wp.point.y = 0.0
            wp.point.z = 0.0

            self.waypoint_pub.publish(wp)
            self.get_logger().info("Published wandering waypoint 3m ahead")

    def cmd_callback(self, msg: Int32):
        if(msg.data>10 or msg.data < 7):
            self.state =  msg.data//10

        self.get_logger().info(f"Received behavior mode: {self.state}")

    def goal_reached_callback(self, msg: Bool):
        if(msg.data):
            self.state = self.INIT
            self.get_logger().info(f"Goal Reached !!!!")

def main(args=None):
    # rclpy.init(args=args)
    # node = StateMachine()
    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()
    rclpy.init(args=args)
    node = StateMachine()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except Exception as e:
        node.get_logger().error(f"Fatal error in main loop: {e}\n{traceback.format_exc()}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
