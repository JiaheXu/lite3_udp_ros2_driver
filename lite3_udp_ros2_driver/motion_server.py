# ros2 topic pub --once /motion_cmd std_msgs/msg/Int32 "{data: 1}"

#!/usr/bin/env python3
import socket
import struct
import rclpy
import time

from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from pino_msgs.msg import SimpleCMD, ComplexCMD
from std_msgs.msg import Bool  # Needed for stand/sit callbacks
     
class ROS2QNX(Node):
    def __init__(self):
        super().__init__('ros2qnx')

        # Create UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.qnx_addr = ("192.168.1.120", 43893)

        # Subscribers
        # self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        # self.create_subscription(Twist, "cmd_vel_corrected", self.cmd_vel_callback, 10)
        # self.create_subscription(Int32, "kick_ball", self.kick_ball_callback, 10)
        self.create_subscription(SimpleCMD, "simple_cmd", self.simple_cmd_callback, 10)
        self.create_subscription(Int32, "motion_cmd", self.motion_callback, 10)
        # self.create_subscription(ComplexCMD, "complex_cmd", self.complex_cmd_callback, 10)
        self.init()
        
        
        # ---------------- Timer ---------------- #
        # Send a command every 1 second
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info("----- ros2qnx node up -----")

    # ---------------- Timer Callback ---------------- #
    def timer_callback(self):
        # Example command: send simple command with cmd_code=999, value=1, type=0
        self.send_simple_cmd(0x21040001, 1, 0)
        #self.get_logger().info("Sent periodic command to QNX")

    def motion_callback(self, msg: Int32):
        print("got a msg !!!!")
        if(msg.data == -1):
            self.stop()
        if(msg.data == 0):
            self.init()
        elif (msg.data == 1):
            self.stand()
        elif (msg.data == 2):
            self.sit()
        elif (msg.data == 3):
            self.shaking()
        elif (msg.data == 4):
            self.shake_hands()
        elif (msg.data == 5):
            self.space_step()


    # ---------------- Callbacks ---------------- #
    def stop(self):
        #cmd_code = 0x21010c0b
        cmd_code = 0x21010C0A
        cmd_value = 7
        cmd_type = 0
        self.send_simple_cmd(cmd_code, cmd_value, cmd_type)

        self.stand()

    def init(self):
        cmd_code = 0x21010C05
        cmd_value = 0
        cmd_type = 0
        self.send_simple_cmd(cmd_code, cmd_value, cmd_type)

    def stand(self):

        time.sleep(1.0)
        cmd_code = 0x21010202
        cmd_value = 0
        cmd_type = 0
        self.send_simple_cmd(cmd_code, cmd_value, cmd_type)
        
        time.sleep(0.5)
        
        cmd_code = 0x21010C03
        cmd_value = 0
        cmd_type = 0
        self.send_simple_cmd(cmd_code, cmd_value, cmd_type)
    
    def sit(self):
        self.stop()
        time.sleep(1.0)
        cmd_code = 0x21010202
        cmd_value = 0
        cmd_type = 0
        self.send_simple_cmd(cmd_code, cmd_value, cmd_type)

    def shaking(self):
        cmd_code = 0x21010204
        cmd_value = 0
        cmd_type = 0
        self.send_simple_cmd(cmd_code, cmd_value, cmd_type)

        # time.sleep(5.0)
        # self.stop()
        print("shaking finished")

    def shake_hands(self):

        cmd_code = 0x21010507
        cmd_value = 0
        cmd_type = 0
        self.send_simple_cmd(cmd_code, cmd_value, cmd_type)

        time.sleep(10.0)
        self.stop()

    def space_step(self):

        cmd_code = 0x2101030C
        cmd_value = 0
        cmd_type = 0
        self.send_simple_cmd(cmd_code, cmd_value, cmd_type)
        time.sleep(4.0)
        self.stop()

        print("space_step finished")




    def cmd_vel_callback(self, msg: Twist):
        # linear.x
        self.send_complex_cmd(320, 8, 1, msg.linear.x)
        # linear.y
        self.send_complex_cmd(325, 8, 1, msg.linear.y)
        # angular.z (negated)
        self.send_complex_cmd(321, 8, 1, -msg.angular.z)

    def kick_ball_callback(self, msg: Int32):
        self.send_simple_cmd(503, msg.data, 0)

    def simple_cmd_callback(self, msg: SimpleCMD):
        self.send_simple_cmd(msg.cmd_code, msg.cmd_value, msg.type)

    def complex_cmd_callback(self, msg: ComplexCMD):
        self.send_complex_cmd(msg.cmd_code, msg.cmd_value, msg.type, msg.data)

    # ---------------- UDP Send Helpers ---------------- #
    def send_simple_cmd(self, cmd_code: int, cmd_value: int, cmd_type: int):
        packet = struct.pack("<iii", cmd_code, cmd_value, cmd_type)
        self.sock.sendto(packet, self.qnx_addr)

    def send_complex_cmd(self, cmd_code: int, cmd_value: int, cmd_type: int, data: float):
        packet = struct.pack("<iiid", cmd_code, cmd_value, cmd_type, data)
        self.sock.sendto(packet, self.qnx_addr)


def main(args=None):
    rclpy.init(args=args)
    node = ROS2QNX()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

