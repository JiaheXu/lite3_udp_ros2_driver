#!/usr/bin/env python3
import socket
import struct
import rclpy
import traceback

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int32, String
from pino_msgs.msg import SimpleCMD, ComplexCMD


class ROS2QNX(Node):
    def __init__(self):
        super().__init__('ros2qnx')

        # --- Callback groups ---
        self.cmd_group = MutuallyExclusiveCallbackGroup()    # for commands
        self.state_group = MutuallyExclusiveCallbackGroup()  # for recv_state
        self.timer_group = MutuallyExclusiveCallbackGroup()  # for periodic + scheduled timers

        # --- UDP socket ---
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind(("0.0.0.0", 43900))  # Fixed local port
            self.sock.setblocking(False)
        except Exception as e:
            self.get_logger().error(f"Failed to bind socket: {e}\n{traceback.format_exc()}")

        # Robot's address
        self.qnx_addr = ("192.168.1.120", 43893)

        # --- Subscribers ---
        self.create_subscription(SimpleCMD, "simple_cmd", self.safe_wrapper(self.simple_cmd_callback), 10, callback_group=self.cmd_group)
        self.create_subscription(Int32, "motion_cmd", self.safe_wrapper(self.motion_callback), 10, callback_group=self.cmd_group)
        self.create_subscription(TwistStamped, "cmd_vel", self.safe_wrapper(self.cmd_vel_callback), 10, callback_group=self.cmd_group)

        # --- Publisher ---
        self.state_pub = self.create_publisher(String, "robot_state", 10)

        # --- Timers ---
        self.create_timer(0.1, self.safe_wrapper(self.recv_state), callback_group=self.state_group)   # recv robot state
        self.create_timer(0.5, self.safe_wrapper(self.timer_callback), callback_group=self.timer_group)  # optional periodic cmd
        
        self.state = 'sit'
        self.init()

        self.get_logger().info("----- ros2qnx node up (robust, multithreaded) -----")

    # -------- Error Wrapper -------- #
    def safe_wrapper(self, func):
        """Wraps any callback/timer to avoid crashing the node."""
        def wrapped(*args, **kwargs):
            try:
                return func(*args, **kwargs)
            except Exception as e:
                self.get_logger().error(f"Error in {func.__name__}: {e}\n{traceback.format_exc()}")
        return wrapped

    # -------- One-shot Timer Helper -------- #
    def schedule_once(self, delay, callback):
        timer = None
        def wrapper():
            try:
                callback()
            except Exception as e:
                self.get_logger().error(f"Error in scheduled callback: {e}\n{traceback.format_exc()}")
            finally:
                timer.cancel()
        timer = self.create_timer(delay, wrapper, callback_group=self.timer_group)
        return timer

    # ---------------- Periodic test ---------------- #
    def timer_callback(self):
        self.send_simple_cmd(0x21040001, 1, 0)

    # ---------------- Receive Robot State ---------------- #
    def recv_state(self):
        try:
            data, addr = self.sock.recvfrom(2048)
            if len(data) == 28:  # Only save if length == 28
                hex_str = data.hex()

                # Save raw hex
                # with open("received_data.txt", "a") as f:
                #     f.write(hex_str + "\n")

                # Check last 8th byte
                # - data is 28 bytes
                # - last 8th byte means: data[-8:-6] (2 bytes)
                last_8th = data[-8:-6].hex()
                state_msg = ""

                if last_8th == "0100":
                    state_msg = "sit"
                    self.state = "sit"
                if last_8th == "0600":
                    state_msg = "stand"
                    self.state = "stand"
                msg = String()
                if state_msg:
                    msg.data = state_msg
                else:
                    msg.data = state_msg

                self.state_pub.publish(msg)

        except BlockingIOError:
            pass  # no data available
        except Exception as e:
            self.get_logger().error(f"recv_state failed: {e}\n{traceback.format_exc()}")


    # ---------------- Motion Callbacks ---------------- #
    def motion_callback(self, msg: Int32):
        self.get_logger().info(f"Got motion cmd: {msg.data}")
        motions = {
            -1: self.stop,
            -2: self.auto,
            -3: self.manual,
            0: self.init,
            1: self.stand,
            2: self.sit,
            
            3: self.move_forward,
            4: self.move_backward,
            5: self.turn_left,
            6: self.turn_right,
            7: self.faster,
            8: self.slower,

            
            10: self.hand_heart,
            11: self.shake_body,
            12: self.space_step,
            13: self.turning_jump,
            14: self.dance1,
            15: self.twist_butt,
            16: self.wave_hand,
            17: self.jump_forward,
            18: self.clap_hand,
            19: self.dance2,
        }
        func = motions.get(msg.data)
        if func:
            func()
        else:
            self.get_logger().warn(f"Unknown motion command: {msg.data}")

    # ---------------- Motion Methods ---------------- #
    def stop(self):
        self.send_simple_cmd(0x21010c0b, 0, 0)

    def ai_off(self):
        self.send_simple_cmd(0x21012109, 0, 0)

    def arm_init(self):
        self.send_simple_cmd(0x21010C05, 0, 0)

    def init(self):
        self.arm_init()
        self.schedule_once(1.0, lambda: self.send_simple_cmd(0x21012109, 0, 0))
        self.schedule_once(1.3, lambda: self.send_simple_cmd(0x2101210d, 0, 0))

    def manual(self):
        self.send_simple_cmd(0x21010C02, 0, 0)

    def auto(self):
        self.send_simple_cmd(0x21010C03, 0, 0)

    def stand(self):
        self.send_simple_cmd(0x21010202, 0, 0)
        # self.schedule_once(3.0, self.stop)

    def sit(self):
        self.send_simple_cmd(0x21010202, 0, 0)
        # self.schedule_once(3.0, self.stop)

    ##################################################################################################
    # preset motions
    ##################################################################################################
    def hand_heart(self):
        self.send_simple_cmd(0x21010508, 0, 0)

    def shake_body(self):
        self.send_simple_cmd(0x21010204, 0, 0)

    def space_step(self):
        self.send_simple_cmd(0x2101030C, 0, 0)

    def turning_jump(self):
        self.send_simple_cmd(0x2101020D, 0, 0)

    def dance1(self):
        self.send_simple_cmd(0x21010521, 0, 0)



    def twist_butt(self):
        self.send_simple_cmd(0x21010509, 0, 0)

    def wave_hand(self):
        self.send_simple_cmd(0x21010506, 0, 0)

    def jump_forward(self):
        self.send_simple_cmd(0x2101050B, 0, 0)
    
    def clap_hand(self):
        self.send_simple_cmd(0x21010507, 0, 0)

    def dance2(self):
        self.send_simple_cmd(0x21010522, 0, 0)








    # ---------------- Complex Command Callbacks ---------------- #
    def cmd_vel_callback(self, msg: TwistStamped):
        cmd = msg.twist
        self.send_complex_cmd(320, 8, 1, cmd.linear.x)
        self.send_complex_cmd(325, 8, 1, cmd.linear.y)
        self.send_complex_cmd(321, 8, 1, -cmd.angular.z)

    def simple_cmd_callback(self, msg: SimpleCMD):
        self.send_simple_cmd(msg.cmd_code, msg.cmd_value, msg.type)

    def complex_cmd_callback(self, msg: ComplexCMD):
        self.send_complex_cmd(msg.cmd_code, msg.cmd_value, msg.type, msg.data)

    # ---------------- UDP Send Helpers ---------------- #
    def send_simple_cmd(self, cmd_code: int, cmd_value: int, cmd_type: int):
        try:
            packet = struct.pack("<iii", cmd_code, cmd_value, cmd_type)
            self.sock.sendto(packet, self.qnx_addr)
        except Exception as e:
            self.get_logger().error(f"send_simple_cmd failed: {e}\n{traceback.format_exc()}")

    def send_complex_cmd(self, cmd_code: int, cmd_value: int, cmd_type: int, data: float):
        try:
            packet = struct.pack("<iiid", cmd_code, cmd_value, cmd_type, data)
            self.sock.sendto(packet, self.qnx_addr)
        except Exception as e:
            self.get_logger().error(f"send_complex_cmd failed: {e}\n{traceback.format_exc()}")


def main(args=None):
    rclpy.init(args=args)
    node = ROS2QNX()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except Exception as e:
        node.get_logger().error(f"Fatal error in main loop: {e}\n{traceback.format_exc()}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
