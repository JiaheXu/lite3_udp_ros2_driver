#!/usr/bin/env python3
import socket
import struct
import rclpy
import traceback
import time
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int32, String, Float32
from pino_msgs.msg import SimpleCMD, ComplexCMD, AudioMSG


class MotionServer(Node):
    def __init__(self):
        super().__init__('motion_server')

        # --- QoS profiles ---
        # Audio should be received even if subscriber starts later => latch it
        self.audio_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # Commands & misc
        self.cmd_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Reliability setting
            durability=QoSDurabilityPolicy.VOLATILE  # Durability setting
        )

        # --- Callback groups ---
        self.cmd_group = ReentrantCallbackGroup()
        self.state_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()

        # --- UDP socket ---
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind(("0.0.0.0", 43900))
            self.sock.setblocking(False)
        except Exception as e:
            self.get_logger().error(f"Failed to bind socket: {e}\n{traceback.format_exc()}")

        # Robot addr
        self.qnx_addr = ("192.168.1.120", 43893)

        # --- Internal state ---
        self.state = "sit"
        self.mode = "manual"
        self.behavior_mode = 0
        self.desired_speed = 0.4
        self.pending_timers = []

        # --- Publishers ---
        self.state_pub = self.create_publisher(String, "robot_state", 10)
        self.audio_pub = self.create_publisher(AudioMSG, "audio_cmd", self.audio_qos)
        self.speed_pub = self.create_publisher(Float32, "desired_speed", 10)

        # --- Motion map ---
        self.motions = {
            -3: self.auto,
            -2: self.manual,
            -1: self.init,
            0: self.stop,
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
            20: self.navigate_prepare,
            30: self.navigate_prepare,
        }

        # --- Timers ---
        self.create_timer(0.1, self.safe_wrapper(self.recv_state), callback_group=self.state_group)
        self.create_timer(0.5, self.safe_wrapper(self.timer_callback), callback_group=self.timer_group)

        # --- Subscribers ---
        self.create_subscription(SimpleCMD, "simple_cmd", self.safe_wrapper(self.simple_cmd_callback), self.cmd_qos, callback_group=self.cmd_group)
        self.create_subscription(Int32, "motion_cmd", self.safe_wrapper(self.motion_callback), self.cmd_qos, callback_group=self.cmd_group)
        self.create_subscription(TwistStamped, "cmd_vel", self.safe_wrapper(self.cmd_vel_callback), 10, callback_group=self.cmd_group)
        self.create_subscription(Int32, "behavior_mode", self.safe_wrapper(self.behavior_callback), 10, callback_group=self.cmd_group)

        self.get_logger().info("----- motion_server node up -----")

        # --- Init sequence (do it after pubs/subs exist) ---
        # Give other nodes a moment to come up to avoid dropping early messages
        self.schedule_once(0.5, self.init)

    # -------- Utilities -------- #
    def safe_wrapper(self, func):
        def wrapped(*args, **kwargs):
            try:
                return func(*args, **kwargs)
            except Exception as e:
                self.get_logger().error(f"Error in {func.__name__}: {e}\n{traceback.format_exc()}")
        return wrapped

    def schedule_once(self, delay, callback):
        timer = None
        def wrapper():
            try:
                callback()
            finally:
                if timer in self.pending_timers:
                    self.pending_timers.remove(timer)
                timer.cancel()
        timer = self.create_timer(delay, wrapper, callback_group=self.timer_group)
        self.pending_timers.append(timer)
        return timer

    def publish_audio(self, text: str, cmd: str = 'speak', voice: str = 'zf_xiaoyi', volume: float = 1.2, speed: float = 0.8, wait_ms: int = 800):
        """Latched audio publisher + optional wait for subscriber to avoid drops."""
        # Optional: wait briefly for at least one subscriber
        t0 = time.time()
        while self.audio_pub.get_subscription_count() == 0 and (time.time() - t0) * 1000 < wait_ms:
            rclpy.spin_once(self, timeout_sec=0.05)

        msg = AudioMSG()
        msg.cmd = cmd
        msg.text = text
        msg.voice = voice
        msg.volume = volume
        msg.speed = speed
        self.audio_pub.publish(msg)
        self.get_logger().info(f"[audio_cmd] -> {cmd}:{voice} '{text}' (subs={self.audio_pub.get_subscription_count()})")

    # -------- Timers -------- #
    def timer_callback(self):
        # keepalive / watchdog
        self.send_simple_cmd(0x21040001, 1, 0)
        msg = Float32()
        msg.data = self.desired_speed
        self.speed_pub.publish(msg)

    # -------- State recv -------- #
    def recv_state(self):
        try:
            data, addr = self.sock.recvfrom(2048)
            if len(data) == 28:
                data_hex = str(data.hex())
                state = data_hex[-32:-28]
                state_msg = None
                if state == "0100":
                    state_msg = "sit"
                    self.state = "sit"
                if state == "1000":
                    state_msg = "stand"
                    self.state = "stand"
                if state_msg:
                    msg = String()
                    msg.data = state_msg
                    self.state_pub.publish(msg)
                    self.get_logger().debug(f"robot state: {self.state}")
        except BlockingIOError:
            pass
        except Exception as e:
            self.get_logger().error(f"recv_state failed: {e}\n{traceback.format_exc()}")

    # -------- Motion callbacks -------- #
    def motion_callback(self, msg: Int32):
        self.get_logger().info(f"[motion_cmd] received: {msg.data}")
        func = self.motions.get(msg.data)
        if func:
            func()
        else:
            self.get_logger().warn(f"Unknown motion command: {msg.data}")

    # -------- Motion methods -------- #
    def stop(self):
        for t in list(self.pending_timers):
            try:
                t.cancel()
            except Exception:
                pass
        self.pending_timers.clear()
        self.send_simple_cmd(0x21010c0b, 0, 0)

    def ai_off(self):
        self.send_simple_cmd(0x21012109, 0, 0)

    def arm_init(self):
        self.send_simple_cmd(0x21010C05, 0, 0)

    def faster(self):
        self.desired_speed = min(self.desired_speed + 0.2, 0.8)

    def slower(self):
        self.desired_speed = max(self.desired_speed - 0.2, 0.2)

    def navigate_prepare(self):
        self.auto()

    def init(self):
        # Arm and enable
        self.arm_init()
        self.send_simple_cmd(0x21012109, 0, 0)
        time.sleep(0.1)
        self.publish_audio("启动初始化，请稍候。")

        self.send_simple_cmd(0x2101210d, 0, 0)
        time.sleep(2.0)  # shorter sleep; rely on state feedback loop below

        # Request stand (toggle cmd shared with sit)
        self.stand()

        # Wait until robot actually reports stand
        max_wait = 20.0
        dt = 0.1
        waited = 0.0
        while self.state != "stand" and waited < max_wait:
            rclpy.spin_once(self, timeout_sec=dt)
            waited += dt
        self.get_logger().info(f"init state: {self.state}")
        if self.state == "stand":
            self.get_logger().info("✅ Robot confirmed standing")
            self.publish_audio("你好主人，我是今天为您服务的导览机器人，很高兴为您服务！")
            # self.move_forward()
        else:
            self.get_logger().warn("⚠️ Stand not confirmed within timeout.")
            self.publish_audio("初始化未完成，请检查机器人状态。")

    def manual(self):
        self.send_simple_cmd(0x21010C02, 0, 0)
        self.mode = 'manual'

    def auto(self):
        self.send_simple_cmd(0x21010C03, 0, 0)
        self.mode = 'auto'

    def stand(self):
        self.get_logger().info("Requesting STAND (toggle cmd)")
        self.send_simple_cmd(0x21010202, 0, 0)

    def sit(self):
        self.get_logger().info("Requesting SIT (toggle cmd)")
        self.send_simple_cmd(0x21010202, 0, 0)

    # --- basic motions ---
    def move_forward(self):  self._send_motion_for_duration(320,  0.5, 1.0)
    def move_backward(self): self._send_motion_for_duration(320, -0.5, 1.0)
    def turn_left(self):     self._send_motion_for_duration(321, -0.6, 2.0)
    def turn_right(self):    self._send_motion_for_duration(321,  0.6, 2.0)

    def _send_motion_for_duration(self, cmd_code: int, value: float, duration: float):
        if self.state == "sit":
            self.get_logger().warn("Ignoring motion: robot is sitting")
            return
        hz = 10
        ticks_total = int(duration * hz)
        self._motion_ticks = 0
        self.auto()

        def step():
            try:
                self.send_complex_cmd(cmd_code, 8, 1, value)
                self._motion_ticks += 1
                if self._motion_ticks >= ticks_total:
                    try:
                        self.send_complex_cmd(cmd_code, 8, 1, 0.0)
                    finally:
                        if timer in self.pending_timers:
                            self.pending_timers.remove(timer)
                        timer.cancel()
            except Exception as e:
                self.get_logger().error(f"_send_motion_for_duration step err: {e}")

        timer = self.create_timer(1.0 / hz, step, callback_group=self.timer_group)
        self.pending_timers.append(timer)

    # --- preset motions ---
    def hand_heart(self):     self.send_simple_cmd(0x21010508, 0, 0)
    def shake_body(self):     self.send_simple_cmd(0x21010204, 0, 0); self.schedule_once(10.0, self.stop)
    def space_step(self):     self.send_simple_cmd(0x2101030C, 0, 0); self.schedule_once(5.0, self.stop)
    def turning_jump(self):   self.send_simple_cmd(0x2101020D, 0, 0)
    def dance1(self):         self.send_simple_cmd(0x21010521, 0, 0); self.schedule_once(10.0, self.stop)
    def twist_butt(self):     self.send_simple_cmd(0x21010509, 0, 0); self.schedule_once(5.0, self.stop)
    def wave_hand(self):      self.send_simple_cmd(0x21010506, 0, 0)
    def jump_forward(self):   self.send_simple_cmd(0x2101050B, 0, 0)
    def clap_hand(self):      self.send_simple_cmd(0x21010507, 0, 0)
    def dance2(self):         self.send_simple_cmd(0x21010522, 0, 0); self.schedule_once(5.0, self.stop)

    # -------- Complex cmd / velocity -------- #
    def cmd_vel_callback(self, msg: TwistStamped):
        if self.mode == "manual":
            self.auto()  # flip to auto if manual
            return
        if self.behavior_mode < 2:
            return
        cmd = msg.twist
        self.send_complex_cmd(320, 8, 1, cmd.linear.x)
        self.send_complex_cmd(325, 8, 1, cmd.linear.y)
        self.send_complex_cmd(321, 8, 1, -cmd.angular.z)

    def behavior_callback(self, msg: Int32):
        self.behavior_mode = msg.data

    def simple_cmd_callback(self, msg: SimpleCMD):
        self.send_simple_cmd(msg.cmd_code, msg.cmd_value, msg.type)

    def complex_cmd_callback(self, msg: ComplexCMD):
        self.send_complex_cmd(msg.cmd_code, msg.cmd_value, msg.type, msg.data)

    # -------- UDP helpers -------- #
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
    node = MotionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
