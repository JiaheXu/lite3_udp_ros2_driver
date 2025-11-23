#!/usr/bin/env python3
import socket
import struct
import rclpy
import traceback
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int32, String, Float32
from sensor_msgs.msg import Joy
from pino_msgs.msg import SimpleCMD, ComplexCMD, AudioMSG


class MotionServer(Node):
    def __init__(self):
        super().__init__('motion_server')

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
        self.desired_speed = 0.6
        self.pending_timers = []
        self.initialized = False
        self.pending_motion_cmd = None
        self.pending_motion_after_stand = None
        self.active_motion_timer = None

        # Track cmd_vel activity
        self.cmd_vel_active = False
        self.last_cmd_vel_time = 0.0

        # --- Publishers ---
        self.state_pub = self.create_publisher(String, "robot_state", 10)
        self.audio_pub = self.create_publisher(AudioMSG, "audio_cmd", 10)
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

        # --- Subscribers ---
        self.simple_cmd_sub = self.create_subscription(
            SimpleCMD,
            "simple_cmd",
            self.safe_wrapper(self.simple_cmd_callback),
            10,
            callback_group=self.cmd_group,
        )
        self.motion_cmd_sub = self.create_subscription(
            Int32,
            "motion_cmd",
            self.safe_wrapper(self.motion_callback),
            10,
            callback_group=self.cmd_group,
        )
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped,
            "cmd_vel",
            self.safe_wrapper(self.cmd_vel_callback),
            10,
            callback_group=self.cmd_group,
        )
        self.behavior_mode_sub = self.create_subscription(
            Int32,
            "behavior_mode",
            self.safe_wrapper(self.behavior_callback),
            10,
            callback_group=self.cmd_group,
        )

        # ---- NEW: Local joystick subscriber ----
        self.joystick_sub = self.create_subscription(
            Joy,
            "local_joystick",
            self.safe_wrapper(self.local_joystick_callback),
            10,
            callback_group=self.cmd_group,
        )

        # --- Timers ---
        self.create_timer(0.1, self.safe_wrapper(self.recv_state), callback_group=self.state_group)
        self.create_timer(0.5, self.safe_wrapper(self.timer_callback), callback_group=self.timer_group)

        # --- Init sequence ---
        self.init()

        self.get_logger().info("----- motion_server node up -----")

    # -------------------------------------------------------------
    # Safe wrapper
    # -------------------------------------------------------------
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

    def publish_audio(self, text: str, cmd: str = 'speak', voice: str = 'zf_xiaoyi', volume: float = 1.2, speed: float = 0.8):
        msg = AudioMSG()
        msg.cmd = cmd
        msg.text = text
        msg.voice = voice
        msg.volume = volume
        msg.speed = speed
        self.audio_pub.publish(msg)

    # -------------------------------------------------------------
    # Timers
    # -------------------------------------------------------------
    def timer_callback(self):
        self.send_simple_cmd(0x21040001, 1, 0)
        msg = Float32()
        msg.data = self.desired_speed
        self.speed_pub.publish(msg)

    # -------------------------------------------------------------
    # State receiver
    # -------------------------------------------------------------
    def recv_state(self):
        try:
            data, addr = self.sock.recvfrom(2048)
            if len(data) == 28:
                data_hex = str(data.hex())
                state = data_hex[-32:-28]
                state_msg = None

                if state == "0100":
                    self.state = "sit"
                    state_msg = "sit"

                if state == "1000":
                    self.state = "stand"
                    state_msg = "stand"

                    if self.pending_motion_after_stand:
                        cmd = self.pending_motion_after_stand
                        self.pending_motion_after_stand = None
                        self.get_logger().info("Executing deferred motion after stand")
                        self._start_timed_motion(*cmd)

                if state_msg:
                    msg = String()
                    msg.data = state_msg
                    self.state_pub.publish(msg)

        except BlockingIOError:
            pass
        except Exception as e:
            self.get_logger().error(f"recv_state failed: {e}\n{traceback.format_exc()}")

    # -------------------------------------------------------------
    # Motion command callbacks
    # -------------------------------------------------------------
    def motion_callback(self, msg: Int32):
        cmd = msg.data
        self.get_logger().info(f"[motion_cmd] received: {cmd}")
        if not self.initialized:
            self.get_logger().info("Initialization in progress, deferring motion command")
            self.pending_motion_cmd = cmd
            return
        self._execute_motion_command(cmd)

    def _execute_motion_command(self, cmd: int):
        func = self.motions.get(cmd)
        if func:
            func()
        else:
            self.get_logger().warn(f"Unknown motion command: {cmd}")

    # -------------------------------------------------------------
    # Basic motion helpers
    # -------------------------------------------------------------
    def stop(self):
        for t in list(self.pending_timers):
            t.cancel()
        self.pending_timers.clear()
        if self.active_motion_timer:
            self.active_motion_timer.cancel()
            self.active_motion_timer = None
        self.pending_motion_after_stand = None
        self.pending_motion_cmd = None
        self.send_simple_cmd(0x21010c0b, 0, 0)

    def ai_off(self):
        self.send_simple_cmd(0x21012109, 0, 0)

    def arm_init(self):
        self.send_simple_cmd(0x21010C05, 0, 0)

    def faster(self):
        self.desired_speed = min(self.desired_speed + 0.2, 1.2)

    def slower(self):
        self.desired_speed = max(self.desired_speed - 0.2, 0.2)

    def navigate_prepare(self):
        self.auto()

    def init(self):
        self.get_logger().info("Starting initialization sequence")
        self.arm_init()
        self.send_simple_cmd(0x21012109, 0, 0)
        self.schedule_once(0.1, self._init_step_enable)

    def _init_step_enable(self):
        self.send_simple_cmd(0x2101210d, 0, 0)
        self.schedule_once(8.0, self._init_complete)

    def _init_complete(self):
        self.initialized = True
        self.get_logger().info("Initialization complete")
        if self.pending_motion_cmd is not None:
            cmd = self.pending_motion_cmd
            self.pending_motion_cmd = None
            self._execute_motion_command(cmd)

    def manual(self):
        self.send_simple_cmd(0x21010C02, 0, 0)
        self.mode = 'manual'

    def auto(self):
        self.send_simple_cmd(0x21010C03, 0, 0)
        self.mode = 'auto'

    def stand(self):
        self.get_logger().info("Requesting STAND (same cmd as SIT)")
        if self.state == 'sit':
            self.send_simple_cmd(0x21010202, 0, 0)

    def sit(self):
        self.get_logger().info("Requesting SIT (same cmd as STAND)")
        if self.state == 'stand':
            self.send_simple_cmd(0x21010202, 0, 0)

    # --- basic motions ---
    def move_forward(self): self._send_motion_for_duration(320, 0.5, 1.0)
    def move_backward(self): self._send_motion_for_duration(320, -0.5, 1.0)
    def turn_left(self): self._send_motion_for_duration(321, -0.6, 2.0)
    def turn_right(self): self._send_motion_for_duration(321, 0.6, 2.0)

    def _send_motion_for_duration(self, cmd_code: int, value: float, duration: float):
        if self.state == "sit":
            self.get_logger().info("Robot is sitting, requesting stand before executing motion")
            self.pending_motion_after_stand = (cmd_code, value, duration)
            self.stand()
            return
        self._start_timed_motion(cmd_code, value, duration)

    def _start_timed_motion(self, cmd_code: int, value: float, duration: float):
        if self.active_motion_timer:
            self.active_motion_timer.cancel()
            self.active_motion_timer = None

        self.auto()
        hz = 10
        ticks_total = max(1, int(duration * hz))
        self._motion_ticks = 1
        self.send_complex_cmd(cmd_code, 8, 1, value)

        timer = None
        def step():
            nonlocal timer
            if self._motion_ticks >= ticks_total:
                self.send_complex_cmd(cmd_code, 8, 1, 0.0)
                timer.cancel()
                self.active_motion_timer = None
                return
            self.send_complex_cmd(cmd_code, 8, 1, value)
            self._motion_ticks += 1

        timer = self.create_timer(1.0 / hz, step, callback_group=self.timer_group)
        self.active_motion_timer = timer

    # -------------------------------------------------------------
    # Preset motions
    # -------------------------------------------------------------
    def hand_heart(self): self.send_simple_cmd(0x21010508, 0, 0)
    def shake_body(self): self.send_simple_cmd(0x21010204, 0, 0); self.schedule_once(10.0, self.stop)
    def space_step(self): self.send_simple_cmd(0x2101030C, 0, 0); self.schedule_once(5.0, self.stop)
    def turning_jump(self): self.send_simple_cmd(0x2101020D, 0, 0)
    def dance1(self): self.send_simple_cmd(0x21010521, 0, 0); self.schedule_once(10.0, self.stop)
    def twist_butt(self): self.send_simple_cmd(0x21010509, 0, 0); self.schedule_once(5.0, self.stop)
    def wave_hand(self): self.send_simple_cmd(0x21010506, 0, 0)
    def jump_forward(self): self.send_simple_cmd(0x2101050B, 0, 0)
    def clap_hand(self): self.send_simple_cmd(0x21010507, 0, 0)
    def dance2(self): self.send_simple_cmd(0x21010522, 0, 0); self.schedule_once(5.0, self.stop)

    # -------------------------------------------------------------
    # cmd_vel callback
    # -------------------------------------------------------------
    def cmd_vel_callback(self, msg: TwistStamped):
        # Mark active time
        self.cmd_vel_active = True
        self.last_cmd_vel_time = self.get_clock().now().nanoseconds * 1e-9

        if self.mode == "manual":
            self.auto()
            return
        if self.behavior_mode < 2:
            return

        cmd = msg.twist

        self.send_complex_cmd(320, 8, 1, cmd.linear.x)
        self.send_complex_cmd(325, 8, 1, cmd.linear.y)
        self.send_complex_cmd(321, 8, 1, -cmd.angular.z)

    # -------------------------------------------------------------
    # NEW: Joystick control logic
    # -------------------------------------------------------------
    def local_joystick_callback(self, msg: Joy):
        """
        Human joystick override logic:
        - If cmd_vel is active (within 0.3s): ignore forward/back from joystick but override yaw.
        - If no cmd_vel: joystick has full control (forward + turn).
        """

        if len(msg.axes) < 2:
            return

        sx = msg.axes[0]    # forward/back
        sy = msg.axes[1]    # turn left/right

        now = self.get_clock().now().nanoseconds * 1e-9
        cmd_vel_recent = (now - self.last_cmd_vel_time) < 0.3

        # Deadzone
        TH = 0.15

        forward = sx * self.desired_speed if abs(sx) > TH else 0.0
        turning = sy * 1.2 if abs(sy) > TH else 0.0

        # ------------------------------------------------
        # Case 1: cmd_vel active → steering override only
        # ------------------------------------------------
        if cmd_vel_recent:
            if abs(turning) > 0.01:
                self.send_complex_cmd(321, 8, 1, turning)
            else:
                self.send_complex_cmd(321, 8, 1, 0.0)
            return

        # ------------------------------------------------
        # Case 2: No cmd_vel → full manual joystick
        # ------------------------------------------------
        if abs(forward) < 0.01 and abs(turning) < 0.01:
            self.send_complex_cmd(320, 8, 1, 0.0)
            self.send_complex_cmd(321, 8, 1, 0.0)
            return

        self.send_complex_cmd(320, 8, 1, forward)
        self.send_complex_cmd(321, 8, 1, turning)

    # -------------------------------------------------------------
    # Remaining callbacks
    # -------------------------------------------------------------
    def behavior_callback(self, msg: Int32):
        self.behavior_mode = msg.data

    def simple_cmd_callback(self, msg: SimpleCMD):
        self.send_simple_cmd(msg.cmd_code, msg.cmd_value, msg.type)

    def complex_cmd_callback(self, msg: ComplexCMD):
        self.send_complex_cmd(msg.cmd_code, msg.cmd_value, msg.type, msg.data)

    # -------------------------------------------------------------
    # UDP send helpers
    # -------------------------------------------------------------
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
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

