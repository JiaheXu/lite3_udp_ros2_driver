#!/usr/bin/env python
"""
Inputs a Joy messages and produces controls
that match the control_interface
"""
import rclpy
from rclpy.node import Node

import time
import math
from geometry_msgs.msg import PointStamped, TwistStamped
from std_msgs.msg import UInt8, Bool, String
from sensor_msgs.msg import Joy

from std_msgs.msg import String, Float32, Int8, UInt8, Bool, UInt32MultiArray, Int32


class JoystickCommandSource(Node):
    def __init__(self):

        super().__init__('joystick_command_node')
        # self.last_log_time = self.get_clock().now()

        # Axes for control
        # self.get_parameter('my_parameter').get_parameter_value().string_value

        # self.declare_parameter('my_parameter', 'world')

        self.left_joystick_x = 0
        self.left_joystick_y = 1
        self.left_trigger = 2
        self.right_joystick_x = 3
        self.right_joystick_y = 4
        self.right_trigger = 5
        self.dpad_left_right = 6
        self.dpad_up_down = 7

        self.max_idx = max(
            self.left_trigger,
            self.left_joystick_x,
            self.left_joystick_y,
            self.dpad_up_down,
            self.right_joystick_x,
            self.right_joystick_y,
            self.right_trigger,
        )

        # Buttons for control
        self.a_button = 0
        self.b_button = 1
        self.x_button = 2
        self.y_button = 3

        self.lb_button = 4
        self.rb_button = 5

        self.back_button = 6
        self.start_button = 7
        self.xbox_button = 8
        self.left_joystick_button = 9
        self.right_joystick_button = 10
        
        self.max_button = max(
            self.b_button,
            self.y_button,
            self.x_button,
            self.a_button,
            self.rb_button,
            self.back_button,
            self.start_button,
            self.left_joystick_button,
            self.right_joystick_button,
            self.lb_button
        )


        self.arm_button_pressed = False
        self.stop_button_pressed = False
        self.auto_button_pressed = False
        self.manual_button_pressed = False
        self.spacestep_button_pressed = False

        self.joystick_sub = self.create_subscription(Joy, "joy", self.joyCallback, 1)

        self.twist_pub = self.create_publisher(TwistStamped, "cmd_vel", 1)
        self.motion_pub = self.create_publisher(Int32, "motion_cmd", 1)

        self.get_logger().info('end of init')

    def joyCallback(self, msg):
        # print("in call back")
        joystick_x = msg.axes[self.left_joystick_x]
        joystick_y = msg.axes[self.right_joystick_y]



        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()

        if( abs(joystick_y) > 0.1 or abs(joystick_x) > 0.1):
            twist_msg.twist.linear.x = joystick_y
            twist_msg.twist.angular.z = joystick_x
            self.twist_pub.publish(twist_msg)

        lb_button = msg.buttons[self.lb_button]
        rb_button = msg.buttons[self.rb_button]
        # print("lb_button: ", lb_button)
        if lb_button:
            if not self.arm_button_pressed:
                print("sending  sit/stand")
                cmd_msg = Int32()
                cmd_msg.data = 1
                self.motion_pub.publish(cmd_msg)

            self.arm_button_pressed = True
        else:
            self.arm_button_pressed = False

        x_button = msg.buttons[self.x_button]
        if x_button:
            if not self.stop_button_pressed:
                cmd_msg = Int32()
                cmd_msg.data = -1
                self.motion_pub.publish(cmd_msg)

            self.stop_button_pressed = True
        else:
            self.stop_button_pressed = False

        a_button = msg.buttons[self.a_button]
        if a_button:
            if not self.auto_button_pressed:
                cmd_msg = Int32()
                cmd_msg.data = -2
                self.motion_pub.publish(cmd_msg)

            self.auto_button_pressed = True
        else:
            self.auto_button_pressed = False

        y_button = msg.buttons[self.y_button]
        if y_button:
            if not self.manual_button_pressed:
                cmd_msg = Int32()
                cmd_msg.data = -3
                self.motion_pub.publish(cmd_msg)

            self.manual_button_pressed = True
        else:
            self.manual_button_pressed = False

        b_button = msg.buttons[self.b_button]
        if b_button:
            if not self.spacestep_button_pressed:
                cmd_msg = Int32()
                cmd_msg.data = 6
                self.motion_pub.publish(cmd_msg)
            self.spacestep_button_pressed = True
        else:
            self.spacestep_button_pressed = False


def main(args=None):
    print("init!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    rclpy.init(args=args)
    node = JoystickCommandSource()
    # try:
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     node.destroy_node()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
