#!/usr/bin/env python3
import socket
import struct
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64
from tf_transformations import quaternion_from_euler
import time

PI = 3.1415926

class QNX2ROS(Node):
    def __init__(self):
        super().__init__('qnx2ros')

        # Parameters
        self.declare_parameter('server_port', 43897)
        self.server_port = self.get_parameter('server_port').value

        # Socket setup
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', self.server_port))

        # Publishers
        self.leg_odom_pub = self.create_publisher(PoseWithCovarianceStamped, 'leg_odom', 10)
        self.leg_odom_pub2 = self.create_publisher(Odometry, 'leg_odom2', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.imu_pub_200 = self.create_publisher(Imu, '/imu/data', 10)
        self.handle_pub = self.create_publisher(Twist, '/handle_state', 10)
        self.ultrasound_pub = self.create_publisher(Float64, '/us_publisher/ultrasound_distance', 10)

        self.timer = self.create_timer(0.001, self.run_routine)
        self.receive_buffer = bytearray(512)

    def run_routine(self):
        try:
            data, _ = self.sock.recvfrom(512)
            self.parse_frame(data)
        except Exception as e:
            self.get_logger().error(f"Socket error: {e}")

    def parse_frame(self, data: bytes):
        data_len = len(data)

        if data_len == 380:  # RobotStateReceived
            self.parse_robot_state(data)
        elif data_len == 108:  # JointStateReceived
            self.parse_joint_state(data)
        elif data_len == 52:   # HandleStateReceived
            self.parse_IMU_state(data)
        elif data_len == 16:   # ImuDataReceived
            self.parse_imu_data(data)

    def parse_robot_state(self, data):
        header = struct.unpack('<3i', data[:12])
        payload = struct.unpack('<15d3Iidii2d', data[12:])
        if header[0] != 2305:
            return

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'odom'
        pose_msg.pose.pose.position.x = payload[6]
        pose_msg.pose.pose.position.y = payload[7]
        pose_msg.pose.pose.position.z = payload[8]
        quat = quaternion_from_euler(payload[0] / 180 * PI, payload[1] / 180 * PI, payload[2] / 180 * PI)
        pose_msg.pose.pose.orientation.x = quat[0]
        pose_msg.pose.pose.orientation.y = quat[1]
        pose_msg.pose.pose.orientation.z = quat[2]
        pose_msg.pose.pose.orientation.w = quat[3]
        self.leg_odom_pub.publish(pose_msg)

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.pose = pose_msg.pose
        odom_msg.twist.twist.linear.x = payload[9]
        odom_msg.twist.twist.linear.y = payload[10]
        odom_msg.twist.twist.angular.z = payload[5]
        self.leg_odom_pub2.publish(odom_msg)

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu'
        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]
        imu_msg.angular_velocity.x = payload[3]
        imu_msg.angular_velocity.y = payload[4]
        imu_msg.angular_velocity.z = payload[5]
        imu_msg.linear_acceleration.x = payload[12]
        imu_msg.linear_acceleration.y = payload[13]
        imu_msg.linear_acceleration.z = payload[14]
        self.imu_pub_200.publish(imu_msg)

        us_msg = Float64()
        us_msg.data = payload[16]
        self.ultrasound_pub.publish(us_msg)

    def parse_joint_state(self, data):
        header = struct.unpack('<3i', data[:12])
        joints = struct.unpack('<12d', data[12:])
        if header[0] != 2306:
            return

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_names = [
            "LF_Joint", "LF_Joint_1", "LF_Joint_2",
            "RF_Joint", "RF_Joint_1", "RF_Joint_2",
            "LB_Joint", "LB_Joint_1", "LB_Joint_2",
            "RB_Joint", "RB_Joint_1", "RB_Joint_2"
        ]
        joint_msg.name = joint_names
        joint_msg.position = [-x for x in joints]
        self.joint_state_pub.publish(joint_msg)

    def parse_handle_state(self, data):
        header = struct.unpack('<3i', data[:12])
        handle = struct.unpack('<6d', data[12:])
        if header[0] != 2309:
            return
        msg = Twist()
        msg.linear.x = handle[0]
        msg.linear.y = handle[1]
        msg.angular.z = -handle[2]
        self.handle_pub.publish(msg)

    def parse_imu_data(self, data):
        header = struct.unpack('<3i', data[:12])
        values = struct.unpack('<9f', data[12:])
        if header[0] != 0x010901:
            return

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'base_link'
        quat = quaternion_from_euler(values[0] / 180 * PI, values[1] / 180 * PI, values[2] / 180 * PI)
        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]
        imu_msg.angular_velocity.x = values[3]
        imu_msg.angular_velocity.y = values[4]
        imu_msg.angular_velocity.z = values[5]
        imu_msg.linear_acceleration.x = values[6]
        imu_msg.linear_acceleration.y = values[7]
        imu_msg.linear_acceleration.z = values[8]
        self.imu_pub_200.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = QNX2ROS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()