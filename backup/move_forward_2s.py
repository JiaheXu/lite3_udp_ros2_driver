#!/usr/bin/env python3
import socket
import struct
import time

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32




sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
qnx_addr = ("192.168.1.120", 43893)



# simple cmds

cmd_code = 0x21012109
cmd_value = 0
cmd_type = 0
packet = struct.pack("<iii", cmd_code, cmd_value, cmd_type)
sock.sendto(packet, qnx_addr)
time.sleep(1.0)

cmd_code = 0x2101210d
cmd_value = 0
cmd_type = 0
packet = struct.pack("<iii", cmd_code, cmd_value, cmd_type)
sock.sendto(packet, qnx_addr)
time.sleep(1.0)

cmd_code = 0x21010202
cmd_value = 0
cmd_type = 0
packet = struct.pack("<iii", cmd_code, cmd_value, cmd_type)
sock.sendto(packet, qnx_addr)
time.sleep(3.0)


# cmd_code = 0x21010c03 # auto mode
# cmd_value = 0
# cmd_type = 0
# packet = struct.pack("<iii", cmd_code, cmd_value, cmd_type)
# sock.sendto(packet, qnx_addr)



# # 0x21010D06
# # complex cmds

# time.sleep(0.1)
# cmd_code = 320
# cmd_value = 8
# cmd_type = 1
# data = 0.5
# for i in range(40):
#     packet = struct.pack("<iiid", cmd_code, cmd_value, cmd_type, data)
#     sock.sendto(packet, qnx_addr)
#     time.sleep(0.1)

