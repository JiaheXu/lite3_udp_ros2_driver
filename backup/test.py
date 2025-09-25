#!/usr/bin/env python3
import socket
import struct
import time

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32




sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
qnx_addr = ("192.168.1.120", 43893)

while True:
    cmd_code = 0x21040001
    cmd_value = 0
    cmd_type = 0
    packet = struct.pack("<iii", cmd_code, cmd_value, cmd_type)
    sock.sendto(packet, qnx_addr)
    time.sleep(0.1)
    print("sent hearbeat: ", time.time())

