#!/usr/bin/env python3
import rclpy
import smbus
from smbus2 import SMBus
import time
bus = SMBus(1)
time.sleep(1) #wait here to avoid 121 IO Error
while True:
    data = bus.write_i2c_block_data(0x04, 0,[1,2,3,4,5])
    result = 0
    for b in data:
        result = result * 256 + int(b)
    print(result)
    time.sleep(1)