#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import mraa
import time


class gpio_test(Node):
    def __init__(self):
        super().__init__("Node2")
        # initialize gpio
        gpio_1=mraa.Gpio(12)
        #set up GPIO channel to output (pin 12)
        gpio_1.dir(mraa.DIR_OUT)
        # output ti pin 12 (GPIO18)
        for i in range(700):
            if i%2==0:
                gpio_1.write(1)
            else:
                gpio_1.write(0)
            time.sleep(1)
        

       

def main(args=None):
    rclpy.init(args=args)
    node=gpio_test()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__== "__main__":
    main()