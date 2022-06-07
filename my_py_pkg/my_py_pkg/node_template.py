#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class NodeClass2(Node):
    def __init__(self):
        super().__init__("Node2") 
       

def main(args=None):
    rclpy.init(args=args)
    node=NodeClass2()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__== "__main__":
    main()