#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class NodeClass(Node):
    def __init__(self):
        super().__init__("Node1") 
        self.counter_ = 0
        self.get_logger().info("hi class")
        self.create_timer(0.5,self.timer_callback)

    def timer_callback(self):
        self.counter_ += 1
        self.get_logger().info("timercallback!" + str(self.counter_))

def main(args=None):
    rclpy.init(args=args)
    node=NodeClass()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__== "__main__":
    main()