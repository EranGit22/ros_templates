#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64



class NumberPublisherClass(Node):
    def __init__(self):
        super().__init__("number_publisher") 

        
        self.number=100
        self.publisher_ = self.create_publisher(Int64, "number",10) #can save up to 10 messages
        self.timer_ = self.create_timer(10, self.publish_numberr)
        self.get_logger().info("Number Publisher has been started")
    
    def publish_numberr(self):
        msg = Int64()
        msg.data = self.number
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node=NumberPublisherClass()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__== "__main__":
    main()