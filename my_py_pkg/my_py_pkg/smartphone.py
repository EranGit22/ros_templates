#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String  

class SmartphoneClass(Node):
    def __init__(self):
        super().__init__("Smartphone") 
        self.subscriber_ = self.create_subscription(String, "robotNEWS",self.callback_robot_news,10) #the third argument is the callback, the second is the topic
        self.get_logger().info("Smartphone has been started")
       
    def callback_robot_news(self, msg):
        self.get_logger().info(msg.data) #msg.data is an argument from the 'String' package

def main(args=None):
    rclpy.init(args=args)
    node = SmartphoneClass()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__== "__main__":
    main()