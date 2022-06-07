#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String




class NewsStationClass(Node):
    def __init__(self):
        super().__init__("NewsStationNode") 

        self.robot_name_ = "robi"
        self.publisher_ = self.create_publisher(String, "robotNEWS",10) #can save up to 10 messages
        self.timer_ = self.create_timer(0.5, self.publish_news)
        self.get_logger().info("Robot News Station ha been started")
    
    def publish_news(self):
        msg = String()
        msg.data = "Hi, this is " + str(self.robot_name_) + "from israel" 
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node=NewsStationClass()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__== "__main__":
    main()