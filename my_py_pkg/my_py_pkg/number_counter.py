#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool


class NumberCounterClass(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.server_ = self.create_service(
            SetBool, "reset_counter", self.callback_reset)
        self.get_logger().info("The Service 'Reset' has been started")

        self.publisher_ = self.create_publisher(
            Int64, "number_counter", 5)  # can save up to 10 messages
        self.counter = 0
        # the third argument is the callback, the second is the topic
        self.subscriber_ = self.create_subscription(
            Int64, "number", self.callback_published_number, 10)
        self.get_logger().info("number counter has been started")

    def callback_published_number(self, msg):
        self.counter += msg.data
        new_msg = Int64()
        new_msg.data = self.counter
        self.publisher_.publish(new_msg)
        # msg.data is an argument from the 'String' package
        self.get_logger().info(str(self.counter))

    def callback_reset(self, request, response):
        
        if request.data:
            self.counter = 0
            response.success = True
            response.message= 'Request True recieved, Counter has been reset.'
        else:
            response.success = False
            response.message = 'Request False recieved, nothing has changed.'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterClass()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
