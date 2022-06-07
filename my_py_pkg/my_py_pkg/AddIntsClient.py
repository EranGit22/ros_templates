#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from functools import partial


class AddingClientOOP(Node):
    def __init__(self):
        super().__init__("AddingClientOOP")
        self.call_add_ints_server(10, 11)

    def call_add_ints_server(self, a, b):
        # The second argument MUST have the exact name of the Server
        clientt = self.create_client(AddTwoInts, "AddTwoInts")
        while not clientt.wait_for_service(1.0):
            self.get_logger().warn("waiting for Server 'AddTwoInts' ")
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        F = clientt.call_async(request)
        # If we do that (without partial, only the 'F' will be sent. So we need another pyhton funcationality (package 'functools')
        F.add_done_callback(partial(self.callback, a=a, b=b))
        # 'add_done_callback'- add a callback to be run when Future (F) is done

    def callback(self, F, a, b):
        try:
            response = F.result()
            self.get_logger().info(str(a) + " + " +
                                   str(b) + " = " + str(response.sum))
        except Exception as e:
            self.get_logger().error("Service Call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = AddingClientOOP()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
