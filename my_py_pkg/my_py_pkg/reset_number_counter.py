#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
from functools import partial


class reset_client(Node):
    def __init__(self):
        super().__init__("ResetClient")
        self.call_ResetCounter(True)

    def call_ResetCounter(self,bool):
        # The second argument MUST have the exact name of the Server
        clientt = self.create_client(SetBool, "reset_counter")
        while not clientt.wait_for_service(1.0):
            self.get_logger().warn("waiting for Server 'ResetCounter' ")
        request = SetBool.Request()
        request = bool
        F = clientt.call_async(request)
        # If we do that (without partial, only the 'F' will be sent. So we need another pyhton funcationality (package 'functools')
        F.add_done_callback(partial(self.callback, bool=bool))
        # 'add_done_callback'- add a callback to be run when Future (F) is done

    def callback(self, F, bool):
        try:
            response = F.result()
            self.get_logger().info("Service Call succeeded. [recieved " + str(bool) +  "]")
        except Exception as e:
            self.get_logger().error("Service Call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = reset_client()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()