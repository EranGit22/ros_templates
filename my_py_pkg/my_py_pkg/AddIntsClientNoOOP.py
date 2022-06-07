#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


def main(args=None):
    rclpy.init(args=args)
    node = Node("AddIntsClientNoOOP")
    client = node.create_client(AddTwoInts, "AddTwoInts") #The second argument MUST have the exact name of the server
    while not client.wait_for_service(1.0):
        node.get_logger().warn("waiting for Server 'AddTwoInts' ")
    
    request = AddTwoInts.Request()
    request.a = 4
    request.b = 5

    F = client.call_async(request)
    rclpy.spin_until_future_complete(node, F)
    try:
        response = F.result()
        node.get_logger().info(str(request.a) + " + " +
                               str(request.b) + " = " + str(response.sum))
    except Exception as e:
        node.get_logger().error("Service Call failed %r" % (e,))

    rclpy.shutdown()


if __name__ == "__main__":
    main()
