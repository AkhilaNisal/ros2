#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyNode(Node):

    def __init__(self):
        super().__init__("firstNode")
        self.counter = 0
        self.create_timer(1.0,self.time_callback)
        self.create_timer(0.2,self.time_callbacknew)


    def time_callback(self):
        self.get_logger().info("Hello" + str(self.counter))
        self.counter += 1

    def time_callbacknew(self):
        self.get_logger().info("________________-")

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()