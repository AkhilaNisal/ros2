#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64 

class NumberCounterNode(Node):

    def __init__(self):
        super().__init__("number_counter")
        self.count = 0

        self.number_count_publisher = self.create_publisher(Int64, "number_count", 10)
        self.number_subscriber = self.create_subscription(Int64, "number",self.callback, 10 )
        self.get_logger().info("Number counter node has been started")

    def callback(self, msg: Int64):
        self.count += msg.data
        new_msg = Int64()
        new_msg.data = self.count
        self.number_count_publisher.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()
