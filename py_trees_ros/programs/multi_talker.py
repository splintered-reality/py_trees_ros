#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
##############################################################################
# Imports
##############################################################################

import rclpy

from std_msgs.msg import String

##############################################################################
# Main
##############################################################################


def main(args=None):
    i = 0
    timer_period = 0.5  # seconds

    rclpy.init(args=args)
    node = rclpy.create_node('multi_talker')

    publisher = node.create_publisher(String, 'topic')

    timeout_reached = False

    def timer_callback():
        nonlocal i
        nonlocal timeout_reached
        message = 'Hello World: %d' % i
        i += 1
        node.get_logger().info('"%s"' % message)
        timeout_reached = True

    timer = node.create_timer(timer_period, timer_callback)

    while not timeout_reached:
        rclpy.spin_once(node)

    node.destroy_timer(timer)
    node.destroy_publisher(publisher)
    timeout_reached = False
    print("Publisher destroyed")

    publisher2 = node.create_publisher(String, 'topic2')

    def timer2_callback():
        nonlocal i
        msg = String()
        msg.data = 'Hello Again: %d' % i
        i += 1
        node.get_logger().info('Publishing: "%s"' % msg.data)
        publisher2.publish(msg)

    timer2 = node.create_timer(timer_period, timer2_callback)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    print("Cancel timer")
    print("Destroy Timer")
    timer2.cancel()
    node.destroy_timer(timer2)
    node.destroy_node()
    rclpy.shutdown()
