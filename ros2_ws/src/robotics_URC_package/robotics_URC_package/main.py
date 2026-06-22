#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor
from robotics_URC_package.trees.main_tree import TreeNode
from robotics_URC_package.blackboard.blackboard_publisher import BlackboardPublisher

def main(args=None):
    rclpy.init(args=args)

    tree_node = TreeNode()
    bb_publisher = BlackboardPublisher()

    executor = MultiThreadedExecutor()
    executor.add_node(tree_node)
    executor.add_node(bb_publisher)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        tree_node.destroy_node()
        bb_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()