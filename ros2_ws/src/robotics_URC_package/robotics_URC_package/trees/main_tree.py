#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from robotics_URC_package.behaviors import actions
from robotics_URC_package.behaviors import conditions
import py_trees
import rclpy
from rclpy.node import Node
import py_trees_ros
import py_trees.blackboard as blackboard
from robotics_URC_package.blackboard.config import BaseBlackboardKeys

from std_msgs.msg import String
import json

def create_tree():
    root = py_trees.composites.Selector(name="Root",memory=False)
    
    bootNode = actions.Boot(name="Boot")
    bootCon = conditions.isBooted(name="isNotBooted", inverted = True)
    bootSeq = py_trees.composites.Sequence(name="Boot Sequence", memory=False)
    bootSeq.add_children([bootCon, bootNode])

    telecomNode = actions.Teleoperation(name="Teleoperation")
    telecomCon = conditions.isTeleoperation(name="isTeleoperation")
    telecomSeq = py_trees.composites.Sequence(name="Teleoperation Sequence", memory=False)
    telecomSeq.add_children([telecomCon, telecomNode])

    safetyNode = actions.MakeSafe(name="Make Safe")
    safetyCon = conditions.isSafe(name="isNotSafe", inverted = True)
    safetySeq = py_trees.composites.Sequence(name="Safety Sequence", memory=False)
    safetySeq.add_children([safetyCon, safetyNode])

    armNode = actions.Arm(name="Arm")
    armCon = conditions.isArm(name="isArm")
    armSeq = py_trees.composites.Sequence(name="Arm Sequence", memory=False)
    armSeq.add_children([armCon, armNode])

    driveNode = actions.Drive(name="Drive")
    driveCon = conditions.isDrive(name="isDrive")
    driveSeq = py_trees.composites.Sequence(name="Drive Sequence", memory=False)
    driveSeq.add_children([driveCon, driveNode])

    driveNode = actions.Drive(name="Drive")
    driveCon = conditions.isDrive(name="isDrive")
    driveSeq = py_trees.composites.Sequence(name="Drive Sequence", memory=False)
    driveSeq.add_children([driveCon, driveNode])

    idleSeq = actions.Idle(name="Idle")

    root.add_children([bootSeq, telecomSeq, safetySeq, armSeq, driveSeq, idleSeq])
    return root

class TreeNode(Node):
    def __init__(self):
        super().__init__("tree_node")

        root = create_tree()
        self.blackboard = root.attach_blackboard_client("TreeClient",namespace="/")
        
        for Key in BaseBlackboardKeys.ALL:
            self.blackboard.register_key(key = BaseBlackboardKeys.NAMESPACE + Key, access=py_trees.common.Access.WRITE)
            self.blackboard.set(name = BaseBlackboardKeys.NAMESPACE + Key, value = BaseBlackboardKeys.DEFAULT_VALUES[Key])

        self.tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)
        # Enable introspection
        self.tree.setup(
            timeout=15.0,
            node=self,
            enable_snapshot_stream=True
        )

        self.timer = self.create_timer(1.0, self.tick)

    def tick(self):
        self.tree.tick()


def main(args=None):
    rclpy.init(args=args)

    node = TreeNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()