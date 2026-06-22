#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import py_trees
from rclpy.node import Node
from std_msgs.msg import String
import json
from robotics_URC_package.blackboard.config import BaseBlackboardKeys

class BlackboardPublisher(Node):
    def __init__(self):
        super().__init__("blackboard_publisher")

        self.publisher = self.create_publisher(String, "blackboard_state", 10)

        self.blackboard = py_trees.blackboard.Client(
            name="BlackboardPublisher",
            namespace="/"
        )

        for Key in BaseBlackboardKeys.ALL:
            self.blackboard.register_key(
                key=BaseBlackboardKeys.NAMESPACE + Key,
                access=py_trees.common.Access.WRITE  # ← was READ, needs WRITE to handle commands
            )

        # ── NEW: listens for external write commands ──────────────────
        self.create_subscription(
            String,
            "blackboard_command",
            self._on_command,
            10
        )

        self.timer = self.create_timer(0.5, self.publish_blackboard)

    def _on_command(self, msg):
        try:
            command = json.loads(msg.data)
            key = command["key"]    # e.g. "base/isBooted"
            value = command["value"]  # e.g. True

            # Strip namespace prefix since our client is already at "/"
            # "base/isBooted" -> set via blackboard client directly
            self.blackboard.set(key, value, overwrite=True)
            self.get_logger().info(f"Command received: {key} = {value}")
        except (KeyError, json.JSONDecodeError) as e:
            self.get_logger().warn(f"Bad command: {e}")

    def publish_blackboard(self):
        data = {}
        for base in BaseBlackboardKeys.ALL:
            key = BaseBlackboardKeys.NAMESPACE + base
            try:
                data[key] = self.blackboard.get(key)
            except KeyError:
                data[key] = None
        msg = String()
        msg.data = json.dumps(data)
        self.publisher.publish(msg)
        self.get_logger().debug(f"Blackboard: {data}")