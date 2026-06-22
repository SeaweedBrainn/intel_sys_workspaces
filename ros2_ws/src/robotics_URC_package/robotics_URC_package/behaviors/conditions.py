#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import random
import typing
import py_trees.blackboard as blackboard
from robotics_URC_package.blackboard.config import BaseBlackboardKeys
import py_trees


class isBooted(py_trees.behaviour.Behaviour):

    def __init__(self, name: str, inverted: bool = False) -> None:
        super(isBooted, self).__init__(name)
        self.bb = self.attach_blackboard_client(name = "BehaviorClient", namespace=BaseBlackboardKeys.NAMESPACE)
        self.bb.register_key(key=BaseBlackboardKeys.IS_BOOTED, access=py_trees.common.Access.READ)
        self.inverted = inverted
        

    def setup(self, **kwargs: typing.Any) -> None:
        pass

    def update(self) -> py_trees.common.Status:
        if self.inverted != True:
            if self.bb.isBooted:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        else:
            if self.bb.isBooted:
                return py_trees.common.Status.FAILURE
            else:
                return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status) -> None:
        pass

class isTeleoperation(py_trees.behaviour.Behaviour):

    def __init__(self, name: str) -> None:
        super(isTeleoperation, self).__init__(name)
        self.bb = self.attach_blackboard_client(name = "BehaviorClient", namespace=BaseBlackboardKeys.NAMESPACE)
        self.bb.register_key(key=BaseBlackboardKeys.IS_TELEOP, access=py_trees.common.Access.READ)

    def setup(self, **kwargs: typing.Any) -> None:
        pass

    def update(self) -> py_trees.common.Status:
        if self.bb.isTeleop:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status: py_trees.common.Status) -> None:
        pass

class isSafe(py_trees.behaviour.Behaviour):

    def __init__(self, name: str, inverted: bool = False) -> None:
        super(isSafe, self).__init__(name)
        self.bb = self.attach_blackboard_client(name = "BehaviorClient", namespace=BaseBlackboardKeys.NAMESPACE)
        self.bb.register_key(key=BaseBlackboardKeys.IS_SAFE, access=py_trees.common.Access.READ)
        self.inverted = inverted

    def setup(self, **kwargs: typing.Any) -> None:
        pass

    def update(self) -> py_trees.common.Status:
        if self.inverted != True:
            if self.bb.isSafe:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        else:
            if self.bb.isSafe:
                return py_trees.common.Status.FAILURE
            else:
                return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status) -> None:
        pass

class isArm(py_trees.behaviour.Behaviour):

    def __init__(self, name: str) -> None:
        super(isArm, self).__init__(name)
        self.bb = self.attach_blackboard_client(name = "BehaviorClient", namespace=BaseBlackboardKeys.NAMESPACE)
        self.bb.register_key(key=BaseBlackboardKeys.IS_ARM, access=py_trees.common.Access.READ)

    def setup(self, **kwargs: typing.Any) -> None:
        pass

    def update(self) -> py_trees.common.Status:
        if self.bb.isArm:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status: py_trees.common.Status) -> None:
        pass

class isDrive(py_trees.behaviour.Behaviour):

    def __init__(self, name: str) -> None:
        super(isDrive, self).__init__(name)
        self.bb = self.attach_blackboard_client(name = "BehaviorClient", namespace=BaseBlackboardKeys.NAMESPACE)
        self.bb.register_key(key=BaseBlackboardKeys.IS_DRIVE, access=py_trees.common.Access.READ)

    def setup(self, **kwargs: typing.Any) -> None:
        pass

    def update(self) -> py_trees.common.Status:
        if self.bb.isDrive:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status: py_trees.common.Status) -> None:
        pass