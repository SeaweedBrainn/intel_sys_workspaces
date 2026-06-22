#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import random
import typing

import py_trees


class Boot(py_trees.behaviour.Behaviour):

    def __init__(self, name: str) -> None:
        super(Boot, self).__init__(name)

    def setup(self, **kwargs: typing.Any) -> None:
        pass

    def update(self) -> py_trees.common.Status:
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        pass

class Teleoperation(py_trees.behaviour.Behaviour):

    def __init__(self, name: str) -> None:
        super(Teleoperation, self).__init__(name)

    def setup(self, **kwargs: typing.Any) -> None:
        pass

    def update(self) -> py_trees.common.Status:
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        pass

class MakeSafe(py_trees.behaviour.Behaviour):

    def __init__(self, name: str) -> None:
        super(MakeSafe, self).__init__(name)

    def setup(self, **kwargs: typing.Any) -> None:
        pass

    def update(self) -> py_trees.common.Status:
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        pass

class Arm(py_trees.behaviour.Behaviour):

    def __init__(self, name: str) -> None:
        super(Arm, self).__init__(name)

    def setup(self, **kwargs: typing.Any) -> None:
        pass

    def update(self) -> py_trees.common.Status:
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        pass

class Drive(py_trees.behaviour.Behaviour):

    def __init__(self, name: str) -> None:
        super(Drive, self).__init__(name)

    def setup(self, **kwargs: typing.Any) -> None:
        pass

    def update(self) -> py_trees.common.Status:
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        pass

class Idle(py_trees.behaviour.Behaviour):

    def __init__(self, name: str) -> None:
        super(Idle, self).__init__(name)

    def setup(self, **kwargs: typing.Any) -> None:
        pass

    def update(self) -> py_trees.common.Status:
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        pass