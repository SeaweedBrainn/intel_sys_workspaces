#!/usr/bin/env python3
# -*- coding: utf-8 -*-

class BaseBlackboardKeys:
    NAMESPACE = "base/"
    
    IS_BOOTED = "isBooted"
    IS_TELEOP = "isTeleop"
    IS_SAFE = "isSafe"
    IS_ARM = "isArm"
    IS_DRIVE = "isDrive"
    IS_IDLE = "isIdle"

    ALL = [IS_BOOTED,
           IS_TELEOP,
           IS_SAFE,
           IS_ARM,
           IS_DRIVE,
           IS_IDLE]
    
    DEFAULT_VALUES = {
        IS_BOOTED: False,
        IS_TELEOP: False,
        IS_SAFE: True,
        IS_ARM: False,
        IS_DRIVE: False,
        IS_IDLE: True
    }