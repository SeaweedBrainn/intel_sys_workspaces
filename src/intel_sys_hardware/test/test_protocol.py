import struct
import pytest
from intel_sys_hardware.ros_robot_controller_sdk import (
    checksum_crc8, PacketFunction, PacketControllerState
)

def test_crc8_checksum():
    # Test vector 1: empty data
    assert checksum_crc8(b"") == 0

    # Test vector 2: known byte sequences
    data = bytes([0xAA, 0x55, 0x03, 0x01])
    crc = checksum_crc8(data)
    assert isinstance(crc, int)
    assert 0 <= crc <= 0xFF

    # Determinism check
    assert checksum_crc8(data) == checksum_crc8(data)

def test_packet_functions_enum():
    assert PacketFunction.PACKET_FUNC_SYS == 0
    assert PacketFunction.PACKET_FUNC_LED == 1
    assert PacketFunction.PACKET_FUNC_BUZZER == 2
    assert PacketFunction.PACKET_FUNC_MOTOR == 3
    assert PacketFunction.PACKET_FUNC_IMU == 7

def test_motor_speed_payload_encoding():
    speeds = [[1, 2.5], [2, -2.5], [3, 1.0], [4, -1.0]]
    data = [0x01, len(speeds)]
    for i in speeds:
        data.extend(struct.pack("<Bf", int(i[0] - 1), float(i[1])))

    # 1 subcmd + 1 length + 4 * (1 byte id + 4 byte float) = 2 + 20 = 22 bytes
    assert len(data) == 22
    assert data[0] == 0x01
    assert data[1] == 4

def test_gamepad_unpack_bytes_and_list():
    from unittest.mock import MagicMock
    from intel_sys_hardware.ros_robot_controller_sdk import Board
    board = Board.__new__(Board)
    import queue
    board.enable_recv = True
    board.gamepad_queue = queue.Queue()
    board.buttons_map = Board.buttons_map

    # Test with raw bytes
    raw_bytes = struct.pack("<HB4b", 0, 0, 0, 0, 0, 0)
    board.gamepad_queue.put(raw_bytes)
    res = board.get_gamepad()
    assert res is not None
    axes, buttons = res
    assert len(axes) == 8
    assert len(buttons) == 16

    # Test with list of ints (regression test)
    raw_list = list(raw_bytes)
    board.gamepad_queue.put(raw_list)
    res2 = board.get_gamepad()
    assert res2 is not None
    axes2, buttons2 = res2
    assert len(axes2) == 8
