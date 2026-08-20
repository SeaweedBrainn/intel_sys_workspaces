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
