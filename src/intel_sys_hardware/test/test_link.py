"""STM32 link liveness: the bridge must notice a dead serial port (docs/ROADMAP.md TODO)."""
import struct
import time

import serial

from intel_sys_hardware import ros_robot_controller_sdk as sdk
from intel_sys_hardware.ros_robot_controller_sdk import Board, PacketFunction, checksum_crc8, link_alive


def test_link_alive_rules():
    assert link_alive(last_frame=10.0, failed=False, now=10.5, timeout=1.0)
    assert not link_alive(last_frame=10.0, failed=False, now=11.5, timeout=1.0)   # silent
    assert not link_alive(last_frame=10.0, failed=True, now=10.1, timeout=1.0)    # I/O error
    assert not link_alive(last_frame=None, failed=False, now=10.0, timeout=1.0)   # never heard


def imu_frame():
    payload = list(struct.pack('<6f', 0, 0, 1, 0, 0, 0))
    body = [int(PacketFunction.PACKET_FUNC_IMU), len(payload)] + payload
    return bytes([0xAA, 0x55] + body + [checksum_crc8(bytes(body))])


class FakeSerial:
    """Feeds scripted reads; an Exception in the script is raised like a dead USB port."""

    script = []

    def __init__(self, *_a, **_k):
        self.is_open = False
        self.rts = self.dtr = False
        self.reads = list(FakeSerial.script)
        self.written = []

    def setPort(self, _p):
        pass

    def open(self):
        self.is_open = True

    def close(self):
        self.is_open = False

    def read(self):
        if not self.reads:
            time.sleep(0.01)
            return b''
        r = self.reads.pop(0)
        if isinstance(r, Exception):
            raise r
        return r

    def write(self, buf):
        if not self.is_open:
            raise serial.SerialException('write on a closed port')
        self.written.append(bytes(buf))


def board_with(monkeypatch, script):
    FakeSerial.script = script
    monkeypatch.setattr(sdk.serial, 'Serial', FakeSerial)
    b = Board(device='/dev/null', baudrate=1)
    b.enable_reception(True)
    return b


def wait_for(cond, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.01)
    return False


def test_valid_frame_marks_the_link_alive(monkeypatch):
    b = board_with(monkeypatch, [imu_frame()])
    assert wait_for(lambda: b.last_frame is not None)
    assert not b.failed
    assert link_alive(b.last_frame, b.failed, time.monotonic(), 1.0)
    b.close()


def test_read_error_marks_the_link_failed(monkeypatch):
    b = board_with(monkeypatch, [imu_frame(), serial.SerialException('device reports readiness to read but returned no data')])
    assert wait_for(lambda: b.failed)
    assert not link_alive(b.last_frame, b.failed, time.monotonic(), 1.0)
    b.close()


def test_write_error_marks_the_link_failed_instead_of_raising(monkeypatch):
    b = board_with(monkeypatch, [])
    b.port.is_open = True

    def dead(_buf):
        raise serial.SerialException('write failed: [Errno 5] Input/output error')
    b.port.write = dead
    b.set_motor_speed([[1, 0.0]])   # must not raise into the ROS callback
    assert b.failed
    b.close()
