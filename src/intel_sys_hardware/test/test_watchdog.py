from intel_sys_hardware.mecanum_controller import is_cmd_vel_stale

NS = 1_000_000_000


def test_fresh_command_not_stale():
    assert not is_cmd_vel_stale(last_ns=10 * NS, now_ns=10 * NS + int(0.1 * NS), timeout_s=0.5)


def test_exactly_at_timeout_not_stale():
    assert not is_cmd_vel_stale(last_ns=0, now_ns=int(0.5 * NS), timeout_s=0.5)


def test_past_timeout_is_stale():
    assert is_cmd_vel_stale(last_ns=0, now_ns=int(0.51 * NS), timeout_s=0.5)


def test_clock_going_backwards_is_stale():
    assert is_cmd_vel_stale(last_ns=10 * NS, now_ns=9 * NS, timeout_s=0.5)
