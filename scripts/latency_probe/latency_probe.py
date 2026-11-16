#!/usr/bin/env python3
"""Latency and responsiveness probe for ROS 2 topics.

Modes:
  stamp        Subscribe to an existing topic whose message has a header and
               measure receive_time - header.stamp (sensor-to-application latency).
               Also reports the observed message rate.
  loopback     Publish a PoseStamped at --rate Hz and subscribe to it in the same
               process; measures RMW delivery latency with no hardware involved.
  responsiveness
               Publish Twist commands on --cmd-topic at --rate Hz and measure the
               time from each command to the first message on --echo-topic that
               arrives after it. This is an upper bound: it includes the
               controller's own loop period.

Output is printed and written as JSON to --out so runs can be compared between
RMW implementations (see compare_rmw.sh).

Examples:
  python3 latency_probe.py stamp --topic /imu_raw --type sensor_msgs/msg/Imu --seconds 10 --out r.json
  python3 latency_probe.py loopback --rate 100 --seconds 10 --out r.json
  python3 latency_probe.py responsiveness --cmd-topic /cmd_vel \
      --echo-topic /odom/wheel --type nav_msgs/msg/Odometry --rate 20 --seconds 10 --out r.json

Note: responsiveness publishes motion commands. Only run it with the wheels free
or the robot suspended, and keep --rate low. Zero Twist is sent on exit.
"""
import argparse
import importlib
import json
import os
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def load_type(type_str):
    """'sensor_msgs/msg/Imu' -> sensor_msgs.msg.Imu class."""
    pkg, cls = type_str.split("/")[0], type_str.split("/")[-1]
    module = importlib.import_module(f"{pkg}.msg")
    return getattr(module, cls)


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def summarize(samples_ms, rate_hz, elapsed_s):
    out = {"samples": len(samples_ms), "rate_hz": round(rate_hz, 2),
           "duration_s": round(elapsed_s, 2)}
    if not samples_ms:
        out.update({"p50_ms": None, "p95_ms": None, "p99_ms": None,
                    "max_ms": None, "mean_ms": None})
        return out
    s = sorted(samples_ms)

    def pct(p):
        idx = min(len(s) - 1, max(0, int(round(p / 100.0 * (len(s) - 1)))))
        return s[idx]

    out.update({
        "p50_ms": round(pct(50), 3),
        "p95_ms": round(pct(95), 3),
        "p99_ms": round(pct(99), 3),
        "max_ms": round(s[-1], 3),
        "mean_ms": round(statistics.fmean(s), 3),
    })
    return out


class Probe(Node):
    def __init__(self, args):
        super().__init__("latency_probe")
        self.args = args
        self.samples = []
        self.msg_count = 0
        self.t_start = None
        self.pending_cmd_times = []   # responsiveness: publish times awaiting an echo
        self.resp_samples = []
        self.pub = None

    def run(self):
        a = self.args
        if a.mode == "stamp":
            msg_cls = load_type(a.type)
            self.create_subscription(msg_cls, a.topic, self.on_stamped, qos_profile_sensor_data)
        elif a.mode == "loopback":
            self.pub = self.create_publisher(PoseStamped, "/latency_probe/loop", 10)
            self.create_subscription(PoseStamped, "/latency_probe/loop", self.on_stamped, 10)
            self.create_timer(1.0 / a.rate, self.publish_loop)
        elif a.mode == "responsiveness":
            msg_cls = load_type(a.type)
            self.pub = self.create_publisher(Twist, a.cmd_topic, 10)
            self.create_subscription(msg_cls, a.echo_topic, self.on_echo, qos_profile_sensor_data)
            self.create_timer(1.0 / a.rate, self.publish_cmd)
        else:
            raise SystemExit(f"unknown mode {a.mode}")

        self.t_start = time.monotonic()
        deadline = self.t_start + a.seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)

        if a.mode == "responsiveness":
            self.pub.publish(Twist())   # stop the robot
        return self.result(time.monotonic() - self.t_start)

    def on_stamped(self, msg):
        self.msg_count += 1
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = stamp_to_sec(msg.header.stamp)
        if stamp > 0:
            self.samples.append((now - stamp) * 1000.0)

    def publish_loop(self):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "probe"
        self.pub.publish(m)

    def publish_cmd(self):
        self.pending_cmd_times.append(time.monotonic())
        t = Twist()
        t.linear.x = 0.0      # zero command: measures responsiveness without motion
        self.pub.publish(t)

    def on_echo(self, msg):
        self.msg_count += 1
        now = time.monotonic()
        # Match this echo to the most recent command published before it.
        cmds = [t for t in self.pending_cmd_times if t <= now]
        if cmds:
            self.resp_samples.append((now - cmds[0]) * 1000.0)
            self.pending_cmd_times.clear()

    def result(self, elapsed):
        a = self.args
        rmw = os.environ.get("RMW_IMPLEMENTATION", "default")
        if a.mode == "responsiveness":
            samples = self.resp_samples
        else:
            samples = self.samples
        rate = self.msg_count / elapsed if elapsed > 0 else 0.0
        out = {"mode": a.mode, "rmw": rmw,
               "topic": a.topic if a.mode == "stamp" else (a.echo_topic if a.mode == "responsiveness" else "/latency_probe/loop"),
               "type": a.type if a.mode != "loopback" else "geometry_msgs/msg/PoseStamped",
               "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S")}
        out.update(summarize(samples, rate, elapsed))
        return out


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("mode", choices=["stamp", "loopback", "responsiveness"])
    p.add_argument("--topic", help="stamp mode: topic to measure")
    p.add_argument("--type", help="message type, e.g. sensor_msgs/msg/Imu")
    p.add_argument("--cmd-topic", default="/cmd_vel", help="responsiveness: Twist command topic")
    p.add_argument("--echo-topic", help="responsiveness: topic that reflects the command")
    p.add_argument("--rate", type=float, default=20.0, help="publish rate Hz (loopback/responsiveness)")
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--out", help="write JSON result here")
    args = p.parse_args(argv)
    if args.mode == "stamp" and not (args.topic and args.type):
        p.error("stamp mode needs --topic and --type")
    if args.mode == "responsiveness" and not (args.echo_topic and args.type):
        p.error("responsiveness mode needs --echo-topic and --type")
    if args.rate <= 0 or args.rate > 100:
        p.error("--rate must be in (0, 100]")
    return args


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init()
    node = Probe(args)
    try:
        result = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    text = json.dumps(result, indent=2)
    print(text)
    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out, "w") as f:
            f.write(text + "\n")
    return 0 if result["samples"] > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
