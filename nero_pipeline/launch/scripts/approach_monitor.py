#!/usr/bin/env python3
import math, os
import rclpy, rclpy.time
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tf2_ros

TOPIC = os.environ.get("JOINT_STATES_TOPIC", "/control/joint_states")

class Monitor(Node):
    def __init__(self):
        super().__init__("approach_monitor")
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)
        self.create_subscription(JointState, TOPIC, self.on_js, 10)
        self.create_timer(0.25, self.tick)
        self.last_print = None
        self.grip_open = None
        self.closes = 0

    def pose(self):
        try:
            t = self.buf.lookup_transform("base_link", "tcp_link", rclpy.time.Time())
            f = self.buf.lookup_transform("base_link", "gripper_link1", rclpy.time.Time())
        except Exception:
            return None
        p, fp = t.transform.translation, f.transform.translation
        v = (fp.x - p.x, fp.y - p.y, fp.z - p.z)
        n = math.sqrt(sum(c * c for c in v)) or 1.0
        ang = math.degrees(math.asin(max(-1.0, min(1.0, v[2] / n))))  # + above / - below horizontal
        return ang, p.z

    @staticmethod
    def fmt(a):
        if a < 0:
            return f"{-a:3.0f} deg BELOW horizontal (90 = straight down)"
        return f"{a:3.0f} deg ABOVE horizontal  <-- pointing UP"

    def on_js(self, msg):
        if "gripper" not in msg.name:
            return
        g = msg.position[msg.name.index("gripper")]
        is_open = g > 0.07
        if self.grip_open is None:
            self.grip_open = is_open
            return
        if is_open == self.grip_open:
            return
        self.grip_open = is_open
        r = self.pose()
        if r is None:
            return
        if is_open:
            label = "RELEASE" if self.closes >= 1 else "PRE-GRASP OPEN"
        else:
            self.closes += 1
            label = "GRASP (closing)" if self.closes == 1 else f"CLOSE #{self.closes}"
        print(f"*** {label}: fingers point {self.fmt(r[0])}, tcp z={r[1]:+.3f}", flush=True)

    def tick(self):
        r = self.pose()
        if r is None:
            return
        if self.last_print is None or abs(r[0] - self.last_print) >= 5:
            self.last_print = r[0]
            print(f"    fingers: {self.fmt(r[0])} | tcp z={r[1]:+.3f}", flush=True)

rclpy.init()
rclpy.spin(Monitor())
