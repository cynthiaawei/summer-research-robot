#!/usr/bin/env python3
# nav2_bt_bridge.py — ROS2 node that converts Nav2 global path into
# (direction, duration_ms, speed_percent) steps and sends them to the Pi over Bluetooth.
#
# Usage (after Nav2 is up):
#   python3 nav2_bt_bridge.py --bt-addr AA:BB:CC:DD:EE:FF
#
# Safe testing defaults:
#   linear_speed = 0.07 m/s, angular_speed = 0.25 rad/s, speed_percent = 18
#   waypoint stride ≈ 0.4 m, heading threshold = 20 deg
#
import argparse
import json
import math
import socket
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile

# PyBluez
try:
    from bluetooth import (
        BluetoothSocket, RFCOMM, find_service
    )
except Exception as e:
    print("PyBluez not available. Install with: sudo apt-get install python3-bluez")
    raise

UUID_SPP = "00001101-0000-1000-8000-00805F9B34FB"

def norm_angle(a):
    # wrap to [-pi, pi]
    return (a + math.pi) % (2 * math.pi) - math.pi

def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

class BTClient:
    def __init__(self, addr, port=None, uuid=UUID_SPP):
        self.addr = addr
        self.port = port
        self.uuid = uuid
        self.sock = None

    def connect(self):
        if self.port is None:
            svc = find_service(uuid=self.uuid, address=self.addr)
            if not svc:
                raise RuntimeError("No SPP service found on device. Is the Pi server running & paired?")
            self.port = svc[0]["port"]

        self.sock = BluetoothSocket(RFCOMM)
        self.sock.settimeout(8.0)
        self.sock.connect((self.addr, self.port))
        self.sock.settimeout(2.0)

    def send_jsonl(self, obj):
        if not self.sock:
            return False
        data = (json.dumps(obj) + "\n").encode("utf-8")
        try:
            self.sock.sendall(data)
            return True
        except (socket.error, OSError):
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
            return False

    def ensure(self):
        if self.sock:
            return True
        try:
            self.connect()
            return True
        except Exception:
            return False

    def close(self):
        try:
            if self.sock:
                self.sock.close()
        except Exception:
            pass
        self.sock = None


class Nav2BTBridge(Node):
    def __init__(self, bt_addr, bt_port, test_lin, test_ang, speed_pct, stride_m, hdg_thresh_deg):
        super().__init__("nav2_bt_bridge")
        self.bt = BTClient(bt_addr, bt_port)
        self.lin = float(test_lin)         # m/s (slow)
        self.ang = float(test_ang)         # rad/s (slow)
        self.speed_pct = int(speed_pct)    # your motor % on the Pi
        self.stride = float(stride_m)      # meters between targets
        self.hdg_thresh = math.radians(float(hdg_thresh_deg))
        self.last_plan_stamp = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(depth=1)
        self.plan_sub = self.create_subscription(Path, "/plan", self.plan_cb, qos)
        self.get_logger().info(
            f"BT target={bt_addr} | lin={self.lin:.2f} m/s | ang={self.ang:.2f} rad/s | "
            f"speed%={self.speed_pct} | stride={self.stride} m | hdg_thresh={hdg_thresh_deg}°"
        )

    # ---- ROS callback -------------------------------------------------
    def plan_cb(self, msg: Path):
        if not msg.poses:
            return
        if self.last_plan_stamp and msg.header.stamp == self.last_plan_stamp:
            return
        self.last_plan_stamp = msg.header.stamp

        # get robot yaw from TF (map -> base_link or base_footprint)
        yaw = None
        x = y = None
        for base in ("base_link", "base_footprint"):
            try:
                tf = self.tf_buffer.lookup_transform("map", base, rclpy.time.Time(), timeout=Duration(seconds=0.2))
                t = tf.transform.translation
                q = tf.transform.rotation
                yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
                x, y = t.x, t.y
                break
            except Exception:
                continue

        if yaw is None:
            self.get_logger().warn("TF not ready; skipping plan->steps")
            return

        # decimate waypoints to ~stride_m spacing starting from current pose projection
        waypts = []
        last = None
        for ps in msg.poses:
            wx, wy = ps.pose.position.x, ps.pose.position.y
            if last is None:
                waypts.append((wx, wy))
                last = (wx, wy)
            else:
                if math.hypot(wx - last[0], wy - last[1]) >= self.stride:
                    waypts.append((wx, wy))
                    last = (wx, wy)
        # always include final goal
        gx, gy = msg.poses[-1].pose.position.x, msg.poses[-1].pose.position.y
        if not waypts or (waypts and math.hypot(waypts[-1][0]-gx, waypts[-1][1]-gy) > 0.1):
            waypts.append((gx, gy))

        steps = self._build_steps_from_waypoints(x, y, yaw, waypts)
        if not steps:
            self.get_logger().warn("No steps produced; skipping send")
            return

        payload = {
            "type": "plan_steps",
            "meta": {
                "timestamp": time.time(),
                "linear_mps": self.lin,
                "angular_rps": self.ang,
                "speed_percent": self.speed_pct,
                "count": len(steps),
            },
            "steps": steps
        }

        if not self.bt.ensure():
            self.get_logger().warn("Bluetooth not connected; retrying next update")
            return

        ok = self.bt.send_jsonl(payload)
        if ok:
            self.get_logger().info(f"Sent {len(steps)} steps over Bluetooth.")
        else:
            self.get_logger().warn("Bluetooth send failed; will reconnect.")

    # ---- plan -> (direction, duration_ms) -----------------------------
    def _build_steps_from_waypoints(self, x, y, yaw, waypts, max_steps=40):
        steps = []
        curx, cury, curyaw = float(x), float(y), float(yaw)

        for (tx, ty) in waypts:
            # desired heading to next target
            dx, dy = tx - curx, ty - cury
            dist = math.hypot(dx, dy)
            if dist < 0.05:
                continue
            desired = math.atan2(dy, dx)
            err = norm_angle(desired - curyaw)

            # rotate if heading error exceeds threshold
            if abs(err) > self.hdg_thresh:
                deg = math.degrees(abs(err))
                dur_s = deg * (math.pi/180.0) / max(1e-3, self.ang)
                steps.append({
                    "direction": "turnleft" if err > 0 else "turnright",
                    "duration_ms": int(1000 * min(dur_s, 5.0)),  # cap each spin to 5s
                    "speed_percent": self.speed_pct
                })
                # update yaw as if we rotated
                curyaw = desired

            # drive straight to target
            dur_s = dist / max(1e-3, self.lin)
            steps.append({
                "direction": "forward",
                "duration_ms": int(1000 * min(dur_s, 8.0)),     # cap each straight to 8s
                "speed_percent": self.speed_pct
            })
            # "teleport" model state to the target for coarse chaining
            curx, cury = tx, ty

            if len(steps) >= max_steps:
                break

        # always finish with a stop
        steps.append({"direction": "stop", "duration_ms": 0, "speed_percent": self.speed_pct})
        # prune tiny moves (<150 ms)
        steps = [s for s in steps if s["duration_ms"] >= 150 or s["direction"] == "stop"]
        return steps


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bt-addr", required=True, help="Bluetooth MAC address of Raspberry Pi")
    parser.add_argument("--bt-port", type=int, default=None, help="RFCOMM port. Omit to auto-discover via SPP UUID")
    parser.add_argument("--linear", type=float, default=0.07, help="Test linear speed (m/s)")
    parser.add_argument("--angular", type=float, default=0.25, help="Test angular speed (rad/s)")
    parser.add_argument("--speed-pct", type=int, default=18, help="Motor speed percent sent to Pi")
    parser.add_argument("--stride-m", type=float, default=0.40, help="Waypoint stride in meters")
    parser.add_argument("--heading-thresh-deg", type=float, default=20.0, help="Rotate if heading error exceeds this")
    args = parser.parse_args()

    rclpy.init()
    node = Nav2BTBridge(
        bt_addr=args.bt_addr,
        bt_port=args.bt_port,
        test_lin=args.linear,
        test_ang=args.angular,
        speed_pct=args.speed_pct,
        stride_m=args.stride_m,
        hdg_thresh_deg=args.heading_thresh_deg,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.bt.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
