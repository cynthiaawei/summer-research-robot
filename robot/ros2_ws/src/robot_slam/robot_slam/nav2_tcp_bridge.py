#!/usr/bin/env python3
# nav2_tcp_bridge.py - Converts Nav2 paths to movement commands over TCP
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

def norm_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

class TCPClient:
    def __init__(self, host, port=9999):
        self.host = host
        self.port = port
        self.sock = None

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(5.0)
        self.sock.connect((self.host, self.port))

    def send_jsonl(self, obj):
        if not self.sock:
            return False
        data = (json.dumps(obj) + "\n").encode("utf-8")
        try:
            self.sock.sendall(data)
            return True
        except:
            try:
                self.sock.close()
            except:
                pass
            self.sock = None
            return False

    def ensure(self):
        if self.sock:
            return True
        try:
            self.connect()
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False

    def close(self):
        try:
            if self.sock:
                self.sock.close()
        except:
            pass
        self.sock = None

class Nav2TCPBridge(Node):
    def __init__(self, host, port, test_lin, test_ang, stride_m, hdg_thresh_deg):
        super().__init__("nav2_tcp_bridge")
        self.tcp = TCPClient(host, port)
        self.lin = float(test_lin)
        self.ang = float(test_ang)
        self.stride = float(stride_m)
        self.hdg_thresh = math.radians(float(hdg_thresh_deg))
        self.last_plan_stamp = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(depth=1)
        self.plan_sub = self.create_subscription(Path, "/plan", self.plan_cb, qos)
        self.get_logger().info(
            f"TCP target={host}:{port} | lin={self.lin:.2f} m/s | ang={self.ang:.2f} rad/s | "
            f"stride={self.stride} m | hdg_thresh={hdg_thresh_deg}° | Using robot default speeds"
        )

    def plan_cb(self, msg: Path):
        if not msg.poses:
            return
        if self.last_plan_stamp and msg.header.stamp == self.last_plan_stamp:
            return
        self.last_plan_stamp = msg.header.stamp

        # Get robot pose from TF
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

        # Decimate waypoints to stride spacing
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

        # Always include final goal
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
                "count": len(steps),
            },
            "steps": steps
        }

        if not self.tcp.ensure():
            self.get_logger().warn("TCP not connected; retrying next update")
            return

        ok = self.tcp.send_jsonl(payload)
        if ok:
            self.get_logger().info(f"Sent {len(steps)} steps over TCP.")
        else:
            self.get_logger().warn("TCP send failed; will reconnect.")

    def _build_steps_from_waypoints(self, x, y, yaw, waypts, max_steps=40):
        steps = []
        curx, cury, curyaw = float(x), float(y), float(yaw)

        for (tx, ty) in waypts:
            # Calculate heading to next target
            dx, dy = tx - curx, ty - cury
            dist = math.hypot(dx, dy)
            if dist < 0.05:
                continue
            desired = math.atan2(dy, dx)
            err = norm_angle(desired - curyaw)

            # Rotate if heading error exceeds threshold
            if abs(err) > self.hdg_thresh:
                deg = math.degrees(abs(err))
                dur_s = deg * (math.pi/180.0) / max(1e-3, self.ang)
                steps.append({
                    "direction": "turnleft" if err > 0 else "turnright",
                    "duration_ms": int(1000 * min(dur_s, 5.0))  # No speed_percent
                })
                curyaw = desired

            # Drive straight to target
            dur_s = dist / max(1e-3, self.lin)
            steps.append({
                "direction": "forward",
                "duration_ms": int(1000 * min(dur_s, 8.0))  # No speed_percent
            })
            curx, cury = tx, ty

            if len(steps) >= max_steps:
                break

        # Always finish with stop
        steps.append({"direction": "stop", "duration_ms": 0})
        
        # Remove tiny moves (< 150ms)
        steps = [s for s in steps if s["duration_ms"] >= 150 or s["direction"] == "stop"]
        return steps

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", required=True, help="Pi IP address or hostname")
    parser.add_argument("--port", type=int, default=9999, help="TCP port")
    parser.add_argument("--linear", type=float, default=0.07, help="Planning linear speed (m/s)")
    parser.add_argument("--angular", type=float, default=0.25, help="Planning angular speed (rad/s)")
    parser.add_argument("--stride-m", type=float, default=0.40, help="Waypoint stride in meters")
    parser.add_argument("--heading-thresh-deg", type=float, default=20.0, help="Rotate if heading error exceeds this")
    args = parser.parse_args()

    rclpy.init()
    node = Nav2TCPBridge(
        host=args.host,
        port=args.port,
        test_lin=args.linear,
        test_ang=args.angular,
        stride_m=args.stride_m,
        hdg_thresh_deg=args.heading_thresh_deg,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.tcp.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()