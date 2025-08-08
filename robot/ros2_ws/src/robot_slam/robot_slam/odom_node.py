#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import socket
import time
import threading
import math
from rclpy.qos import QoSProfile

class NetworkOdometryNode(Node):
    def __init__(self):
        super().__init__('network_odom_node')
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publisher with reliable QoS
        qos_profile = QoSProfile(depth=10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_profile)
        
        # Robot state
        self.x = 0.0
        self.y = 0.0 
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        
        # Network connection
        self.socket = None
        self.connected = False
        self.windows_ip = self.get_windows_ip()
        
        # Timer for publishing
        self.create_timer(0.05, self.publish_odometry)  # 20Hz
        
        # Start network connection
        self.connect_thread = threading.Thread(target=self.connect_to_bridge, daemon=True)
        self.connect_thread.start()
        
        self.get_logger().info("🤖 Network Arduino Odometry node started")
        self.get_logger().info(f"🌐 Connecting to Windows bridge at {self.windows_ip}:9999")
        
    def get_windows_ip(self):
        """Get Windows host IP from WSL2"""
        try:
            with open('/etc/resolv.conf', 'r') as f:
                for line in f:
                    if 'nameserver' in line:
                        return line.split()[1]
        except:
            pass
        return "172.18.144.1"  # Default WSL2 host IP
        
    def connect_to_bridge(self):
        """Connect to Arduino bridge on Windows"""
        retry_count = 0
        while rclpy.ok() and retry_count < 10:
            try:
                self.get_logger().info(f"🔍 Attempt {retry_count + 1}: Connecting to {self.windows_ip}:9999")
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(5)
                self.socket.connect((self.windows_ip, 9999))
                
                self.connected = True
                self.get_logger().info("✅ Connected to Arduino bridge!")
                
                # Send PING to test
                self.socket.send(b'PING\n')
                
                # Start reading data
                self.read_network_data()
                break
                
            except Exception as e:
                self.get_logger().warn(f"❌ Connection failed: {e}")
                if self.socket:
                    self.socket.close()
                retry_count += 1
                time.sleep(2)
                
        if not self.connected:
            self.get_logger().error("❌ Could not connect to Arduino bridge after 10 attempts!")
            self.get_logger().info("📝 Make sure arduino_bridge.py is running on Windows")
                
    def read_network_data(self):
        """Read Arduino data via network"""
        buffer = ""
        while rclpy.ok() and self.connected:
            try:
                self.socket.settimeout(1.0)
                data = self.socket.recv(1024).decode('utf-8', errors='ignore')
                
                if data:
                    buffer += data
                    
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        if line.startswith('ODOM:'):
                            # Parse: ODOM:pos_x,pos_y,heading,vel_x,vel_y,vel_angular
                            parts = line.replace('ODOM:', '').split(',')
                            if len(parts) >= 6:
                                try:
                                    pos_x = float(parts[0])
                                    pos_y = float(parts[1])
                                    heading = float(parts[2])
                                    vel_x = float(parts[3])
                                    vel_y = float(parts[4])
                                    vel_angular = float(parts[5])
                                    
                                    # Update robot state
                                    self.x = pos_x
                                    self.y = pos_y
                                    self.theta = heading
                                    self.vx = vel_x
                                    self.vy = vel_y
                                    self.vth = vel_angular
                                    
                                except ValueError as e:
                                    self.get_logger().warn(f"⚠️ Parse error: {e}")
                        
                        elif line == 'PONG':
                            self.get_logger().info("🏓 Arduino bridge responding")
                            
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"❌ Network error: {e}")
                self.connected = False
                break
                
    def euler_to_quaternion(self, yaw):
        """Convert yaw to quaternion"""
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        
    def publish_odometry(self):
        """Publish odometry message and TF"""
        current_time = self.get_clock().now()
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.euler_to_quaternion(self.theta)
        
        # Velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth
        
        # Add covariance
        odom.pose.covariance = [0] * 36
        odom.twist.covariance = [0] * 36
        odom.pose.covariance[0] = 0.1   # x
        odom.pose.covariance[7] = 0.1   # y  
        odom.pose.covariance[35] = 0.2  # yaw
        odom.twist.covariance[0] = 0.1   # vx
        odom.twist.covariance[7] = 0.1   # vy
        odom.twist.covariance[35] = 0.2  # vyaw
        
        # Publish
        self.odom_pub.publish(odom)
        
        # TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = current_time.to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = self.euler_to_quaternion(self.theta)
        self.tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = NetworkOdometryNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()