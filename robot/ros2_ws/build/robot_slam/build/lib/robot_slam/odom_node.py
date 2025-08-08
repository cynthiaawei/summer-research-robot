#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import time
import threading
import math
from rclpy.qos import QoSProfile

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        
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
        
        # Arduino connection
        self.arduino = None
        self.arduino_connected = False
        
        # Timer for publishing
        self.create_timer(0.05, self.publish_odometry)  # 20Hz
        
        # Start Arduino connection in separate thread
        self.arduino_thread = threading.Thread(target=self.connect_arduino, daemon=True)
        self.arduino_thread.start()
        
        self.get_logger().info("🤖 Arduino Odometry node started")
        
    def connect_arduino(self):
        """Connect to Arduino and start reading data"""
        ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']
        
        for port in ports:
            try:
                self.get_logger().info(f"🔍 Trying to connect to {port}...")
                self.arduino = serial.Serial(port, 115200, timeout=2.0)
                time.sleep(2)  # Wait for Arduino reset
                
                # Test connection
                self.arduino.write(b'PING\n')
                time.sleep(0.5)
                response = self.arduino.readline().decode().strip()
                
                if 'PONG' in response:
                    self.arduino_connected = True
                    self.get_logger().info(f"✅ Arduino found on {port}")
                    break
                else:
                    self.arduino.close()
                    self.get_logger().warn(f"⚠️  No response from {port}")
                    
            except Exception as e:
                self.get_logger().warn(f"❌ Failed to connect to {port}: {e}")
                
        if not self.arduino_connected:
            self.get_logger().error("❌ Arduino Nano 33 IoT not found!")
            return
            
        # Start reading data
        self.read_arduino_data()
        
    def read_arduino_data(self):
        """Read and process Arduino data"""
        while rclpy.ok() and self.arduino_connected:
            try:
                if self.arduino and self.arduino.in_waiting > 0:
                    line = self.arduino.readline().decode().strip()
                    
                    if line.startswith('ODOM:'):
                        # Parse: ODOM:vx,vy,vth,dt
                        parts = line.replace('ODOM:', '').split(',')
                        if len(parts) >= 4:
                            vx = float(parts[0])
                            vy = float(parts[1]) 
                            vth = float(parts[2])
                            dt = float(parts[3])
                            
                            # Update robot pose
                            self.update_pose(vx, vy, vth, dt)
                            
            except Exception as e:
                self.get_logger().error(f"❌ Arduino read error: {e}")
                self.arduino_connected = False
                break
                
            time.sleep(0.01)
            
    def update_pose(self, vx, vy, vth, dt):
        """Update robot pose from velocity"""
        # Store velocities
        self.vx = vx
        self.vy = vy 
        self.vth = vth
        
        # Update position
        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        delta_th = vth * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_th
        
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
        
        # Add covariance (rough estimates)
        odom.pose.covariance = [0] * 36
        odom.twist.covariance = [0] * 36
        
        # Set diagonal elements (position and orientation uncertainty)
        odom.pose.covariance[0] = 0.1   # x
        odom.pose.covariance[7] = 0.1   # y  
        odom.pose.covariance[35] = 0.2  # yaw
        
        # Velocity uncertainty
        odom.twist.covariance[0] = 0.1   # vx
        odom.twist.covariance[7] = 0.1   # vy
        odom.twist.covariance[35] = 0.2  # vyaw
        
        # Publish odometry
        self.odom_pub.publish(odom)
        
        # Publish TF transform
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
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = OdometryNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()