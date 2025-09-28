#!/usr/bin/env python3
"""
Simple monitoring script for SLAM system.
Shows map statistics and scan matching performance.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import numpy as np

class SlamMonitor(Node):
    def __init__(self):
        super().__init__('slam_monitor')
        
        # Subscribe to topics
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, 'slam_pose', self.pose_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        
        # Statistics
        self.map_stats = {'free': 0, 'occupied': 0, 'unknown': 0}
        self.current_pose = None
        self.scan_count = 0
        self.last_print_time = self.get_clock().now()
        
        # Create timer for periodic status updates
        self.timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info('SLAM Monitor started')
    
    def map_callback(self, msg):
        """Analyze map data"""
        data = np.array(msg.data)
        
        # Count cell types
        self.map_stats['unknown'] = np.sum(data == -1)
        self.map_stats['free'] = np.sum((data >= 0) & (data < 50))
        self.map_stats['occupied'] = np.sum(data >= 50)
        
        total_cells = len(data)
        known_cells = total_cells - self.map_stats['unknown']
        known_percent = (known_cells / total_cells) * 100 if total_cells > 0 else 0
        
        # Only log significant changes
        if known_percent > 0.1:  # More than 0.1% of map is known
            self.get_logger().debug(f'Map: {known_percent:.2f}% explored')
    
    def pose_callback(self, msg):
        """Track robot pose"""
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        }
    
    def scan_callback(self, msg):
        """Count scans"""
        self.scan_count += 1
    
    def print_status(self):
        """Print periodic status update"""
        self.get_logger().info('='*50)
        self.get_logger().info('SLAM STATUS REPORT')
        
        # Map statistics
        total = sum(self.map_stats.values())
        if total > 0:
            unknown_pct = (self.map_stats['unknown'] / total) * 100
            free_pct = (self.map_stats['free'] / total) * 100
            occupied_pct = (self.map_stats['occupied'] / total) * 100
            
            self.get_logger().info(f'Map Coverage:')
            self.get_logger().info(f'  Unknown:  {unknown_pct:.1f}% ({self.map_stats["unknown"]} cells)')
            self.get_logger().info(f'  Free:     {free_pct:.1f}% ({self.map_stats["free"]} cells)')
            self.get_logger().info(f'  Occupied: {occupied_pct:.1f}% ({self.map_stats["occupied"]} cells)')
        
        # Pose
        if self.current_pose:
            self.get_logger().info(f'Robot Pose: x={self.current_pose["x"]:.3f}, y={self.current_pose["y"]:.3f}, z={self.current_pose["z"]:.3f}')
        else:
            self.get_logger().info('Robot Pose: Not yet received')
        
        # Scan rate
        time_now = self.get_clock().now()
        duration = (time_now - self.last_print_time).nanoseconds / 1e9
        if duration > 0:
            scan_rate = self.scan_count / duration
            self.get_logger().info(f'Scan Rate: {scan_rate:.1f} Hz')
        
        self.scan_count = 0
        self.last_print_time = time_now
        self.get_logger().info('='*50)

def main(args=None):
    rclpy.init(args=args)
    monitor = SlamMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
