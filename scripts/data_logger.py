#!/usr/bin/env python
"""
Data Logger Node for Search and Rescue
Logs all mission-critical data for analysis and replay
"""

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image, LaserScan, Imu
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry, Path
import json
import os
from datetime import datetime
import csv

class DataLogger:
    def __init__(self):
        rospy.init_node('data_logger', anonymous=False)
        
        # Parameters
        self.log_directory = rospy.get_param('~log_directory', '/tmp/sar_logs')
        self.log_rate = rospy.get_param('~log_rate', 10)  # Hz
        self.save_images = rospy.get_param('~save_images', False)
        
        # Create log directory
        if not os.path.exists(self.log_directory):
            os.makedirs(self.log_directory)
        
        # Generate session ID
        self.session_id = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.session_dir = os.path.join(self.log_directory, self.session_id)
        os.makedirs(self.session_dir)
        
        # Data buffers
        self.odom_data = []
        self.victim_locations = []
        self.system_events = []
        self.sensor_data = []
        
        # CSV writers
        self.init_csv_files()
        
        # Subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/sar/victim_detected', PoseStamped, self.victim_callback)
        rospy.Subscriber('/sar/status', String, self.status_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        
        if self.save_images:
            rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        
        # Timer for periodic saving
        rospy.Timer(rospy.Duration(1.0 / self.log_rate), self.save_data_callback)
        
        rospy.loginfo(f"Data Logger initialized. Session ID: {self.session_id}")
        rospy.loginfo(f"Logs will be saved to: {self.session_dir}")
    
    def init_csv_files(self):
        """Initialize CSV files for different data types"""
        # Odometry log
        self.odom_file = open(os.path.join(self.session_dir, 'odometry.csv'), 'w')
        self.odom_writer = csv.writer(self.odom_file)
        self.odom_writer.writerow([
            'timestamp', 'pos_x', 'pos_y', 'pos_z', 
            'orient_x', 'orient_y', 'orient_z', 'orient_w',
            'linear_vel_x', 'linear_vel_y', 'angular_vel_z'
        ])
        
        # Victim locations log
        self.victim_file = open(os.path.join(self.session_dir, 'victims.csv'), 'w')
        self.victim_writer = csv.writer(self.victim_file)
        self.victim_writer.writerow([
            'timestamp', 'victim_id', 'pos_x', 'pos_y', 'pos_z', 'confidence'
        ])
        
        # System events log
        self.events_file = open(os.path.join(self.session_dir, 'events.csv'), 'w')
        self.events_writer = csv.writer(self.events_file)
        self.events_writer.writerow(['timestamp', 'event_type', 'description'])
        
        # Sensor data log
        self.sensor_file = open(os.path.join(self.session_dir, 'sensors.csv'), 'w')
        self.sensor_writer = csv.writer(self.sensor_file)
        self.sensor_writer.writerow([
            'timestamp', 'sensor_type', 'min_range', 'max_range', 'avg_range'
        ])
    
    def odom_callback(self, msg):
        """Log odometry data"""
        data = {
            'timestamp': msg.header.stamp.to_sec(),
            'pos_x': msg.pose.pose.position.x,
            'pos_y': msg.pose.pose.position.y,
            'pos_z': msg.pose.pose.position.z,
            'orient_x': msg.pose.pose.orientation.x,
            'orient_y': msg.pose.pose.orientation.y,
            'orient_z': msg.pose.pose.orientation.z,
            'orient_w': msg.pose.pose.orientation.w,
            'linear_vel_x': msg.twist.twist.linear.x,
            'linear_vel_y': msg.twist.twist.linear.y,
            'angular_vel_z': msg.twist.twist.angular.z
        }
        self.odom_data.append(data)
    
    def victim_callback(self, msg):
        """Log victim detection"""
        timestamp = rospy.Time.now().to_sec()
        victim_id = len(self.victim_locations) + 1
        
        data = {
            'timestamp': timestamp,
            'victim_id': victim_id,
            'pos_x': msg.pose.position.x,
            'pos_y': msg.pose.position.y,
            'pos_z': msg.pose.position.z,
            'confidence': 0.8  # Would come from detector
        }
        self.victim_locations.append(data)
        
        # Write immediately for important events
        self.victim_writer.writerow([
            data['timestamp'], data['victim_id'],
            data['pos_x'], data['pos_y'], data['pos_z'],
            data['confidence']
        ])
        self.victim_file.flush()
        
        rospy.loginfo(f"Logged victim detection #{victim_id}")
        
        # Also log as system event
        self.log_event('VICTIM_DETECTED', 
                      f"Victim #{victim_id} at ({data['pos_x']:.2f}, {data['pos_y']:.2f})")
    
    def status_callback(self, msg):
        """Log status messages"""
        self.log_event('STATUS_UPDATE', msg.data)
    
    def cmd_vel_callback(self, msg):
        """Log velocity commands"""
        # Could log these if needed for detailed analysis
        pass
    
    def scan_callback(self, msg):
        """Log laser scan summary statistics"""
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        
        if valid_ranges:
            data = {
                'timestamp': rospy.Time.now().to_sec(),
                'sensor_type': 'laser',
                'min_range': min(valid_ranges),
                'max_range': max(valid_ranges),
                'avg_range': sum(valid_ranges) / len(valid_ranges)
            }
            self.sensor_data.append(data)
    
    def imu_callback(self, msg):
        """Log IMU data summary"""
        data = {
            'timestamp': msg.header.stamp.to_sec(),
            'sensor_type': 'imu',
            'accel_x': msg.linear_acceleration.x,
            'accel_y': msg.linear_acceleration.y,
            'accel_z': msg.linear_acceleration.z
        }
        # Could add to sensor_data if needed
    
    def image_callback(self, msg):
        """Save images periodically"""
        # Save image to disk (throttled)
        if len(self.odom_data) % 100 == 0:  # Save every 100th image
            filename = f"image_{msg.header.stamp.to_sec()}.jpg"
            # Would use cv_bridge to save the image
            pass
    
    def log_event(self, event_type, description):
        """Log a system event"""
        timestamp = rospy.Time.now().to_sec()
        
        event = {
            'timestamp': timestamp,
            'event_type': event_type,
            'description': description
        }
        self.system_events.append(event)
        
        # Write immediately
        self.events_writer.writerow([timestamp, event_type, description])
        self.events_file.flush()
    
    def save_data_callback(self, event):
        """Periodically save buffered data to CSV files"""
        # Save odometry data
        for data in self.odom_data:
            self.odom_writer.writerow([
                data['timestamp'], data['pos_x'], data['pos_y'], data['pos_z'],
                data['orient_x'], data['orient_y'], data['orient_z'], data['orient_w'],
                data['linear_vel_x'], data['linear_vel_y'], data['angular_vel_z']
            ])
        self.odom_data.clear()
        self.odom_file.flush()
        
        # Save sensor data
        for data in self.sensor_data:
            self.sensor_writer.writerow([
                data['timestamp'], data['sensor_type'],
                data['min_range'], data['max_range'], data['avg_range']
            ])
        self.sensor_data.clear()
        self.sensor_file.flush()
    
    def save_mission_summary(self):
        """Save mission summary as JSON"""
        summary = {
            'session_id': self.session_id,
            'start_time': self.session_id,
            'end_time': datetime.now().strftime('%Y%m%d_%H%M%S'),
            'victims_found': len(self.victim_locations),
            'total_events': len(self.system_events),
            'victim_locations': self.victim_locations,
            'mission_statistics': {
                'odometry_samples': len(self.odom_data),
                'sensor_samples': len(self.sensor_data)
            }
        }
        
        summary_file = os.path.join(self.session_dir, 'mission_summary.json')
        with open(summary_file, 'w') as f:
            json.dump(summary, f, indent=2)
        
        rospy.loginfo(f"Mission summary saved to {summary_file}")
    
    def cleanup(self):
        """Close all files and save final summary"""
        rospy.loginfo("Cleaning up and saving final data...")
        
        # Save any remaining buffered data
        self.save_data_callback(None)
        
        # Close all files
        self.odom_file.close()
        self.victim_file.close()
        self.events_file.close()
        self.sensor_file.close()
        
        # Save mission summary
        self.save_mission_summary()
        
        rospy.loginfo(f"All data saved to {self.session_dir}")
    
    def run(self):
        """Main run loop"""
        rospy.loginfo("Data Logger running...")
        rospy.on_shutdown(self.cleanup)
        rospy.spin()

if __name__ == '__main__':
    try:
        logger = DataLogger()
        logger.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Data Logger terminated")
