#!/usr/bin/env python
"""
Communication Relay Node
Relays status and victim information to base station
"""

import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import json
import socket
import threading

class CommunicationRelay:
    def __init__(self):
        rospy.init_node('communication_relay', anonymous=False)
        
        # Parameters
        self.base_station_ip = rospy.get_param('~base_station_ip', '127.0.0.1')
        self.relay_port = rospy.get_param('~relay_port', 9090)
        self.update_rate = rospy.get_param('~update_rate', 1.0)  # Hz
        
        # State
        self.current_position = None
        self.latest_status = "Initializing..."
        self.victims_found = []
        self.socket = None
        self.connected = False
        
        # Publishers
        self.command_pub = rospy.Publisher('/sar/remote_command', String, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/sar/status', String, self.status_callback)
        rospy.Subscriber('/sar/victim_detected', PoseStamped, self.victim_callback)
        
        # Initialize socket connection
        self.init_connection()
        
        # Timer for periodic updates
        rospy.Timer(rospy.Duration(1.0 / self.update_rate), self.send_update)
        
        # Start command receiver thread
        self.receiver_thread = threading.Thread(target=self.receive_commands)
        self.receiver_thread.daemon = True
        self.receiver_thread.start()
        
        rospy.loginfo(f"Communication Relay initialized - Target: {self.base_station_ip}:{self.relay_port}")
    
    def init_connection(self):
        """Initialize socket connection to base station"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)
            # Note: In real deployment, would connect here
            # self.socket.connect((self.base_station_ip, self.relay_port))
            # self.connected = True
            rospy.loginfo("Communication relay ready (simulation mode)")
        except Exception as e:
            rospy.logwarn(f"Could not connect to base station: {e}")
            self.connected = False
    
    def odom_callback(self, msg):
        """Update current position"""
        self.current_position = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        }
    
    def status_callback(self, msg):
        """Update status message"""
        self.latest_status = msg.data
    
    def victim_callback(self, msg):
        """Log victim detection"""
        victim_data = {
            'id': len(self.victims_found) + 1,
            'timestamp': rospy.Time.now().to_sec(),
            'position': {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z
            }
        }
        self.victims_found.append(victim_data)
        
        rospy.logwarn(f"CRITICAL: Victim #{victim_data['id']} detected - Relaying to base station")
        
        # Send immediate update for victim detection
        self.send_victim_alert(victim_data)
    
    def send_update(self, event):
        """Send periodic status update to base station"""
        if not self.current_position:
            return
        
        update_data = {
            'type': 'status_update',
            'timestamp': rospy.Time.now().to_sec(),
            'robot_id': rospy.get_namespace(),
            'position': self.current_position,
            'status': self.latest_status,
            'victims_found': len(self.victims_found),
            'battery_level': 100.0,  # Would be from battery monitor
            'connection_strength': 95.0  # Would be from network monitor
        }
        
        self.send_data(update_data)
    
    def send_victim_alert(self, victim_data):
        """Send immediate alert for victim detection"""
        alert_data = {
            'type': 'victim_alert',
            'timestamp': rospy.Time.now().to_sec(),
            'robot_id': rospy.get_namespace(),
            'victim': victim_data,
            'priority': 'CRITICAL'
        }
        
        self.send_data(alert_data)
    
    def send_data(self, data):
        """Send data to base station"""
        try:
            json_data = json.dumps(data)
            
            # In real deployment, would send via socket
            if self.connected and self.socket:
                self.socket.sendall(json_data.encode('utf-8'))
            
            # Log for debugging
            rospy.logdebug(f"Sent to base station: {json_data[:100]}...")
            
        except Exception as e:
            rospy.logerr(f"Error sending data: {e}")
            self.connected = False
            # Attempt reconnection
            self.init_connection()
    
    def receive_commands(self):
        """Receive commands from base station (runs in separate thread)"""
        while not rospy.is_shutdown():
            try:
                if self.connected and self.socket:
                    data = self.socket.recv(1024)
                    if data:
                        command_str = data.decode('utf-8')
                        self.process_command(command_str)
                else:
                    rospy.sleep(1.0)
            except socket.timeout:
                continue
            except Exception as e:
                rospy.logerr(f"Error receiving command: {e}")
                rospy.sleep(1.0)
    
    def process_command(self, command_str):
        """Process command from base station"""
        try:
            command = json.loads(command_str)
            
            rospy.loginfo(f"Received command: {command.get('type', 'unknown')}")
            
            if command['type'] == 'abort_mission':
                rospy.logwarn("ABORT command received from base station")
                msg = String()
                msg.data = "ABORT"
                self.command_pub.publish(msg)
            
            elif command['type'] == 'return_home':
                rospy.loginfo("Return home command received")
                msg = String()
                msg.data = "RETURN_HOME"
                self.command_pub.publish(msg)
            
            elif command['type'] == 'change_search_pattern':
                pattern = command.get('pattern', 'coverage')
                rospy.loginfo(f"Changing search pattern to: {pattern}")
                msg = String()
                msg.data = f"CHANGE_PATTERN:{pattern}"
                self.command_pub.publish(msg)
            
            elif command['type'] == 'investigate_location':
                x = command.get('x', 0.0)
                y = command.get('y', 0.0)
                rospy.loginfo(f"Investigating location: ({x}, {y})")
                msg = String()
                msg.data = f"INVESTIGATE:{x},{y}"
                self.command_pub.publish(msg)
            
            else:
                rospy.logwarn(f"Unknown command type: {command.get('type')}")
                
        except json.JSONDecodeError as e:
            rospy.logerr(f"Invalid command format: {e}")
        except Exception as e:
            rospy.logerr(f"Error processing command: {e}")
    
    def generate_mission_report(self):
        """Generate final mission report"""
        report = {
            'type': 'mission_report',
            'timestamp': rospy.Time.now().to_sec(),
            'robot_id': rospy.get_namespace(),
            'mission_summary': {
                'status': 'COMPLETED',
                'victims_found': len(self.victims_found),
                'victim_details': self.victims_found,
                'final_position': self.current_position
            }
        }
        
        return report
    
    def cleanup(self):
        """Cleanup before shutdown"""
        rospy.loginfo("Sending final mission report...")
        
        report = self.generate_mission_report()
        self.send_data(report)
        
        if self.socket:
            self.socket.close()
        
        rospy.loginfo("Communication relay shutdown complete")
    
    def run(self):
        """Main run loop"""
        rospy.loginfo("Communication Relay running...")
        rospy.on_shutdown(self.cleanup)
        rospy.spin()

if __name__ == '__main__':
    try:
        relay = CommunicationRelay()
        relay.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Communication Relay terminated")
