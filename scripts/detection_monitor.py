#!/usr/bin/env python3
"""
Detection Monitor - Visual feedback for victim detections
Run this to get clear terminal feedback about detections
"""

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from colorama import Fore, Back, Style, init
import sys

# Initialize colorama for colored terminal output
init(autoreset=True)

class DetectionMonitor:
    def __init__(self):
        rospy.init_node('detection_monitor', anonymous=True)
        
        self.victim_count = 0
        self.detection_history = []
        
        # Subscribers
        rospy.Subscriber('/sar/victim_detected', PoseStamped, self.victim_callback)
        rospy.Subscriber('/sar/status', String, self.status_callback)
        rospy.Subscriber('/sar/detection_info', String, self.detection_info_callback)
        
        self.print_header()
        
        rospy.loginfo("Detection Monitor started - Watch for victim detections!")
    
    def print_header(self):
        """Print a nice header"""
        print("\n" + "="*70)
        print(Fore.CYAN + Style.BRIGHT + "  🚁 SEARCH AND RESCUE - VICTIM DETECTION MONITOR 🚁")
        print("="*70 + "\n")
        print(Fore.YELLOW + "Monitoring for victim detections...")
        print(Fore.WHITE + "Status updates will appear below:\n")
    
    def victim_callback(self, msg):
        """Handle victim detection"""
        self.victim_count += 1
        
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        # Store detection
        self.detection_history.append({
            'id': self.victim_count,
            'x': x,
            'y': y,
            'z': z,
            'time': rospy.Time.now()
        })
        
        # Print dramatic alert
        print("\n" + "🚨 " * 20)
        print(Back.RED + Fore.WHITE + Style.BRIGHT + 
              f"  ⚠️  VICTIM DETECTED - ID: {self.victim_count}  ⚠️  ")
        print("🚨 " * 20)
        
        print(Fore.GREEN + Style.BRIGHT + f"\n📍 Location:")
        print(f"   X: {x:.2f} meters")
        print(f"   Y: {y:.2f} meters")
        print(f"   Z: {z:.2f} meters")
        
        print(Fore.CYAN + f"\n🕐 Time: {rospy.Time.now().to_sec():.2f}")
        print(Fore.YELLOW + f"📊 Total Victims Found: {self.victim_count}")
        print("\n" + "-"*70 + "\n")
    
    def status_callback(self, msg):
        """Handle status updates"""
        status = msg.data
        
        # Color code based on content
        if "victim" in status.lower() or "detected" in status.lower():
            print(Fore.RED + Style.BRIGHT + f"🔴 {status}")
        elif "searching" in status.lower():
            print(Fore.BLUE + f"🔍 {status}")
        elif "complete" in status.lower():
            print(Fore.GREEN + Style.BRIGHT + f"✅ {status}")
        else:
            print(Fore.WHITE + f"ℹ️  {status}")
    
    def detection_info_callback(self, msg):
        """Handle detection info"""
        info = msg.data
        
        if "detection" in info.lower():
            # Extract confidence if present
            if "confidence" in info.lower():
                print(Fore.MAGENTA + f"   🎯 {info}")
            else:
                print(Fore.CYAN + f"   ℹ️  {info}")
    
    def print_summary(self):
        """Print final summary"""
        print("\n" + "="*70)
        print(Fore.CYAN + Style.BRIGHT + "MISSION SUMMARY")
        print("="*70)
        print(Fore.YELLOW + f"\n📊 Total Victims Detected: {self.victim_count}\n")
        
        if self.detection_history:
            print(Fore.GREEN + "Victim Locations:")
            print("-" * 70)
            print(f"{'ID':<5} {'X (m)':<15} {'Y (m)':<15} {'Z (m)':<15}")
            print("-" * 70)
            
            for victim in self.detection_history:
                print(f"{victim['id']:<5} {victim['x']:<15.2f} {victim['y']:<15.2f} {victim['z']:<15.2f}")
        
        print("\n" + "="*70 + "\n")
    
    def run(self):
        """Main run loop"""
        rospy.on_shutdown(self.print_summary)
        rospy.spin()

if __name__ == '__main__':
    try:
        monitor = DetectionMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\n\nMonitor stopped by user.")
        sys.exit(0)
