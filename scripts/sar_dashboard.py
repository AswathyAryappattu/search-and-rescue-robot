#!/usr/bin/env python3
"""
Simple GUI Dashboard for SAR System
Shows real-time victim detections and status
"""

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import tkinter as tk
from tkinter import ttk
import threading
from datetime import datetime

class SARDashboard:
    def __init__(self):
        rospy.init_node('sar_dashboard', anonymous=True)
        
        # Data storage
        self.victims = []
        self.current_position = {"x": 0.0, "y": 0.0}
        self.status = "Initializing..."
        self.waypoint_count = 0
        
        # Create GUI
        self.root = tk.Tk()
        self.root.title("🚁 Search & Rescue Dashboard")
        self.root.geometry("800x600")
        self.root.configure(bg='#2C3E50')
        
        self.setup_gui()
        
        # ROS Subscribers
        rospy.Subscriber('/sar/victim_detected', PoseStamped, self.victim_callback)
        rospy.Subscriber('/sar/status', String, self.status_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Start ROS in separate thread
        self.ros_thread = threading.Thread(target=self.ros_spin)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
        # Update GUI periodically
        self.update_gui()
        
        rospy.loginfo("SAR Dashboard started")
    
    def setup_gui(self):
        """Setup GUI elements"""
        # Title
        title = tk.Label(
            self.root, 
            text="🚁 SEARCH & RESCUE CONTROL CENTER",
            font=("Arial", 20, "bold"),
            bg='#2C3E50',
            fg='#ECF0F1'
        )
        title.pack(pady=20)
        
        # Status Frame
        status_frame = tk.Frame(self.root, bg='#34495E', relief=tk.RAISED, bd=2)
        status_frame.pack(fill=tk.BOTH, padx=20, pady=10)
        
        tk.Label(
            status_frame,
            text="MISSION STATUS",
            font=("Arial", 14, "bold"),
            bg='#34495E',
            fg='#3498DB'
        ).pack(pady=5)
        
        self.status_label = tk.Label(
            status_frame,
            text="Initializing...",
            font=("Arial", 12),
            bg='#34495E',
            fg='#ECF0F1',
            wraplength=700
        )
        self.status_label.pack(pady=10)
        
        # Robot Position Frame
        pos_frame = tk.Frame(self.root, bg='#34495E', relief=tk.RAISED, bd=2)
        pos_frame.pack(fill=tk.BOTH, padx=20, pady=10)
        
        tk.Label(
            pos_frame,
            text="ROBOT POSITION",
            font=("Arial", 14, "bold"),
            bg='#34495E',
            fg='#2ECC71'
        ).pack(pady=5)
        
        self.position_label = tk.Label(
            pos_frame,
            text="X: 0.00 m  |  Y: 0.00 m",
            font=("Arial", 12, "bold"),
            bg='#34495E',
            fg='#ECF0F1'
        )
        self.position_label.pack(pady=10)
        
        # Victims Frame
        victims_frame = tk.Frame(self.root, bg='#34495E', relief=tk.RAISED, bd=2)
        victims_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)
        
        tk.Label(
            victims_frame,
            text="🚨 VICTIMS DETECTED",
            font=("Arial", 14, "bold"),
            bg='#34495E',
            fg='#E74C3C'
        ).pack(pady=5)
        
        self.victim_count_label = tk.Label(
            victims_frame,
            text="Total: 0",
            font=("Arial", 16, "bold"),
            bg='#34495E',
            fg='#E74C3C'
        )
        self.victim_count_label.pack(pady=5)
        
        # Victim List
        list_frame = tk.Frame(victims_frame, bg='#34495E')
        list_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Scrollbar
        scrollbar = tk.Scrollbar(list_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Listbox
        self.victim_listbox = tk.Listbox(
            list_frame,
            font=("Courier", 11),
            bg='#2C3E50',
            fg='#ECF0F1',
            selectbackground='#E74C3C',
            yscrollcommand=scrollbar.set,
            height=10
        )
        self.victim_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.victim_listbox.yview)
        
        # Add header
        self.victim_listbox.insert(tk.END, "ID    X (m)     Y (m)     TIME")
        self.victim_listbox.insert(tk.END, "-" * 50)
    
    def victim_callback(self, msg):
        """Handle victim detection"""
        victim_id = len(self.victims) + 1
        x = msg.pose.position.x
        y = msg.pose.position.y
        time = datetime.now().strftime("%H:%M:%S")
        
        victim_data = {
            'id': victim_id,
            'x': x,
            'y': y,
            'time': time
        }
        
        self.victims.append(victim_data)
        rospy.logwarn(f"Dashboard: Victim #{victim_id} detected at ({x:.2f}, {y:.2f})")
    
    def status_callback(self, msg):
        """Handle status updates"""
        self.status = msg.data
    
    def odom_callback(self, msg):
        """Handle odometry updates"""
        self.current_position['x'] = msg.pose.pose.position.x
        self.current_position['y'] = msg.pose.pose.position.y
    
    def update_gui(self):
        """Update GUI with latest data"""
        # Update status
        self.status_label.config(text=self.status)
        
        # Update position
        pos_text = f"X: {self.current_position['x']:.2f} m  |  Y: {self.current_position['y']:.2f} m"
        self.position_label.config(text=pos_text)
        
        # Update victim count
        self.victim_count_label.config(text=f"Total: {len(self.victims)}")
        
        # Update victim list
        current_items = self.victim_listbox.size()
        expected_items = len(self.victims) + 2  # +2 for header lines
        
        if current_items != expected_items:
            # Clear and rebuild list
            self.victim_listbox.delete(2, tk.END)  # Keep header
            
            for victim in self.victims:
                line = f"#{victim['id']:<3}  {victim['x']:<8.2f}  {victim['y']:<8.2f}  {victim['time']}"
                self.victim_listbox.insert(tk.END, line)
                
                # Highlight new victims
                if victim == self.victims[-1]:
                    self.victim_listbox.itemconfig(tk.END, bg='#E74C3C', fg='#FFFFFF')
        
        # Schedule next update
        self.root.after(100, self.update_gui)
    
    def ros_spin(self):
        """ROS spin in separate thread"""
        rospy.spin()
    
    def run(self):
        """Start the dashboard"""
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            rospy.loginfo("Dashboard closed")

if __name__ == '__main__':
    try:
        dashboard = SARDashboard()
        dashboard.run()
    except rospy.ROSInterruptException:
        pass
