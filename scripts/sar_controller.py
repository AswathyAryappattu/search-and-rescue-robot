#!/usr/bin/env python
"""
Search and Rescue Main Control Node
Coordinates the entire search and rescue operation
"""

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String, Bool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from cv_bridge import CvBridge
import numpy as np
import cv2
from enum import Enum

class RobotState(Enum):
    IDLE = 0
    SEARCHING = 1
    INVESTIGATING = 2
    RETURNING = 3
    EMERGENCY = 4

class SearchRescueController:
    def __init__(self):
        rospy.init_node('sar_controller', anonymous=False)
        
        # Parameters
        self.robot_name = rospy.get_param('~robot_name', 'rescue_robot')
        self.search_area_size = rospy.get_param('~search_area_size', 50.0)
        self.detection_threshold = rospy.get_param('~detection_threshold', 0.7)
        
        # State
        self.current_state = RobotState.IDLE
        self.current_pose = None
        self.home_position = None
        self.victims_found = []
        self.search_grid = None
        self.bridge = CvBridge()
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.status_pub = rospy.Publisher('/sar/status', String, queue_size=10)
        self.victim_pub = rospy.Publisher('/sar/victim_detected', PoseStamped, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/sar/victim_confirmation', Bool, self.victim_confirm_callback)
        
        # Action client for navigation
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        
        # Search pattern waypoints
        self.search_waypoints = []
        self.current_waypoint_idx = 0
        
        rospy.loginfo("Search and Rescue Controller initialized")
        
    def odom_callback(self, msg):
        """Update current robot position"""
        self.current_pose = msg.pose.pose
        if self.home_position is None:
            self.home_position = self.current_pose
            rospy.loginfo(f"Home position set at: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f})")
    
    def image_callback(self, msg):
        """Process camera images for victim detection"""
        if self.current_state != RobotState.SEARCHING:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Simple victim detection (look for human-like colors/shapes)
            # In real scenario, use ML model like YOLO or SSD
            victim_detected = self.detect_victim(cv_image)
            
            if victim_detected:
                rospy.logwarn("Potential victim detected! Investigating...")
                self.current_state = RobotState.INVESTIGATING
                self.stop_robot()
                self.investigate_victim()
                
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")
    
    def detect_victim(self, image):
        """
        Simplified victim detection algorithm
        In production, use trained ML model
        """
        # Convert to HSV for color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Detect skin tones (simplified)
        lower_skin = np.array([0, 20, 70], dtype=np.uint8)
        upper_skin = np.array([20, 255, 255], dtype=np.uint8)
        skin_mask = cv2.inRange(hsv, lower_skin, upper_skin)
        
        # Count skin pixels
        skin_pixels = cv2.countNonZero(skin_mask)
        total_pixels = image.shape[0] * image.shape[1]
        
        # If significant skin area detected
        if skin_pixels > total_pixels * 0.05:
            return True
        return False
    
    def laser_callback(self, msg):
        """Process laser scan for obstacle avoidance"""
        # Check for obstacles in front
        front_ranges = msg.ranges[len(msg.ranges)//2 - 10:len(msg.ranges)//2 + 10]
        min_distance = min([r for r in front_ranges if r > 0.1])
        
        if min_distance < 0.5 and self.current_state == RobotState.SEARCHING:
            rospy.logwarn("Obstacle detected! Adjusting path...")
            self.avoid_obstacle()
    
    def map_callback(self, msg):
        """Update map for path planning"""
        self.search_grid = msg
    
    def victim_confirm_callback(self, msg):
        """Handle victim confirmation from operator"""
        if msg.data:
            rospy.loginfo("Victim confirmed by operator!")
            victim_pose = PoseStamped()
            victim_pose.header.stamp = rospy.Time.now()
            victim_pose.header.frame_id = "map"
            victim_pose.pose = self.current_pose
            
            self.victims_found.append(victim_pose)
            self.victim_pub.publish(victim_pose)
            
            # Resume search
            self.current_state = RobotState.SEARCHING
        else:
            rospy.loginfo("False alarm. Resuming search...")
            self.current_state = RobotState.SEARCHING
    
    def generate_search_pattern(self):
        """Generate systematic search pattern waypoints"""
        rospy.loginfo("Generating search pattern...")
        
        # Create a grid-based search pattern
        spacing = 5.0  # meters between waypoints
        waypoints = []
        
        for i in range(int(self.search_area_size / spacing)):
            for j in range(int(self.search_area_size / spacing)):
                x = i * spacing
                y = j * spacing if i % 2 == 0 else self.search_area_size - j * spacing
                waypoints.append((x, y))
        
        self.search_waypoints = waypoints
        rospy.loginfo(f"Generated {len(waypoints)} waypoints")
    
    def navigate_to_waypoint(self, x, y):
        """Navigate to a specific waypoint"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0
        
        rospy.loginfo(f"Navigating to waypoint: ({x:.2f}, {y:.2f})")
        self.move_base_client.send_goal(goal)
        
        # Wait for result
        wait = self.move_base_client.wait_for_result(rospy.Duration(60.0))
        
        if wait:
            return self.move_base_client.get_state()
        else:
            rospy.logwarn("Timeout reaching waypoint")
            return None
    
    def stop_robot(self):
        """Stop the robot immediately"""
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
    
    def avoid_obstacle(self):
        """Simple obstacle avoidance"""
        # Rotate in place
        twist = Twist()
        twist.angular.z = 0.5
        
        for _ in range(20):
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)
        
        self.stop_robot()
    
    def investigate_victim(self):
        """Investigate potential victim detection"""
        rospy.loginfo("Investigating potential victim...")
        
        # Stop and wait for operator confirmation
        self.stop_robot()
        
        status_msg = String()
        status_msg.data = f"Potential victim at ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f}). Awaiting confirmation."
        self.status_pub.publish(status_msg)
        
        # In real scenario, take photos, thermal images, etc.
        rospy.sleep(2.0)
    
    def return_home(self):
        """Return to home position"""
        rospy.loginfo("Returning to home position...")
        self.current_state = RobotState.RETURNING
        
        if self.home_position:
            self.navigate_to_waypoint(
                self.home_position.position.x,
                self.home_position.position.y
            )
        
        rospy.loginfo("Arrived at home position")
        self.current_state = RobotState.IDLE
    
    def execute_search(self):
        """Main search execution loop"""
        rospy.loginfo("Starting search and rescue operation...")
        self.current_state = RobotState.SEARCHING
        
        self.generate_search_pattern()
        
        rate = rospy.Rate(1)  # 1 Hz
        
        while not rospy.is_shutdown() and self.current_waypoint_idx < len(self.search_waypoints):
            if self.current_state == RobotState.SEARCHING:
                # Navigate to next waypoint
                x, y = self.search_waypoints[self.current_waypoint_idx]
                result = self.navigate_to_waypoint(x, y)
                
                if result:
                    rospy.loginfo(f"Reached waypoint {self.current_waypoint_idx + 1}/{len(self.search_waypoints)}")
                    self.current_waypoint_idx += 1
                
                # Publish status
                status_msg = String()
                status_msg.data = f"Searching... {self.current_waypoint_idx}/{len(self.search_waypoints)} waypoints completed. Victims found: {len(self.victims_found)}"
                self.status_pub.publish(status_msg)
            
            elif self.current_state == RobotState.INVESTIGATING:
                # Wait while investigating
                rospy.sleep(1.0)
            
            rate.sleep()
        
        # Search complete
        rospy.loginfo(f"Search complete! Found {len(self.victims_found)} victims.")
        self.return_home()
    
    def run(self):
        """Main run loop"""
        rospy.loginfo("Search and Rescue Controller ready")
        
        # Wait for initial position
        while self.current_pose is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        # Start search operation
        self.execute_search()
        
        rospy.loginfo("Mission complete!")

if __name__ == '__main__':
    try:
        controller = SearchRescueController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Search and Rescue Controller terminated")
