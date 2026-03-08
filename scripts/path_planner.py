#!/usr/bin/env python
"""
Path Planning Node for Search and Rescue
Generates optimal coverage paths for search operations
"""

import rospy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
import numpy as np
from collections import deque

class PathPlanner:
    def __init__(self):
        rospy.init_node('path_planner', anonymous=False)
        
        # Parameters
        self.planning_algorithm = rospy.get_param('~planning_algorithm', 'coverage')
        self.cell_size = rospy.get_param('~cell_size', 1.0)
        self.robot_radius = rospy.get_param('~robot_radius', 0.3)
        
        # State
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.coverage_path = None
        
        # Publishers
        self.path_pub = rospy.Publisher('/sar/planned_path', Path, queue_size=10)
        self.status_pub = rospy.Publisher('/sar/planner_status', String, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/sar/replan_request', String, self.replan_callback)
        
        rospy.loginfo(f"Path Planner initialized with algorithm: {self.planning_algorithm}")
    
    def map_callback(self, msg):
        """Process incoming map data"""
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin
        
        rospy.loginfo_once("Map received, planning coverage path...")
        self.plan_coverage_path()
    
    def replan_callback(self, msg):
        """Handle replan requests"""
        rospy.loginfo("Replanning path...")
        self.plan_coverage_path()
    
    def plan_coverage_path(self):
        """Plan a coverage path based on the map"""
        if self.map_data is None:
            rospy.logwarn("No map data available for planning")
            return
        
        if self.planning_algorithm == 'coverage':
            path = self.coverage_path_planning()
        elif self.planning_algorithm == 'spiral':
            path = self.spiral_path_planning()
        elif self.planning_algorithm == 'boustrophedon':
            path = self.boustrophedon_decomposition()
        else:
            rospy.logerr(f"Unknown planning algorithm: {self.planning_algorithm}")
            return
        
        if path:
            self.coverage_path = path
            self.publish_path(path)
            rospy.loginfo(f"Coverage path generated with {len(path.poses)} waypoints")
    
    def coverage_path_planning(self):
        """
        Generate a systematic coverage path using grid-based planning
        """
        height, width = self.map_data.shape
        
        # Create a grid of cells to visit
        cell_width = int(self.cell_size / self.map_resolution)
        cell_height = int(self.cell_size / self.map_resolution)
        
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        
        # Generate zigzag pattern
        for i in range(0, height, cell_height):
            row_cells = range(0, width, cell_width)
            
            # Alternate direction for each row
            if (i // cell_height) % 2 == 1:
                row_cells = reversed(list(row_cells))
            
            for j in row_cells:
                # Check if cell is free
                if self.is_cell_free(i, j, cell_height, cell_width):
                    pose = PoseStamped()
                    pose.header = path.header
                    
                    # Convert grid to world coordinates
                    world_x = self.map_origin.position.x + j * self.map_resolution
                    world_y = self.map_origin.position.y + i * self.map_resolution
                    
                    pose.pose.position.x = world_x
                    pose.pose.position.y = world_y
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.w = 1.0
                    
                    path.poses.append(pose)
        
        return path
    
    def spiral_path_planning(self):
        """
        Generate a spiral coverage path from outside to inside
        """
        height, width = self.map_data.shape
        center_x, center_y = width // 2, height // 2
        
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        
        visited = set()
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # right, down, left, up
        dir_idx = 0
        steps = 1
        
        x, y = center_x, center_y
        
        while len(visited) < (height * width) // 100:  # Limit iterations
            for _ in range(2):  # Move in each direction pair
                dx, dy = directions[dir_idx % 4]
                
                for _ in range(steps):
                    if 0 <= y < height and 0 <= x < width:
                        if (x, y) not in visited and self.is_cell_free(y, x, 1, 1):
                            visited.add((x, y))
                            
                            pose = PoseStamped()
                            pose.header = path.header
                            
                            world_x = self.map_origin.position.x + x * self.map_resolution
                            world_y = self.map_origin.position.y + y * self.map_resolution
                            
                            pose.pose.position.x = world_x
                            pose.pose.position.y = world_y
                            pose.pose.position.z = 0.0
                            pose.pose.orientation.w = 1.0
                            
                            path.poses.append(pose)
                    
                    x += dx
                    y += dy
                
                dir_idx += 1
            
            steps += 1
        
        return path
    
    def boustrophedon_decomposition(self):
        """
        Boustrophedon (ox-plowing) path planning
        More efficient for rectangular areas
        """
        height, width = self.map_data.shape
        
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        
        cell_width = int(self.cell_size / self.map_resolution)
        stripe_width = int(self.robot_radius * 2 / self.map_resolution)
        
        direction = 1  # 1 for forward, -1 for backward
        
        for x in range(0, width, stripe_width):
            if direction == 1:
                y_range = range(0, height, cell_width)
            else:
                y_range = range(height - 1, -1, -cell_width)
            
            for y in y_range:
                if self.is_cell_free(y, x, cell_width, stripe_width):
                    pose = PoseStamped()
                    pose.header = path.header
                    
                    world_x = self.map_origin.position.x + x * self.map_resolution
                    world_y = self.map_origin.position.y + y * self.map_resolution
                    
                    pose.pose.position.x = world_x
                    pose.pose.position.y = world_y
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.w = 1.0
                    
                    path.poses.append(pose)
            
            direction *= -1  # Alternate direction
        
        return path
    
    def is_cell_free(self, row, col, height, width):
        """
        Check if a cell (and its neighborhood) is free of obstacles
        """
        map_height, map_width = self.map_data.shape
        
        # Check bounds
        if row < 0 or col < 0 or row + height > map_height or col + width > map_width:
            return False
        
        # Check if cell is free (value < 50 means free, -1 is unknown, >50 is occupied)
        cell_region = self.map_data[row:row+height, col:col+width]
        
        # Consider cell free if majority is free and no obstacles
        free_count = np.sum((cell_region >= 0) & (cell_region < 50))
        total_count = height * width
        
        return free_count > (total_count * 0.8)
    
    def optimize_path(self, path):
        """
        Optimize path by removing redundant waypoints
        """
        if len(path.poses) < 3:
            return path
        
        optimized_path = Path()
        optimized_path.header = path.header
        optimized_path.poses.append(path.poses[0])
        
        for i in range(1, len(path.poses) - 1):
            prev_pose = path.poses[i - 1]
            curr_pose = path.poses[i]
            next_pose = path.poses[i + 1]
            
            # Check if current point is on the line between prev and next
            if not self.is_collinear(prev_pose, curr_pose, next_pose):
                optimized_path.poses.append(curr_pose)
        
        optimized_path.poses.append(path.poses[-1])
        
        return optimized_path
    
    def is_collinear(self, p1, p2, p3, threshold=0.1):
        """Check if three points are approximately collinear"""
        x1, y1 = p1.pose.position.x, p1.pose.position.y
        x2, y2 = p2.pose.position.x, p2.pose.position.y
        x3, y3 = p3.pose.position.x, p3.pose.position.y
        
        # Calculate cross product
        cross_product = abs((y2 - y1) * (x3 - x2) - (y3 - y2) * (x2 - x1))
        
        return cross_product < threshold
    
    def publish_path(self, path):
        """Publish the planned path"""
        if path and len(path.poses) > 0:
            self.path_pub.publish(path)
            
            status_msg = String()
            status_msg.data = f"Path published with {len(path.poses)} waypoints"
            self.status_pub.publish(status_msg)
    
    def run(self):
        """Main run loop"""
        rospy.loginfo("Path Planner running...")
        rospy.spin()

if __name__ == '__main__':
    try:
        planner = PathPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Path Planner terminated")
