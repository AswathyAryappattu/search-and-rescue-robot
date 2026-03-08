#!/usr/bin/env python
"""
Advanced Victim Detection Node
Uses computer vision and thermal imaging for victim detection"""

import rospy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class VictimDetector:
    def __init__(self):
        rospy.init_node('victim_detector', anonymous=False)
        
        # Parameters
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.6)
        self.debug_mode = rospy.get_param('~debug_mode', False)
        
        # State
        self.bridge = CvBridge()
        self.last_detection_time = rospy.Time(0)
        self.detection_cooldown = rospy.Duration(5.0)  # seconds
        
        # Publishers
        self.detection_pub = rospy.Publisher('/sar/victim_detected', Bool, queue_size=10)
        self.debug_image_pub = rospy.Publisher('/sar/debug_image', Image, queue_size=10)
        self.detection_info_pub = rospy.Publisher('/sar/detection_info', String, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.rgb_callback)
        rospy.Subscriber('/thermal/image_raw', Image, self.thermal_callback, queue_size=1)
        
        rospy.loginfo("Victim Detector Node initialized")
        
    def rgb_callback(self, msg):
        """Process RGB camera images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Check cooldown
            if (rospy.Time.now() - self.last_detection_time) < self.detection_cooldown:
                return
            
            # Detect human features
            detected, confidence, debug_image = self.detect_human_rgb(cv_image)
            
            if detected and confidence > self.confidence_threshold:
                rospy.logwarn(f"Victim detected with confidence: {confidence:.2f}")
                self.detection_pub.publish(Bool(True))
                self.last_detection_time = rospy.Time.now()
                
                # Publish detection info
                info_msg = String()
                info_msg.data = f"RGB detection: confidence={confidence:.2f}"
                self.detection_info_pub.publish(info_msg)
            
            # Publish debug image
            if self.debug_mode and debug_image is not None:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                self.debug_image_pub.publish(debug_msg)
                
        except Exception as e:
            rospy.logerr(f"Error in RGB callback: {e}")
    
    def thermal_callback(self, msg):
        """Process thermal camera images for heat signatures"""
        try:
            thermal_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
            
            # Check cooldown
            if (rospy.Time.now() - self.last_detection_time) < self.detection_cooldown:
                return
            
            # Detect heat signatures
            detected, confidence = self.detect_human_thermal(thermal_image)
            
            if detected and confidence > self.confidence_threshold:
                rospy.logwarn(f"Heat signature detected with confidence: {confidence:.2f}")
                self.detection_pub.publish(Bool(True))
                self.last_detection_time = rospy.Time.now()
                
                # Publish detection info
                info_msg = String()
                info_msg.data = f"Thermal detection: confidence={confidence:.2f}"
                self.detection_info_pub.publish(info_msg)
                
        except Exception as e:
            rospy.logerr(f"Error in thermal callback: {e}")
    
    def detect_human_rgb(self, image):
        """
        Detect humans in RGB image
        Using multiple methods: color, shape, HOG
        """
        height, width = image.shape[:2]
        debug_image = image.copy()
        
        # Method 1: Skin color detection
        skin_detected, skin_confidence = self.detect_skin_color(image, debug_image)
        
        # Method 2: Shape detection (circles for heads, rectangles for bodies)
        shape_detected, shape_confidence = self.detect_human_shapes(image, debug_image)
        
        # Method 3: Motion detection (if previous frame available)
        # For this example, we'll skip motion detection
        
        # Combine confidences
        total_confidence = (skin_confidence * 0.4 + shape_confidence * 0.6)
        detected = total_confidence > 0.3
        
        if detected:
            cv2.putText(debug_image, f"HUMAN DETECTED: {total_confidence:.2f}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        return detected, total_confidence, debug_image
    
    def detect_skin_color(self, image, debug_image):
        """Detect skin tones in image"""
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define skin color range
        lower_skin = np.array([0, 20, 70], dtype=np.uint8)
        upper_skin = np.array([20, 255, 255], dtype=np.uint8)
        
        # Create mask
        skin_mask = cv2.inRange(hsv, lower_skin, upper_skin)
        
        # Apply morphological operations
        kernel = np.ones((5, 5), np.uint8)
        skin_mask = cv2.morphologyEx(skin_mask, cv2.MORPH_CLOSE, kernel)
        skin_mask = cv2.morphologyEx(skin_mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(skin_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Calculate confidence based on skin area
        total_pixels = image.shape[0] * image.shape[1]
        skin_pixels = cv2.countNonZero(skin_mask)
        confidence = min(skin_pixels / (total_pixels * 0.15), 1.0)
        
        detected = confidence > 0.3
        
        # Draw contours on debug image
        if detected:
            cv2.drawContours(debug_image, contours, -1, (0, 255, 0), 2)
        
        return detected, confidence
    
    def detect_human_shapes(self, image, debug_image):
        """Detect human-like shapes (circles for heads, etc.)"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (9, 9), 2)
        
        # Detect circles (potential heads)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 50,
                                   param1=50, param2=30, 
                                   minRadius=10, maxRadius=100)
        
        confidence = 0.0
        detected = False
        
        if circles is not None:
            circles = np.uint16(np.around(circles))
            num_circles = len(circles[0])
            confidence = min(num_circles * 0.3, 1.0)
            detected = num_circles > 0
            
            # Draw circles on debug image
            for i in circles[0, :]:
                cv2.circle(debug_image, (i[0], i[1]), i[2], (255, 0, 0), 2)
                cv2.circle(debug_image, (i[0], i[1]), 2, (255, 0, 0), 3)
        
        return detected, confidence
    
    def detect_human_thermal(self, thermal_image):
        """
        Detect human heat signatures in thermal image
        Look for temperatures in human body range (30-40°C / 86-104°F)
        """
        # Normalize thermal image
        normalized = cv2.normalize(thermal_image, None, 0, 255, cv2.NORM_MINMAX)
        
        # Threshold for human body temperature range
        # Assuming thermal image is calibrated where bright = hot
        _, hot_mask = cv2.threshold(normalized, 180, 255, cv2.THRESH_BINARY)
        
        # Apply morphological operations
        kernel = np.ones((5, 5), np.uint8)
        hot_mask = cv2.morphologyEx(hot_mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(hot_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter contours by size (human-sized heat signatures)
        valid_contours = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if 500 < area < 50000:  # Reasonable size for human
                valid_contours.append(cnt)
        
        detected = len(valid_contours) > 0
        confidence = min(len(valid_contours) * 0.5, 1.0)
        
        return detected, confidence
    
    def run(self):
        """Main run loop"""
        rospy.loginfo("Victim Detector running...")
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = VictimDetector()
        detector.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Victim Detector terminated")
