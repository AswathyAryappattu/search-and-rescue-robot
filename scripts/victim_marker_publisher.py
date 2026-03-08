#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

class VictimMarkerPublisher:
    def __init__(self):
        rospy.init_node("victim_marker_publisher", anonymous=True)

        # Publisher for RViz markers
        self.marker_pub = rospy.Publisher(
            "/sar/victim_markers",
            MarkerArray,
            queue_size=10
        )

        # Subscribe to victim pose (PoseStamped)
        self.victim_sub = rospy.Subscriber(
            "/sar/victim_detected",       # <-- MUST match the real topic
            PoseStamped,                  # <-- TYPE must match publisher
            self.victim_callback
        )

        self.markers = []   # store all detected victims
        self.next_id = 0    # unique ID for each marker

    def victim_callback(self, msg: PoseStamped):
        """Called whenever a new victim pose is received"""

        # Create a new marker
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id or "map"
        marker.header.stamp = rospy.Time.now()

        marker.ns = "victims"
        marker.id = self.next_id
        self.next_id += 1

        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose = msg.pose

        # Size of the cylinder (your values)
        marker.scale.x = 1.0   # diameter in x
        marker.scale.y = 1.0   # diameter in y
        marker.scale.z = 0.1   # height

        # Color - semi-transparent red
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.3

        marker.lifetime = rospy.Duration(0)  # forever

        # Store and republish all markers
        self.markers.append(marker)
        marker_array = MarkerArray()
        marker_array.markers = self.markers
        self.marker_pub.publish(marker_array)

    def run(self):
        rospy.loginfo("Victim Marker Publisher running...")
        rospy.spin()

if __name__ == '__main__':
    try:
        publisher = VictimMarkerPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Victim Marker Publisher terminated")
