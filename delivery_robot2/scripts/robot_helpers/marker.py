#!/usr/bin/env python3
from visualization_msgs.msg import Marker
import rospy

class MarkerManager:
    def publish_robot_marker(self, position, marker_pub):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "robot"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 0.5
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        # Check if marker_pub is publishing to the correct type
        if marker_pub.type == "visualization_msgs/Marker":
            try:
                marker_pub.publish(marker)
            except rospy.ROSException as e:
                rospy.logerr(f"[MARKER] Failed to publish robot marker: {e}")
        else:
            rospy.logerr("[MARKER] Incorrect publisher type for marker_pub. Expected visualization_msgs/Marker.")


    def add_destination_marker(self, destination_name, position, marker_pub):
        rospy.loginfo(f"[MARKER] Added marker at {position[0]}, {position[1]}")
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "destination"
        marker.id = hash(destination_name) % 1000  # Unique ID for each destination
        marker.type = Marker.CUBE
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 0.5
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        try:
            marker_pub.publish(marker)
            return marker
        except rospy.ROSException as e:
            rospy.logerr(f"[MARKER] Failed to publish destination marker: {e}")

    def mark_destination_reached(self, marker_pub, marker):
        rospy.loginfo("[MARKER] Marking destination as reached.")
        marker.color.r = 0.0
        marker.color.g = 1.0  # Change color to green
        marker.color.b = 0.0
        marker.header.stamp = rospy.Time.now()  # Update timestamp
        marker_pub.publish(marker)
        