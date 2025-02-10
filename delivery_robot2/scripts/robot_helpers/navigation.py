from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import rospy
from math import sqrt

class NavigationManager:
    def __init__(self, position_manager, marker_manager, marker_pub):
        self.position_manager = position_manager
        self.marker_manager = marker_manager
        self.marker_pub = marker_pub

    def navigate_to_destination(self, start_location, destination, position_pub, path_pub, marker):

        rate = rospy.Rate(2)  # 2 Hz
        path = Path()
        path.header.frame_id = "map"

        current_position = self.position_manager.get_position()
        rospy.loginfo(f"[NAVIGATION] Start navigation from {start_location}, {destination}")
    

        if(start_location != current_position):
            while not rospy.is_shutdown() and not self.reached_destination(current_position, start_location):
                pose_msg = PoseStamped()
                pose_msg.header.frame_id = "map"
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.pose.position.x = current_position[0]
                pose_msg.pose.position.y = current_position[1]

                # Publish the pose
                position_pub.publish(pose_msg)

                # Update the path
                path.poses.append(pose_msg)
                path_pub.publish(path)

                # Update current position towards destination
                for i in range(2):
                    if current_position[i] < start_location[i]:
                        current_position[i] = min(start_location[i], current_position[i] + 0.1)
                    elif current_position[i] > start_location[i]:
                        current_position[i] = max(start_location[i], current_position[i] - 0.1)

                self.marker_manager.publish_robot_marker(current_position, self.marker_pub)
                rate.sleep()
            self.position_manager.update_position(current_position)


        while not rospy.is_shutdown() and not self.reached_destination(current_position, destination):
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "map"
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.pose.position.x = current_position[0]
            pose_msg.pose.position.y = current_position[1]

            # Publish the pose
            position_pub.publish(pose_msg)

            # Update the path
            path.poses.append(pose_msg)
            path_pub.publish(path)

            # Update current position towards destination
            for i in range(2):
                if current_position[i] < destination[i]:
                    current_position[i] = min(destination[i], current_position[i] + 0.1)
                elif current_position[i] > destination[i]:
                    current_position[i] = max(destination[i], current_position[i] - 0.1)

            self.marker_manager.publish_robot_marker(current_position, self.marker_pub)
            rate.sleep()

        rospy.loginfo("[NAVIGATION] Destination reached.")
        self.marker_manager.mark_destination_reached(self.marker_pub, marker)
        self.position_manager.update_position(current_position)

    def reached_destination(self, current_position, destination):
        return sqrt((current_position[0] - destination[0])**2 + (current_position[1] - destination[1])**2) < 0.1