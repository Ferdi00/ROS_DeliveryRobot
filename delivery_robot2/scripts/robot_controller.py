#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from delivery_robot2.msg import DeliveryOrder
from nav_msgs.msg import Path

from robot_helpers.position import PositionManager
from robot_helpers.marker import MarkerManager
from robot_helpers.navigation import NavigationManager
from delivery_robot2.srv import GetOrdersList, GetOrdersListResponse

priority_map = {"high": 1, "medium": 2, "low": 3}
queue_updated = False  # Flag to track updates to the order queue

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller')
        rospy.loginfo("[ROBOT_CONTROLLER] Robot Controller node initialized.")

        self.order_queue = []  # Queue to store incoming orders
        self.completed_orders = set()  # Track completed orders

        # Load predefined points from ROS parameters
        self.predefined_points = rospy.get_param('predefined_points')
        
        # Publishers
        self.position_pub = rospy.Publisher('/robot/pose', PoseStamped, queue_size=10)
        self.marker_pub = rospy.Publisher('/robot/visual_marker', Marker, queue_size=10)
        self.path_pub = rospy.Publisher('/robot/path', Path, queue_size=10)

        # Initialize helpers
        self.position_manager = PositionManager()
        self.marker_manager = MarkerManager()
        self.navigation_manager = NavigationManager(
            self.position_manager, self.marker_manager, self.marker_pub
        )

        # Subscriber to receive orders
        self.order_sub = rospy.Subscriber('/delivery_order', DeliveryOrder, self.handle_new_order)

        # Service to get the list of orders
        self.get_orders_service = rospy.Service('/get_orders_list', GetOrdersList, self.handle_get_orders_list)

        # Initialize robot marker
        self.marker_manager.publish_robot_marker(self.position_manager.get_position(), self.marker_pub)

        rate = rospy.Rate(0.2)  # 0.2 Hz -> 1 loop every 5 seconds
        while not rospy.is_shutdown():
            if self.order_queue:
                # Sort the queue only if it has been updated
                if self.queue_updated:
                    self.order_queue.sort(key=lambda order: priority_map.get(order.priority, 4))
                    self.queue_updated = False  # Reset the flag
                self.process_next_order()    

            else:
                rospy.loginfo("[ROBOT_CONTROLLER] Order queue is empty. Waiting for new orders.")
            rate.sleep()

    def handle_new_order(self, msg):
        if msg.order_id in self.completed_orders:
            return

        # Check if order is already in the queue based on order_id
        if any(order.order_id == msg.order_id for order in self.order_queue):
            return

        # Add the order to the queue
        self.order_queue.append(msg)
        self.queue_updated = True  # Mark the queue as updated
        

    def process_next_order(self):
        if not self.order_queue:
            rospy.loginfo("[ROBOT_CONTROLLER] No orders in the queue to process.")
            return

        # Get the next order from the queue
        self.current_order = self.order_queue.pop(0)
        rospy.loginfo(f"[ROBOT_CONTROLLER] Processing order with ID: {self.current_order.order_id}")

        # Determine start and destination locations
        start_location = (
            self.position_manager.get_position()
            if self.current_order.start_location == "current"
            else self.predefined_points.get(self.current_order.start_location)
        )
        destination = self.predefined_points.get(self.current_order.destination)

        if start_location is None or destination is None:
            rospy.logerr(f"[ROBOT_CONTROLLER] Invalid start or destination location for order {self.current_order.order_id}.")
            return

        # Add destination marker
        marker = self.marker_manager.add_destination_marker(
            self.current_order.destination, destination, self.marker_pub
        )

        # Navigate to destination
        self.navigation_manager.navigate_to_destination(
            start_location, destination, self.position_pub, self.path_pub, marker
        )

        # Mark the order as completed
        rospy.loginfo(f"[ROBOT_CONTROLLER] Order {self.current_order.order_id} completed.")
        self.completed_orders.add(self.current_order.order_id)

        for order in self.order_queue[:]:
            if order.order_id in self.completed_orders:
                self.order_queue.remove(order)

    def handle_get_orders_list(self, req):
        response = GetOrdersListResponse()
        response.pending_orders = [order for order in self.order_queue]
        response.completed_orders = [f"order_id: {order_id}" for order_id in self.completed_orders]
        return response


if __name__ == '__main__':
    rospy.loginfo("[ROBOT_CONTROLLER] Starting Robot Controller node.")
    controller = RobotController()
    rospy.spin()
