#!/usr/bin/env python3
import rospy
from delivery_robot2.msg import DeliveryOrder
from delivery_robot2.srv import AddOrder, AddOrderResponse, TriggerTestOrders, TriggerTestOrdersResponse

class OrderManager:
    def __init__(self):
        rospy.init_node('order_manager')
        rospy.loginfo("[ORDER MANAGER] Order Manager node initialized.")
        self.orders = {}

        # Load predefined points from ROS parameters
        self.predefined_points = rospy.get_param('predefined_points')
        
        # Publisher for new orders
        self.order_pub = rospy.Publisher('/delivery_order', DeliveryOrder, queue_size=10)
        rospy.loginfo("[ORDER MANAGER] Publisher for /delivery_order created.")

        # Services
        
        self.add_order_service = rospy.Service('/add_order', AddOrder, self.handle_add_order)
        rospy.loginfo("[ORDER MANAGER] Service /add_order created.")

        self.trigger_test_orders_service = rospy.Service('/trigger_test_orders', TriggerTestOrders, self.handle_trigger_test_orders)
        rospy.loginfo("[ORDER MANAGER] Service /trigger_test_orders created.")


    def resolve_location(self, location):
        if location == "current":
            return "current"
        return self.predefined_points.get(location, None)


    def handle_add_order(self, req):
        #rospy.loginfo(f"Received request to add order: {req}")
        if req.order_id in self.orders:
            rospy.logwarn(f"[ORDER MANAGER] Order {req.order_id} already exists.")
            return AddOrderResponse(False, f"[ORDER MANAGER] Order {req.order_id} already exists.")

        start_location = self.resolve_location(req.start_location)
        destination = self.resolve_location(req.destination)


        if start_location is None or destination is None:
            rospy.logwarn("[ORDER MANAGER] Invalid start or destination location.")
            return AddOrderResponse(False, "[ORDER MANAGER] Invalid start or destination location.")
        
        if req.priority not in ["low","medium","high"]:
            rospy.logwarn("[ORDER MANAGER] Invalid priority value.")
            return AddOrderResponse(False, "[ORDER MANAGER] Invalid priority value.")

        # Create and store the order
        order = DeliveryOrder(
            order_id=req.order_id,
            start_location=req.start_location,
            destination=req.destination,
            priority=req.priority
        )
        self.orders[req.order_id] = order
        
        return AddOrderResponse(True, f"[ORDER MANAGER] Order {req.order_id} successfully added.")

    def handle_trigger_test_orders(self, req):

        self.test_orders = rospy.get_param('test_orders', [])
        for order_data in self.test_orders:
            order = DeliveryOrder(
                order_id=order_data['order_id'],
                start_location=order_data['start_location'],
                destination=order_data['destination'],
                priority=order_data['priority']
            )
            self.orders[order.order_id] = order
        

        self.prioritize_orders()

        for order_id, order in self.orders.items():

            if order.priority not in ["low","medium","high"]:
                rospy.logwarn("[ORDER MANAGER] Invalid priority value.")
                return AddOrderResponse(False, "[ORDER MANAGER] Invalid priority value.")
            self.order_pub.publish(order)
            rospy.loginfo(f"[ORDER MANAGER] Published order {order.order_id}.")
        return TriggerTestOrdersResponse(True, "[ORDER MANAGER] Test orders triggered successfully.")

    def prioritize_orders(self):
        # Sort orders by priority (e.g., "high", "medium", "low")
        priority_map = {"high": 1, "medium": 2, "low": 3}
        self.orders = dict(
            sorted(
                self.orders.items(),
                key=lambda item: priority_map.get(item[1].priority, 4)  # Default to lowest priority
            )
        )
        

    def publish_order(self):
        rate = rospy.Rate(1)  # 2 Hz
        while not rospy.is_shutdown():
            for order_id, order in self.orders.items():
                self.order_pub.publish(order)
            rate.sleep()

if __name__ == '__main__':
    rospy.loginfo("[ORDER MANAGER] Starting Order Manager node.")
    manager = OrderManager()
    manager.publish_order()  