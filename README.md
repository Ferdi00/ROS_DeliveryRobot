## Project Overview

The ROS Delivery Robot project is a robotic delivery order management and control system based on ROS (Robot Operating System). The system is designed to automate and optimize the management of delivery orders, providing efficient, automated, and safe delivery operations with visual monitoring through RViz.

## Features

- **Order Management**: Receives new order requests, validates them, assigns priorities, and publishes them on a ROS topic to be processed by other nodes.
- **Robot Control**: Manages the robot that physically executes the delivery orders, handling navigation to the destination and managing the queue based on priority.
- **Navigation**: Gradually moves the robot from its current position to the specified destination, continuously updating the position and visualizing the path.
- **Visualization**: Provides a visual interface through RViz to monitor the robot's status and path, including visual markers for the robot's position and destination.

## Technologies Used

- **ROS (Robot Operating System)**: The core framework for robot software development.
- **Python**: The primary programming language used for scripting and node development.
- **RViz**: A 3D visualization tool for ROS.
- **ROS Messages and Services**: Custom messages and services for communication between nodes.

## Project Structure

```
delivery_robot2/
├── CMakeLists.txt
├── config/
│   └── params.yaml
├── launch/
│   └── delivery_robot.launch
├── msg/
│   └── DeliveryOrder.msg
├── package.xml
├── ReadMe
├── scripts/
│   ├── order_manager.py
│   ├── robot_controller.py
│   └── robot_helpers/
│       ├── marker.py
│       ├── navigation.py
│       └── position.py
├── srv/
│   ├── AddOrder.srv
│   ├── GetOrdersList.srv
│   └── TriggerTestOrders.srv
└── README.md
```

## Installation

### Prerequisites

- ROS (Robot Operating System) installed on your machine.
- Python 3.x
- Catkin workspace set up.

### Steps

1. **Clone the Repository**:
    ```sh
    git clone https://github.com/yourusername/ROS_DeliveryRobot.git
    cd ROS_DeliveryRobot/delivery_robot2
    ```

2. **Build the Package**:
    ```sh
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

3. **Install Dependencies**:
    Ensure all necessary ROS dependencies are installed:
    ```sh
    rosdep install --from-paths src --ignore-src -r -y
    ```

4. **Launch the System**:
    ```sh
    roslaunch delivery_robot2 delivery_robot.launch
    ```

## Usage

### Order Manager

The order_manager.py node is responsible for managing delivery orders. It receives new orders, validates them, assigns priorities, and publishes them to be processed by the robot controller.

### Robot Controller

The robot_controller.py node controls the robot, managing the queue of orders based on priority and handling navigation to the destination.

### Navigation

The navigation.py script handles the actual navigation of the robot, moving it from its current position to the specified destination and updating the position continuously.

### Visualization

The system uses RViz to provide a visual interface for monitoring the robot's status and path. Visual markers are used to represent the robot's position and destination.

## Configuration

The params.yaml file in the `config` directory contains predefined points and test orders that can be used to simulate the system.

```yaml
predefined_points:
    point1: [-5.0, 3.2]
    point2: [7.0, -2.0]
    point3: [3.0, 6.5]

test_orders:
    - order_id: 1
      start_location: "current"
      destination: "point1"
      priority: "high"
    - order_id: 2
      start_location: "point1"
      destination: "point3"
      priority: "low"
    - order_id: 3
      start_location: "point1"
      destination: "point2"
      priority: "medium"
```

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request for any improvements or bug fixes.

## License

This project is licensed under the MIT License. See the LICENSE file for details.

---

For more information, please refer to the [ROS documentation](http://wiki.ros.org/).