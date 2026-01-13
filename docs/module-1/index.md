# ROS 2 Fundamentals

## Overview

ROS 2 (Robot Operating System 2) serves as the nervous system of modern robotics applications. This module covers the foundational concepts necessary to build distributed robotic systems using ROS 2 Humble Hawksbill, the long-term support version appropriate for educational curriculum.

## Learning Objectives

By the end of this module, students will be able to:
- Understand the ROS 2 architecture and its components
- Create and connect nodes using rclpy (Python client library)
- Implement topics for asynchronous communication
- Use services for synchronous request-response patterns
- Define and work with custom message types
- Model robot kinematics using URDF (Unified Robot Description Format)

## Prerequisites

- Basic Python programming knowledge
- Understanding of object-oriented programming concepts
- Familiarity with command-line interfaces

## Module Structure

### 1. Introduction to ROS 2
- History and evolution from ROS 1
- ROS 2 architecture and middleware
- Installation and environment setup
- ROS 2 workspace creation

### 2. Nodes and Communication
- Creating ROS 2 nodes with rclpy
- Publisher-subscriber pattern with Topics
- Request-response pattern with Services
- Actions for long-running tasks

#### Creating a Basic ROS 2 Node

Here's an example of a basic ROS 2 node using rclpy:

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### Publisher-Subscriber Implementation

Here's an example of a publisher and subscriber pair:

**Publisher Code:**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):

    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    talker = Talker()

    try:
        rclpy.spin(talker)
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Subscriber Code:**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):

    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    listener = Listener()

    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        pass
    finally:
        listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Messages and Services
- Standard message types
- Creating custom message definitions
- Service interfaces
- Parameter management

#### Service Implementation

Here's an example of creating and using services in ROS 2:

**Service Server Code:**
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Service Client Code:**
```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class MinimalClient(Node):

    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClient()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        f'Result of add_two_ints: {response.sum}')

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 4. URDF and Robot Modeling
- Robot kinematics and joint types
- Link and joint definitions
- Visual and collision properties
- Static and dynamic transforms (TF)

#### URDF Examples and Explanation

Here's an example of a simple URDF robot model:

**Simple Robot URDF (my_robot.urdf):**
```xml
<?xml version="1.0"?>
<robot name="my_robot">

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
      <origin rpy="1.5707963267948966 0 0"/>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
      <origin rpy="1.5707963267948966 0 0"/>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.004"/>
    </inertial>
  </link>

  <!-- Joint connecting base to right wheel -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.3 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Left wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
      <origin rpy="1.5707963267948966 0 0"/>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
      <origin rpy="1.5707963267948966 0 0"/>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.004"/>
    </inertial>
  </link>

  <!-- Joint connecting base to left wheel -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.3 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

</robot>
```

**Python Script to Publish TF Transforms for URDF:**
```python
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class StatePublisher(Node):

    def __init__(self):
        super().__init__('state_publisher')
        self.br = TransformBroadcaster(self)
        self.nodeName = self.get_name()
        self.get_logger().info(f'{self.nodeName}: starting')

    def publish_transforms(self, wheel_l, wheel_r, alpha):
        t = TransformStamped()

        # Base frame
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = StatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 5. Practical Exercises

#### Exercise 1: Creating a Simple Robot Publisher/Subscriber

Create a ROS 2 package with two nodes:
1. A publisher node that publishes velocity commands to control a robot
2. A subscriber node that listens to sensor data from the robot
3. Use custom message types for communication

**Steps:**
1. Create a new ROS 2 package: `ros2 pkg create --build-type ament_python robot_control_pkg`
2. Implement a publisher node that sends velocity commands every 500ms
3. Implement a subscriber node that receives and logs sensor data
4. Test the communication between nodes using `ros2 run`

#### Exercise 2: Implementing a Service for Robot Control

Create a service server and client for robot control:
1. Define a custom service file for robot movement commands
2. Implement a service server that controls robot movement
3. Create a client that sends movement requests to the server
4. Test the service using command-line tools

#### Exercise 3: Building a URDF Model of a Simple Robot

Create a complete URDF model for a differential drive robot:
1. Define the base link with visual and collision properties
2. Add wheels with appropriate joints and geometry
3. Include proper inertial properties for physics simulation
4. Test the URDF model using `check_urdf` and visualization tools

#### Exercise 4: Visualizing the Robot in RViz

Set up RViz to visualize your robot model:
1. Launch a robot state publisher node
2. Configure RViz to display the robot model
3. Add TF transforms to show robot pose
4. Verify that all links and joints are displayed correctly

## Technology Stack

- **ROS 2 Version**: Humble Hawksbill (LTS)
- **Client Library**: rclpy (Python)
- **Build System**: colcon
- **Visualization**: RViz2
- **Simulation**: Gazebo Garden (integrated with ROS 2)

## Assessment Criteria

Students will demonstrate proficiency by:
1. Creating a ROS 2 package with multiple interconnected nodes
2. Implementing custom message types for robot communication
3. Building a complete URDF model with proper kinematic chains
4. Successfully visualizing and controlling the robot model

This module establishes the foundation for all subsequent modules in the curriculum, as all advanced robotics concepts build upon the communication and architecture patterns established in ROS 2.