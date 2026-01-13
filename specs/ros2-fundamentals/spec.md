# ROS 2 Fundamentals Module Specification

## Overview
This module introduces students to Robot Operating System 2 (ROS 2), focusing on core concepts essential for developing embodied AI applications. Students will learn ROS 2 architecture, communication patterns, and development tools necessary for building humanoid robotics applications.

## Learning Objectives
By the end of this module, students will be able to:

### Core Competencies
- Explain the ROS 2 architecture and its distributed computing model
- Create and manage ROS 2 packages, nodes, and topics
- Implement services and actions for robot communication
- Use ROS 2 tools for debugging and visualization
- Apply ROS 2 best practices for modular robotics development
- Integrate ROS 2 with other frameworks (NVIDIA Isaac, simulation environments)

### Practical Skills
- Design modular robot systems following the constitution's Modular Architecture principle
- Implement simulation-to-reality transfer concepts (Simulation-to-Reality Transfer principle)
- Create accessible, well-documented code that enables other developers to extend the system (Developer Accessibility principle)
- Establish proper safety protocols and error handling in ROS 2 systems
- Deploy ROS 2 applications to both simulated and physical robotic platforms

### Assessment Outcomes
Students will demonstrate mastery by implementing a complete robot system that:
- Communicates effectively between multiple nodes using topics, services, and actions
- Integrates with simulation environments for testing
- Follows ROS 2 best practices for code organization and documentation
- Implements proper error handling and safety mechanisms

## Prerequisites
Students should have:

### Programming Skills
- Proficiency in Python or C++ (intermediate level)
- Understanding of object-oriented programming concepts
- Experience with package management and dependency resolution

### Technical Background
- Familiarity with Linux command line operations
- Basic understanding of computer networking concepts (TCP/IP, ports)
- Fundamental knowledge of robotics concepts (kinematics, sensors, actuators)
- Understanding of basic linear algebra (vectors, matrices)

### Development Environment
- Access to a Linux-based system (Ubuntu 22.04 LTS recommended)
- Ability to set up development environments and install packages
- Basic experience with version control systems (Git)

### Transition Skills
- Developers coming from digital AI backgrounds should be prepared to learn about:
  - Real-time systems and timing constraints
  - Hardware integration and sensor fusion
  - Physical embodiment challenges vs. pure software systems
  - Safety considerations in physical systems

## Module Structure

### Lesson 1: Introduction to ROS 2 Architecture
- ROS 2 vs ROS 1 differences and improvements
- DDS (Data Distribution Service) overview
- Nodes, packages, and the ROS 2 workspace
- Setting up the development environment

### Lesson 2: Communication Patterns
- Topics and message passing
- Services and synchronous communication
- Actions and asynchronous goal-oriented communication
- Message definitions and custom message types

### Lesson 3: ROS 2 Tools and Visualization
- Using ros2 command-line tools
- rviz2 for visualization
- rqt tools for monitoring and debugging
- rosbag2 for data recording and playback

### Lesson 4: Advanced ROS 2 Concepts
- Parameters and configuration management
- Launch files and system orchestration
- Lifecycle nodes and state management
- Testing and simulation integration

## Hands-on Projects

### Project 1: Robot Publisher/Subscriber System
**Objective**: Implement basic communication patterns between robot components
- Create a sensor node that publishes simulated sensor data (LIDAR, IMU, camera)
- Create an actuator node that subscribes to motor command messages
- Implement a controller node that processes sensor data and generates motor commands
- Use proper message types (sensor_msgs, geometry_msgs) and coordinate frames
- Add debugging capabilities using ROS 2 tools (rqt, ros2 topic echo)

### Project 2: Service-Based Navigation System
**Objective**: Implement synchronous communication for path planning and navigation
- Create a path planning service that accepts goal coordinates and returns a path
- Implement a navigation node that uses the service to plan routes
- Add obstacle detection and path replanning capabilities
- Include error handling for unreachable goals or navigation failures
- Test integration with simulation environment (Gazebo/Unity)

### Project 3: Action-Based Manipulation System
**Objective**: Implement asynchronous goal-oriented communication for robotic manipulation
- Create an action server for controlling a robotic arm
- Implement action client that sends manipulation goals (pick, place, move)
- Handle action feedback, result, and preemption
- Include safety checks and error recovery mechanisms
- Integrate with perception system for object detection and localization

### Project 4: Integrated Robot System
**Objective**: Combine all communication patterns in a complete robot application
- Integrate all previous projects into a unified system
- Implement launch files for coordinated system startup
- Add parameter management for system configuration
- Create visualization tools to monitor system state
- Test system in both simulation and physical robot environments (where available)

### Mini-Exercises Throughout Module
- **Exercise 1**: Create your first ROS 2 package and node
- **Exercise 2**: Implement custom message types for robot-specific data
- **Exercise 3**: Use rosbag2 to record and replay sensor data
- **Exercise 4**: Debug communication issues using ROS 2 introspection tools
- **Exercise 5**: Implement lifecycle nodes for resource management

## Assessment Criteria
- Successfully implement all four hands-on projects
- Demonstrate understanding of ROS 2 communication patterns
- Create well-documented and modular ROS 2 packages
- Pass practical evaluations on debugging and troubleshooting

## Technical Requirements

### Software Stack
- **ROS 2**: Humble Hawksbill (LTS version) - primary development framework
- **Operating System**: Ubuntu 22.04 LTS (recommended) or equivalent containerized environment
- **Development Tools**:
  - Build system: colcon (ROS 2 build tool)
  - IDE support: VS Code with ROS 2 extensions or CLion
  - Version control: Git with appropriate .gitignore files
- **Simulation Environment**: Gazebo Garden or Unity 2023.x (for later integration)
- **Visualization**: RViz2 for 3D visualization and RQt for GUI tools

### Core Dependencies
- **Programming Languages**: Python 3.10+ or C++17
- **DDS Implementation**: Fast DDS (default) or Cyclone DDS
- **Core Packages**:
  - rclcpp and rclpy (ROS 2 client libraries)
  - std_msgs, geometry_msgs, sensor_msgs (standard message types)
  - nav_msgs, trajectory_msgs (navigation and trajectory messages)
  - tf2 (transform library for coordinate frame management)
  - rosbag2 (data recording and playback)
  - launch (system orchestration)

### Hardware Requirements
- **Development Machine**:
  - 8+ GB RAM (16+ GB recommended)
  - Multi-core processor (4+ cores recommended)
  - 50+ GB free disk space for development environment
- **Robot Hardware** (for physical testing):
  - ROS 2 compatible robot platform (e.g., TurtleBot 4, Clearpath platforms)
  - Network connectivity for distributed computing
  - Safety equipment and controlled testing environment

### Setup Procedures

#### Option 1: Native Installation (Ubuntu 22.04)
1. Update system packages: `sudo apt update`
2. Set up ROS 2 apt repository: `sudo apt install software-properties-common`
3. Add ROS 2 GPG key and repository
4. Install ROS 2 Humble Desktop: `sudo apt install ros-humble-desktop`
5. Install development tools: `sudo apt install python3-colcon-common-extensions`
6. Source the ROS 2 environment: `source /opt/ros/humble/setup.bash`

#### Option 2: Containerized Environment (Docker)
1. Install Docker and Docker Compose
2. Pull the official ROS 2 Humble development image
3. Configure volume mounts for workspace persistence
4. Set up X11 forwarding for GUI applications (RViz2, RQt)

#### Option 3: Development Container (VS Code)
1. Install VS Code and Remote Development extension pack
2. Create .devcontainer configuration for ROS 2 development
3. Configure the container with all necessary dependencies
4. Mount workspace directory for persistent development

### Verification Steps
1. Test basic ROS 2 installation: `ros2 topic list`
2. Verify Python/C++ client libraries: `python3 -c "import rclpy"`
3. Test visualization tools: `rviz2` and `rqt`
4. Run basic publisher/subscriber tutorial
5. Validate build system: `colcon build` on a sample workspace

### Troubleshooting Guide
- Common installation issues and solutions
- Network configuration for multi-machine setups
- Permission issues with serial devices
- Performance optimization for simulation environments

## Integration Points
This module serves as the foundation for subsequent modules:
- Connects to Gazebo/Unity simulation environments (Module 2)
- Integrates with NVIDIA Isaac for perception and control (Module 3)
- Enables Vision-Language-Action implementations (Module 4)

## Resources
- Official ROS 2 documentation and tutorials
- Sample code repositories
- Video lectures and demonstrations
- Troubleshooting guide for common issues

## Timeline
- Estimated duration: 2-3 weeks (8-12 hours per week)
- Format: 40% theory, 60% hands-on practice
- Assessment: Weekly checkpoints with final project evaluation

## Success Metrics
- Students can independently create ROS 2 packages
- Ability to debug communication issues
- Understanding of best practices for robot software architecture
- Preparation for advanced robotics frameworks in subsequent modules