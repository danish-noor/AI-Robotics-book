# Simulation & Digital Twins

## Overview

Digital twins and simulation environments are crucial for robotics development, allowing for safe, repeatable testing and rapid iteration without the risks and costs associated with physical hardware. This module focuses on Gazebo as the primary simulation platform, with considerations for Unity integration where appropriate.

## Learning Objectives

By the end of this module, students will be able to:
- Set up and configure Gazebo simulation environments
- Create realistic physics models for robots and environments
- Implement sensor simulation including LiDAR and IMU
- Perform sensor fusion in simulated environments
- Validate robot behaviors in simulation before hardware deployment

## Prerequisites

- Completion of Module 1 (ROS 2 fundamentals)
- Understanding of basic physics concepts
- Familiarity with URDF robot modeling

## Module Structure

### 1. Gazebo Fundamentals
- Gazebo architecture and components
- World creation and environment modeling
- Physics engine configuration (ODE, Bullet, DART)
- Integration with ROS 2 via Gazebo ROS packages

#### Gazebo Installation and Setup

**Installing Gazebo Garden:**
```bash
# Add Gazebo repository
curl -sSL http://get.gazebosim.org | sh

# Install Gazebo Garden
sudo apt-get install gz-garden
```

**ROS 2 Gazebo Integration:**
```bash
# Install Gazebo ROS packages for Humble
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros-control
```

### 2. Robot Simulation
- Loading URDF models into Gazebo
- Creating robot spawn configurations
- Joint control and actuator simulation
- Implementing robot state publishers

#### Loading URDF Models in Gazebo

**Launch file for spawning robot in Gazebo:**
```xml
<launch>
  <!-- Robot State Publisher -->
  <node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher">
    <param name="robot_description" value="$(var robot_description_file)"/>
  </node>

  <!-- Spawn Robot in Gazebo -->
  <node pkg="gazebo_ros" exec="spawn_entity.py" name="spawn_robot" args="-entity my_robot -file $(find-pkg-share my_robot_description)/urdf/my_robot.urdf"/>

  <!-- Launch Gazebo -->
  <include file="$(find-pkg-share gazebo_ros)/launch/gazebo.launch.py"/>
</launch>
```

### 3. Sensor Simulation
- LiDAR sensor modeling and configuration
- IMU sensor simulation
- Camera and depth sensor integration
- Sensor noise and accuracy modeling

#### LiDAR Sensor Configuration

**Example LiDAR sensor in URDF:**
```xml
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.05"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>

<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0 0 0.2" rpy="0 0 0"/>
</joint>

<gazebo reference="lidar_link">
  <sensor name="lidar_sensor" type="ray">
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/lidar</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

#### IMU Sensor Configuration

**Example IMU sensor in URDF:**
```xml
<link name="imu_link">
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001"/>
  </inertial>
</link>

<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>

<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <topic>imu/data</topic>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/imu</namespace>
        <remapping>~/out:=data</remapping>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
  </sensor>
</gazebo>
```

### 4. Physics and Environment Modeling
- Creating realistic environments
- Material properties and surface interactions
- Dynamic object simulation
- Environmental effects (lighting, weather)

#### Gazebo World Creation

**Example world file (my_world.sdf):**
```xml
<sdf version="1.7">
  <world name="my_world">
    <!-- Include outdoor environment -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Custom objects -->
    <model name="table">
      <pose>2 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1.0 0.5 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.0 0.5 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.6 0.2 1</ambient>
            <diffuse>0.8 0.6 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>1.0</iyy>
            <iyz>0.0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Physics parameters -->
    <physics name="ode" type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
  </world>
</sdf>
```

### 5. Sensor Fusion and Navigation
- Combining LiDAR and IMU data
- Simulated navigation stack testing
- SLAM algorithm validation in simulation
- Path planning and obstacle avoidance

#### Sensor Fusion Example

**Robot Localization using EKF:**
```yaml
# ekf_config.yaml
frequency: 50

# Frame definitions
map_frame: map
odom_frame: odom
base_link_frame: base_link
world_frame: odom

# Enable dynamic process noise covariance
enable_dynamic_process_noise_covariance: true

# Publishers
publish_tf: true
publish_acceleration: false

# Sensor configuration
odom0: /odom
odom0_config: [true, true, false,
                false, false, true,
                false, false, false,
                false, false, false,
                false, false, false]

imu0: /imu/data
imu0_config: [false, false, false,
              true, true, true,
              false, false, false,
              true, true, true,
              false, false, false]
```

### 6. Simulation-to-Reality Transfer
- Identifying sim-to-real gaps
- Techniques for improving transferability
- Validation strategies for hardware deployment
- Performance comparison between simulation and reality

## Technology Stack

- **Primary Simulator**: Gazebo Garden
- **ROS 2 Integration**: Gazebo ROS packages
- **Physics Engine**: ODE (Open Dynamics Engine)
- **Visualization**: RViz2 with Gazebo GUI
- **Alternative**: Unity 2023.x (for game-engine based simulation)

## Practical Exercises

### Exercise 1: Creating a Basic Gazebo Environment

Create a simple simulation environment with your robot:
1. Create a world file with basic obstacles
2. Spawn your robot model in the environment
3. Verify that sensors are publishing data
4. Test basic movement commands in simulation

### Exercise 2: Sensor Integration and Fusion

Integrate multiple sensors and perform sensor fusion:
1. Add LiDAR and IMU sensors to your robot URDF
2. Configure the robot_localization package for state estimation
3. Verify that sensor data is being fused correctly
4. Compare estimated pose with ground truth from simulation

### Exercise 3: Navigation in Simulation

Set up navigation stack in simulation:
1. Configure costmap parameters for your robot
2. Set up global and local planners
3. Test navigation to multiple goals in simulation
4. Evaluate navigation performance metrics

## Assessment Criteria

Students will demonstrate proficiency by:
1. Creating a complete simulation environment with obstacles
2. Implementing sensor fusion algorithms that combine LiDAR and IMU data
3. Successfully navigating a robot through the simulated environment
4. Comparing simulation results with theoretical expectations

This module bridges the gap between theoretical robotics concepts and practical implementation, providing a safe environment for testing complex behaviors before hardware deployment.