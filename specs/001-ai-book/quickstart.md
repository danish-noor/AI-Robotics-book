# Quickstart Guide: AI Robotics Book Curriculum

## Prerequisites

Before starting the curriculum, ensure you have the following installed:

### System Requirements
- Ubuntu 22.04 LTS (recommended) or compatible Linux distribution
- At least 8GB RAM (16GB recommended for simulation)
- Multi-core processor (4+ cores recommended)
- NVIDIA GPU with CUDA support (optional but recommended for Isaac modules)
- At least 50GB free disk space

### Software Requirements
1. **ROS 2 Humble Hawksbill**
   ```bash
   # Setup locale
   sudo locale-gen en_US.UTF-8
   sudo update-locale LANG=en_US.UTF-8

   # Setup sources
   sudo apt update && sudo apt install -y curl gnupg lsb-release
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

   # Install ROS 2
   sudo apt update
   sudo apt install -y ros-humble-desktop
   sudo apt install -y python3-colcon-common-extensions
   ```

2. **Gazebo Garden**
   ```bash
   sudo apt install -y ros-humble-gazebo-*
   ```

3. **Python Environment**
   ```bash
   sudo apt install -y python3-pip python3-venv
   python3 -m venv ~/ros2_env
   source ~/ros2_env/bin/activate
   pip install --upgrade pip
   ```

4. **Docusaurus for Documentation**
   ```bash
   npm install -g create-docusaurus@latest
   ```

## Setting Up Your Workspace

1. **Create ROS 2 Workspace**
   ```bash
   mkdir -p ~/ai_robotics_ws/src
   cd ~/ai_robotics_ws
   colcon build
   source install/setup.bash
   ```

2. **Clone Curriculum Repository**
   ```bash
   cd ~/ai_robotics_ws/src
   git clone [repository-url]  # Replace with actual repository URL
   ```

3. **Install Additional Dependencies**
   ```bash
   cd ~/ai_robotics_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build
   source install/setup.bash
   ```

## Starting with Module 1: ROS 2 Fundamentals

1. **Verify ROS 2 Installation**
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 topic list
   ```

2. **Run Your First Example**
   ```bash
   ros2 run demo_nodes_cpp talker
   # In a new terminal:
   ros2 run demo_nodes_py listener
   ```

3. **Access Module Documentation**
   ```bash
   cd ~/ai_robotics_ws/src/docs
   npm run start  # Starts Docusaurus development server
   ```

## Running Simulations

1. **Launch Gazebo with Sample Robot**
   ```bash
   ros2 launch gazebo_ros empty_world.launch.py
   ```

2. **Spawn a Robot Model**
   ```bash
   ros2 run gazebo_ros spawn_entity.py -entity my_robot -file path/to/robot.urdf
   ```

## Development Workflow

1. **Always source your environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ai_robotics_ws/install/setup.bash
   ```

2. **Build your workspace after changes**:
   ```bash
   cd ~/ai_robotics_ws
   colcon build --packages-select [package-name]
   source install/setup.bash
   ```

3. **Run tests**:
   ```bash
   colcon test --packages-select [package-name]
   colcon test-result --all
   ```

## Troubleshooting

### Common Issues

**ROS 2 Commands Not Found**
- Ensure you've sourced the ROS 2 environment: `source /opt/ros/humble/setup.bash`

**Gazebo Won't Start**
- Check if you have proper graphics support: `glxinfo | grep "OpenGL renderer"`
- For headless systems, install: `sudo apt install -y xvfb`

**Python Package Issues**
- Use virtual environment: `source ~/ros2_env/bin/activate`
- Install packages in the virtual environment rather than globally

## Next Steps

After completing the setup:
1. Go through the documentation in `docs/intro.md` to understand Physical AI concepts
2. Start with Module 1 by reading `docs/module-1/index.md`
3. Follow the practical exercises in sequence
4. Join the curriculum community for support and discussions