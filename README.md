# AI Robotics Book Curriculum

Welcome to the comprehensive 4-module course on Physical AI & Humanoid Robotics! This course guides developers from digital AI to embodied intelligence, covering ROS 2, simulation environments, NVIDIA Isaac, and Vision-Language-Action (VLA) models.

## Curriculum Structure

### [Module 1: ROS 2 Fundamentals](docs/module-1/index.md)
- **Duration**: 3 weeks (8-12 hours/week)
- **Focus**: Core ROS 2 architecture, communication patterns, and robot software development
- **Outcome**: Solid foundation in robot operating system concepts and practices

### [Module 2: Simulation & Digital Twins](docs/module-2/index.md)
- **Duration**: 4 weeks (10-15 hours/week)
- **Focus**: Simulation environments, physics modeling, and sim-to-real transfer
- **Outcome**: Expertise in simulation-driven robot development and validation

### [Module 3: NVIDIA Isaac Integration](docs/module-3/index.md)
- **Duration**: 4 weeks (12-15 hours/week)
- **Focus**: GPU-accelerated AI, perception systems, and Isaac platform integration
- **Outcome**: Skills in deploying AI models for real-time robotics applications

### [Module 4: Vision-Language-Action (VLA) Models](docs/module-4/index.md) (Capstone)
- **Duration**: 8 weeks (15-20 hours/week)
- **Focus**: Multimodal AI, human-robot interaction, and complete embodied systems
- **Outcome**: Mastery of advanced VLA systems for humanoid robotics

## Learning Objectives

By completing this course, students will be able to:
- Design and implement modular robot systems using ROS 2 architecture
- Create and validate robot behaviors in simulation environments
- Deploy GPU-accelerated AI models for real-time perception and control
- Build Vision-Language-Action systems that understand natural language commands
- Integrate all components into complete embodied AI systems
- Apply simulation-to-reality transfer techniques for physical robot deployment

## Technical Requirements

### Development Environment
- **OS**: Ubuntu 22.04 LTS
- **GPU**: NVIDIA RTX 4090, RTX 6000 Ada, or A6000 (48GB+ VRAM recommended)
- **CPU**: 16+ cores (Intel i9-13900K or AMD Ryzen 9 7950X)
- **RAM**: 256+ GB (512GB recommended for large model training)
- **Storage**: 4TB+ NVMe SSD RAID 0 for datasets and models

### Software Stack
- **ROS 2**: Humble Hawksbill with Isaac ROS packages
- **Simulation**: Gazebo Garden, Unity 2022.3 LTS
- **AI Frameworks**: PyTorch 2.1+, NVIDIA Isaac, TensorRT 8.6+
- **Development Tools**: Docker, CUDA 12.1+, Isaac Sim

### Hardware for Deployment
- **Edge Platform**: NVIDIA Jetson AGX Orin 64GB
- **Robot Platform**: Isaac-compatible humanoid robot
- **Sensors**: High-resolution cameras, IMU, LIDAR, audio systems

## Getting Started

1. Review the [Project Constitution](.specify/memory/constitution.md) to understand core principles
2. Start with [Module 1: ROS 2 Fundamentals](docs/module-1/index.md)
3. Progress through modules sequentially, building on previous knowledge
4. Use simulation environments early for safe experimentation
5. Complete with the capstone VLA project integrating all concepts

## Deployment

This site is configured for deployment to GitHub Pages. The deployment workflow is set up in `.github/workflows/deploy.yml`.

## Contributing

This course follows Spec-Driven Development principles. Contributions should follow the established patterns:
1. Create specifications before implementation
2. Follow the modular architecture principles
3. Maintain simulation-to-reality transfer capabilities
4. Ensure cross-platform integration
5. Prioritize developer accessibility

---
*This course represents a comprehensive pathway from digital AI to embodied intelligence, following the Physical AI & Humanoid Robotics constitution principles of modular architecture, simulation-to-reality transfer, cross-platform integration, and developer accessibility.*