# Research: AI Robotics Book Curriculum

## Decision: Technology Stack Selection
**Rationale**: Selected ROS 2 Humble Hawksbill as the LTS version appropriate for educational curriculum, ensuring long-term support and stability during the course duration. Gazebo Garden chosen as primary simulation platform due to extensive ROS 2 integration and industry adoption.

## Decision: Documentation Platform
**Rationale**: Docusaurus Classic theme selected for educational content due to its clean, readable layout and good navigation capabilities that are suitable for learning materials and tutorials.

## Decision: Simulation Platform Priority
**Rationale**: Gazebo selected as the primary simulation platform because it's the industry standard for robotics simulation with extensive ROS integration, comprehensive documentation, and strong community support beneficial for educational purposes.

## Decision: LLM Integration Approach
**Rationale**: Practical integration approach chosen focusing on open-source LLMs to provide students with hands-on experience they can replicate without licensing restrictions, while still learning core concepts applicable to commercial solutions.

## Decision: Target Audience Level
**Rationale**: Advanced Beginners level selected to provide foundational concepts while assuming basic programming knowledge, allowing for progression to advanced topics without overwhelming newcomers.

## Alternatives Considered

### ROS 2 Versions
- **Humble Hawksbill (Selected)**: LTS version, stable, long-term support
- **Jazzy Jalisco**: Newer features but less stable for learning
- **Iron Irwini**: Balanced option but shorter support cycle

### Simulation Platforms
- **Gazebo (Selected)**: Industry standard, extensive ROS integration
- **Unity**: Advanced graphics but less robotics-focused
- **Both equally**: Would require more content but no clear pedagogical advantage

### Documentation Themes
- **Classic (Selected)**: Clean layout suitable for educational content
- **Doc-heavy**: More features but potentially overwhelming for beginners
- **Custom**: Would require additional development time

### LLM Approaches
- **Practical Integration (Selected)**: Hands-on experience with open-source models
- **Theoretical Overview**: Less practical value for students
- **Commercial Solutions**: Licensing restrictions would limit student access