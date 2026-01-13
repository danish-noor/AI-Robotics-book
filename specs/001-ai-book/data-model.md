# Data Model: AI Robotics Book Curriculum

## Key Entities

### Learning Modules
- **Attributes**:
  - id: string (unique identifier like "module-1")
  - title: string (display title)
  - description: string (brief overview)
  - objectives: array of strings (learning objectives)
  - prerequisites: array of strings (required knowledge)
  - duration: number (estimated hours)
  - content_path: string (path to documentation)
- **Relationships**: Contains multiple Exercises and Assessments
- **Validation**: Title and content_path are required

### Exercises
- **Attributes**:
  - id: string (unique identifier)
  - module_id: string (references Learning Module)
  - title: string
  - description: string
  - difficulty: enum (beginner, intermediate, advanced)
  - estimated_time: number (minutes)
  - files: array of strings (required files/paths)
- **Relationships**: Belongs to one Learning Module
- **Validation**: All attributes required

### Assessments
- **Attributes**:
  - id: string (unique identifier)
  - module_id: string (references Learning Module)
  - title: string
  - type: enum (quiz, practical, project)
  - passing_score: number (percentage)
  - questions: array of objects (for quizzes)
  - criteria: array of strings (for practical assessments)
- **Relationships**: Belongs to one Learning Module
- **Validation**: Required attributes ensure proper evaluation

### Robot Models
- **Attributes**:
  - id: string (unique identifier)
  - name: string (display name)
  - urdf_path: string (path to URDF file)
  - description: string
  - sensors: array of strings (LiDAR, IMU, etc.)
  - actuators: array of strings (joint types)
- **Validation**: URDF path must be valid and accessible

### Simulation Environments
- **Attributes**:
  - id: string (unique identifier)
  - name: string
  - world_file: string (path to Gazebo world file)
  - description: string
  - complexity: enum (simple, moderate, complex)
  - objectives: array of strings (what this environment tests)
- **Validation**: World file must be valid Gazebo format

### AI Integration Points
- **Attributes**:
  - id: string (unique identifier)
  - name: string
  - type: enum (perception, planning, control, interaction)
  - description: string
  - input_types: array of strings (data types accepted)
  - output_types: array of strings (data types produced)
- **Validation**: Type must be one of the defined enum values

## State Transitions

### Module Progression
- **Not Started** → **In Progress** (when student begins module)
- **In Progress** → **Completed** (when student completes all exercises and passes assessment)
- **Completed** → **Reviewed** (when student reviews material)

### Exercise Status
- **Available** → **Started** (when student begins exercise)
- **Started** → **Completed** (when student finishes exercise)
- **Completed** → **Graded** (when exercise is evaluated)

## Relationships

- Learning Modules contain multiple Exercises and Assessments
- Exercises and Assessments belong to exactly one Learning Module
- Robot Models may be used across multiple modules
- Simulation Environments are referenced by specific exercises
- AI Integration Points connect to specific implementation examples