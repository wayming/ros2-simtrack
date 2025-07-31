# ROS2 Simulation and Tracking

A collection of ROS2 workspaces for TurtleBot3 simulation, control, and monitoring.

## Workspaces

### 1. Mini Spawn Demo
Located in `mini_spawn_demo_ws/`, this workspace contains demo code for spawning TurtleBot3 models.

### 2. Turtle Autonomous Patrol
Located in `turtle_autonomous_patrol_ws/`, this workspace implements autonomous patrol functionality for TurtleBot3, including:
- TurtleBot3 core functionalities
- Navigation and path planning
- Dynamixel SDK integration
- Cartographer SLAM integration

### 3. Turtle Commander
Located in `turtle_commander_ws/`, this workspace provides command and control capabilities for the TurtleBot3.

### 4. Turtle Monitor
Located in `turtle_monitor_ws/`, this workspace contains monitoring functionalities for the TurtleBot3, including:
- TurtleBot3 state monitoring
- Dynamixel SDK integration
- Simulation support

### 5. Waffle TF2 Navigation
Located in `waffle_tf2_nav_ws/`, this workspace focuses on TF2-based navigation for the TurtleBot3 Waffle model.

## Prerequisites

- ROS2
- TurtleBot3 packages
- Gazebo simulator
- Navigation2 (Nav2)
- Cartographer

## Building and Running

Each workspace contains its own build and run scripts:
- `boot.sh` - Initializes the workspace environment
- `build.sh` - Builds the workspace packages
- `run.sh` - Executes the workspace applications

## Docker Support

Each workspace includes Dockerfile for containerized deployment and testing.

## License

[Add your license information here]

## Contributors

- wayming (Owner)
