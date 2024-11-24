# Walker Bot Simulation

This repository contains a ROS 2 package implementing a walker robot using the State Design Pattern. The robot alternates between moving forward and turning to avoid obstacles based on sensor input.

## Overview

The walker robot operates using the following logic:

- **Forward State**: The robot moves forward until an obstacle is detected.
- **Clockwise/Anticlockwise States**: If an obstacle is detected, the robot alternates between turning clockwise and anticlockwise until the path is clear.

The package integrates:

- ROS 2 Humble
- Gazebo for simulation
- `sensor_msgs`, `geometry_msgs`, and `nav_msgs` for communication
- A state machine design for control logic

## Features

- Simulation of the walker robot in a custom Gazebo world.
- Obstacle avoidance using laser scan data.
- State transitions based on proximity sensing.
- Bag file recording and playback for debugging and analysis.

## Assumptions and Dependencies

- **ROS 2 Humble**: Ensure you have ROS 2 Humble installed on your system.
- **Gazebo**: Gazebo should be properly set up to simulate the robot.
- **TurtleBot3 Model**: The walker uses the TurtleBot3 burger model.

## Environment Setup

Before running the simulation, set up your environment as follows:

1. Source the ROS 2 Humble setup file:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Set the TurtleBot3 model:
   ```bash
   export TURTLEBOT3_MODEL=burger
   ```

3. Update the Gazebo model path:
   ```bash
   if [[ ":$GAZEBO_MODEL_PATH:" != *":/opt/ros/humble/share/turtlebot3_gazebo/models:"* ]]; then
     export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH
   fi
   ```

4. To make these settings permanent, add them to your `~/.bashrc`:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
   echo 'if [[ ":$GAZEBO_MODEL_PATH:" != *":/opt/ros/humble/share/turtlebot3_gazebo/models:"* ]]; then export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH; fi' >> ~/.bashrc
   source ~/.bashrc
   ```

## Build Instructions

1. Clone the repository:
   ```bash
   git clone https://github.com/sarangnair1998/my_gazebo_tutorials.git
   cd my_gazebo_tutorials
   ```

2. Build the package using colcon:
   ```bash
   colcon build --packages-select walker
   ```

3. Source the build overlay:
   ```bash
   source install/setup.bash
   ```

## Running the Simulation

Launch the simulation with:
```bash
ros2 launch walker custom_world.launch.py
```

Observe the robot behavior in Gazebo. It should move forward, detect obstacles, and turn to avoid collisions.

## Bag File Recording and Playback

### Recording Bag Files

The `custom_world.launch.py` file includes an argument to enable or disable bag file recording:

- To enable recording:
  ```bash
  ros2 launch walker custom_world.launch.py record_bag:=true
  ```
  Bag files will be saved in the `~/.ros` directory.

- To disable recording:
  ```bash
  ros2 launch walker custom_world.launch.py record_bag:=false
  ```

### Inspecting Bag Files

To inspect the contents of a recorded bag file:
```bash
ros2 bag info <bag_file_directory>
```

### Playing Back Bag Files

- Ensure Gazebo is not running.
- Play back the recorded bag file:
  ```bash
  ros2 bag play <bag_file_directory>
  ```

## Additional Notes

- **Clang-Tidy and Cpplint**: The code follows best practices and has been linted using clang-tidy and cpplint. The results can be found in the `results` directory.
- **License**: This package is licensed under the MIT License. Refer to the LICENSE file for details.

## Troubleshooting

- If `colcon build` fails, ensure that all dependencies are installed and sourced correctly.
- Verify your Gazebo installation and environment variables if the simulation doesn't start.
