# Build the project
colcon build

# Wait for the build to finish
wait

# Source the setup.bash file
source install/setup.bash

# Wait for the build to finish
wait

# Launch the mobile robot control
ros2 launch mobile_robot_control mobile_robot_launch.py

