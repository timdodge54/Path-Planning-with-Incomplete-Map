# Final Project

- Ideas
- reinforcement learning
  - state
  - ultrasonic sensor
- planner
  - takes in map at some frequency

# Installation Instructions
- [ ] install ros2 foxy
- [ ] run the following command from root of ws `rosdep install --from-paths src -y --ignore-src`
- [ ] install catkin_pkg `pip install catkin_pkg`
- [ ] source ros2 `source ros2`
- [ ] run colcon `colcon build --symlink install`
- [ ] source local install `. install/local_setup.bash`
- [ ] run launch file `ros2 launch reinforcement_planning launch_nodes.launch.py` 
