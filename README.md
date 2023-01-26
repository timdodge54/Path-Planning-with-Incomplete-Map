# Final Project

# Installation Instructions

- [ ] install [ros2 foxy](https://docs.ros.org/en/foxy/Installation.html)
- [ ] run the following command from root of ws `rosdep install --from-paths src -y --ignore-src`
- [ ] install catkin_pkg `pip install catkin_pkg`
- [ ] source ros2 installation
- [ ] run colcon `colcon build --symlink-install`
- [ ] source local install `. install/local_setup.bash`
- [ ] run launch file for gazebo, robot state publisher and static frame publisher `ros2 launch reinforcement_planning gazebo_robot.launch.py`
- [ ] wait for gazebo to fully launch
- [ ] run launch file for path planner and path saver `ros2 launch reinforcement_planning nav_launch.launch.py`
- [ ] set nav goal in rviz
- [ ] run launch file for reinforcement learning `ros2 launch reinforcement_planning interface.launch.py`

# File Structure

- reinforcement_planning and plan_msg are the main two packages created by us
- There were modifications to launch files and model files in turtlebot3_gazebo and turtlebot3_description
  - src contains cpp code
  - scripts contains python ros code
  - ddpg_planning is a nested python package that will contain reinforcment learning python code

# Low Fidelity trained Reinforcement Learning Model following the path
<!-- blank line -->
<figure class="video_container">
  <video controls="true" allowfullscreen="true" poster="video/low.png">
    <source src="video/low.mp4" type="video/mp4">
<!--   <source src="path/to/video.ogg" type="video/ogg"> -->
<!--   <source src="path/to/video.webm" type="video/webm"> -->
  </video>
</figure>
<!-- blank line -->


https://user-images.githubusercontent.com/93363769/214743284-15000bea-fbf8-4d19-a7c6-eb58967b27ce.mp4


# Explination of Code




https://user-images.githubusercontent.com/93363769/214743295-366a1be6-7739-4a08-a292-f436c62a3c15.mp4




<!-- blank line -->
<figure class="video_container">
  <video controls="true" allowfullscreen="true" poster="video/high.png">
    <source src="video/high.mkv" type="video/mkv">
<!--   <source src="path/to/video.ogg" type="video/ogg"> -->
<!--   <source src="path/to/video.webm" type="video/webm"> -->
  </video>
</figure>
<!-- blank line -->
