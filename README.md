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

# File Structure
- reinforcement_planning is the only package that is created by us
- within reinforcement_planning 
	- src contains cpp code
	- scripts contains python ros code
	- reinforcement_planning is a nested python package that will contain reinforcment learning python code
## Ros Node Sturcture
- All ros node inherit from a ros node base class
	- from python this is rclpy.node.Node
	- from rclcpp::Node
- a Node typical ros interfaces fall under 2 main categories each with two subcategories
	- The first category falls within the msg type
		- messages travel across "topics"
		- Publishing
			- a node uses a publisher to publish a msg across this topic
			- publishers do not care if anyone else is publishing on a topic or if anyone is picking the msg that is publishing
		- Subscriber
			- a node uses a subsribers to recieve all msgs across a topic
			- a subcriber can subsribe to a topic with nothing being published or a topic that multiple nodes are publishing to
	- The second category falls within the srv type
		- services also travel on topics
		- srv are messages that have two fields
			- request the information that is needed for the callback function (explaind in the next few lines)
			- response the data that will be sent back to the client
		- Server
			- when a server is queried it uses a callback which reponds to the query
			- a callback is a function that is triggered by a server 
			- the parameter field of the callback function should be the request of the srv message
		- Client 
			- clients can only query topics that already exist 
			- the client will get the response and use that data
- ROS Timers
	- If you want to publisher, subscribe or query at some sort of regular frequency timers can be used
	- when you create a timer you can place a function that will be executed at a user specified frequency

