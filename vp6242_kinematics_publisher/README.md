# VP6242 Kinematics Joint_state Publisher

Firstly, clone on your catkin_ws/src the following dependencies:

	git clone https://github.com/bglima/robotiq_85_gripper
	git clone https://github.com/bglima/VP6242_ROS.git

	sudo apt-get install ros-$ROS_DISTRO-moveit
	sudo apt-get install ros-$ROS_DISTRO-effort-controllers
	sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers

	git clone https://github.com/bglima/vp6242_robotiq85_ros.git
	cd ~/catkin_ws
	catkin_make


Download and install the control tools for our robot model in order to turn on the topics which handle the arm.

	cd catkin_ws/src
	wstool init
	wstool merge https://raw.github.com/ros-controls/ros_control/kinetic-devel/ros_control.rosinstall
	wstool update
	cd ..
	rosdep install --from-paths . --ignore-src --rosdistro kinetic -y
	catkin_make


The following step comprises the installing of packages which allow us to publish joint states throug a ROS Node.
(Available on http://wiki.ros.org/robot_state_publisher 
and http://wiki.ros.org/urdf/Tutorials/Using%20urdf%20with%20robot_state_publisher)

	cd catkin_ws/src
	git clone https://github.com/ros/robot_state_publisher/tree/kinetic-devel/src
	cd ..
	catkin_make

Finally, clone the folder /vp6242_kinematics_publisher from https://github.com/glauberrleite/denso-pick-and-place.git inside your catkin_ws/src. 
Path back to the top level /catkin_ws 
and build with catkin_make.
