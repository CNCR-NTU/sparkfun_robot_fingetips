cmake_minimum_required(VERSION 2.8.3)
project(sparkfun_robot_fingertips)

catkin_python_setup()
find_package(catkin REQUIRED rospy std_msgs)
catkin_package(
	DEPENDS
	CATKIN_DEPENDS
	INCLUDE_DIRS
)
