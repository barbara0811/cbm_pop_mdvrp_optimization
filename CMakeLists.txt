cmake_minimum_required(VERSION 2.8.3)
project(cbm_pop_mdvrp)

find_package(catkin REQUIRED COMPONENTS
	rospy
  	std_msgs
	geometry_msgs
	message_generation
)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
	FILES
	BestSolution.msg
	WeightMatrix.msg
	MissionInfo.msg
	FloatArray.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
)
