
cmake_minimum_required(VERSION 2.8.3)

project(across_optitrack)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	std_msgs
	tf
	geometry_msgs
	genmsg 
	message_generation)

add_message_files(
	DIRECTORY msg
	FILES pc2quadcopter.msg  RigidBody.msg Vector3F.msg quadcopter2pc.msg)


generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
	INCLUDE_DIRS launch src msg
	LIBRARIES across_optitrack
	CATKIN_DEPENDS geometry_msgs roscpp std_msgs message_runtime
	DEPENDS system_lib
	)

include_directories(include ${catkin_INCLUDE_DIRS})



add_executable(MOCAPnode src/MOCAPNode.cpp)
add_executable(mocap_tfpub src/mocap_tfpub.cpp)
#add_executable(kalman_filter_data src/kalman_filter_data.cpp)

add_library(MS src/MOCAPSocket.cpp)
add_library(JOY src/SDLinput.cpp)

target_link_libraries(MOCAPnode MS)
target_link_libraries(MOCAPnode JOY)
target_link_libraries(MOCAPnode SDL pthread)

target_link_libraries(MOCAPnode ${catkin_LIBRARIES})
#target_link_libraries(kalman_filter_data ${catkin_LIBRARIES})

target_link_libraries(mocap_tfpub ${catkin_LIBRARIES} MS)
