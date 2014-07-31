cmake_minimum_required(VERSION 2.8.3)
project(davinci_trajectory)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs rospy std_msgs)

add_message_files(FILES
	Constants.msg
	TrajoptCall.msg
)

catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
   #INCLUDE_DIRS include
   CATKIN_DEPENDS message_runtime geometry_msgs)

#install(PROGRAMS scripts/tf_echo scripts/save_tf scripts/load_tf scripts/topic_echo scripts/publisher scripts/pose_publisher scripts/tf_publisher
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)
