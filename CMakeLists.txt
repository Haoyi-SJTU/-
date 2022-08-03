cmake_minimum_required(VERSION 3.0.2)
project(jakarobot)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  a.msg 
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include

  # LIBRARIES jakarobot
  CATKIN_DEPENDS roscpp rospy std_msgs message_generation message_runtime

  # DEPENDS system_lib
)

include_directories(

  # include
  ${catkin_INCLUDE_DIRS}
  ./src/libs
)

ADD_EXECUTABLE(jakatest src/jakatest.cpp)
ADD_EXECUTABLE(talker src/talker.cpp)
TARGET_LINK_LIBRARIES(jakatest ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/libs/libjakaAPI.so)
TARGET_LINK_LIBRARIES(talker ${catkin_LIBRARIES})
ADD_DEPENDENCIES(jakatest ${${PROJECT_NAME}_EXPORTED_TARGETS})
ADD_DEPENDENCIES(talker ${${PROJECT_NAME}_EXPORTED_TARGETS})

# ADD_DEPENDENCIES(jakatest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
