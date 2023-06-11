
cmake_minimum_required(VERSION 3.0.2)
project(jetson_on_wheels)

find_package(catkin REQUIRED COMPONENTS
  roscpp  
  roslib
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  ClockSync.srv
)

generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES jetson_on_wheels
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(motor_controller src/motor_controller.cpp include/JHPWMDriver/src/JHPWMPCA9685.cpp)
target_link_libraries(motor_controller ${catkin_LIBRARIES} i2c)

add_executable(steering_controller src/steering_controller.cpp include/JHPWMDriver/src/JHPWMPCA9685.cpp)
target_link_libraries(steering_controller ${catkin_LIBRARIES} i2c)

add_executable(clock_sync src/clock_sync.cpp)
target_link_libraries(clock_sync ${catkin_LIBRARIES})