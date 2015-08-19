cmake_minimum_required(VERSION 2.8.3)
project(talker)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)
#catkin_package()
#include_directories(${catkin_INCLUDE_DIRS})
include_directories(${rclcpp_INCLUDE_DIRS}
                    ${rmw_implementation_INCLUDE_DIRS}
                    ${std_msgs_INCLUDE_DIRS})
add_executable(talker talker.cpp)
#target_link_libraries(talker ${catkin_LIBRARIES})
target_link_libraries(talker
                      ${rclcpp_LIBRARIES}
                      ${rmw_implementation_LIBRARIES}
                      ${std_msgs_LIBRARIES})
#install(TARGETS talker
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS talker RUNTIME DESTINATION bin)
ament_package()
