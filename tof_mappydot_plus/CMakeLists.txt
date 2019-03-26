cmake_minimum_required(VERSION 2.8.3)
project(tof_mappydot_plus)

add_compile_options(-std=c++11 -li2c)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs diagnostic_msgs)

catkin_package(INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(mappydot_plus_node src/mappydot_plus_activity.cpp src/mappydot_plus_node.cpp src/watchdog.cpp)
#add_dependencies(mappydot_plus_node diagnostic_msgs_gencpp)

include_directories(${catkin_INCLUDE_DIRS})

target_link_libraries(mappydot_plus_node ${catkin_LIBRARIES})

install(TARGETS mappydot_plus_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

