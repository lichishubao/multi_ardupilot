cmake_minimum_required(VERSION 3.0.2)
project(offboard)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES offboard
#  CATKIN_DEPENDS geometry_msgs mavros roscpp
#  DEPENDS system_lib
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(
    ${PROJECT_NAME}_node
    src/main.cpp

)

add_dependencies(
        ${PROJECT_NAME}_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)

