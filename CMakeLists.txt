cmake_minimum_required(VERSION 3.0.2)

project(Rossenheimer)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
sensor_msgs
geometry_msgs
nav_msgs
tf
gazebo_ros
image_transport
cv_bridge
)

find_package(gazebo REQUIRED)
find_package(OpenCV 4 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf gazebo_ros image_transport cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(SOURCES
   src/main.cpp
   src/Controller.cpp
   src/Lidar.cpp
   src/Motor.cpp
   src/Odometer.cpp
   src/Sensor.cpp
   src/TurtleBot3.cpp
   src/Camera.cpp
)

set(HEADERS
  include/Rossenheimer/Controller.h
  include/Rossenheimer/Lidar.h
  include/Rossenheimer/Motor.h
  include/Rossenheimer/Odometer.h
  include/Rossenheimer/Sensor.h
  include/Rossenheimer/TurtleBot3.h
  include/Rossenheimer/Camera.h
)

add_executable(main ${SOURCES} ${HEADERS})
#add_executable(main src/main.cpp src/Controller.cpp src/Lidar.cpp src/Motor.cpp src/Odometer.cpp src/Sensor.cpp src/TurtleBot3.cpp src/Camera.cpp)
add_dependencies(main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_INCLUDE_DIRS})

install(TARGETS main
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install (DIRECTORY launch models rviz worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

