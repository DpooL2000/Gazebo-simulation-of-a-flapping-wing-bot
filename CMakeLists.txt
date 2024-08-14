cmake_minimum_required(VERSION 3.0.2)
project(bird2)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs

)

find_package(gazebo REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5PrintSupport REQUIRED) 
find_package(Boost REQUIRED system filesystem date_time)
find_package(ignition-math6 REQUIRED)

set(QCUSTOMPLOT_DIR "/home/dpool/Downloads/QCustomPlot/qcustomplot")
set(QCUSTOMPLOT_SRC ${QCUSTOMPLOT_DIR}/qcustomplot.cpp)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
  ${Boost_INCLUDE_DIR} 
  ${catkin_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS} 
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5PrintSupport_INCLUDE_DIRS}
  ${IGNITION-MATH_INCLUDE_DIRS}
  ${QCUSTOMPLOT_DIR}
  ${PROJECT_SOURCE_DIR}/include
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
)

add_library(flapping_controller src/flapping_controller.cpp)
target_link_libraries(flapping_controller ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(pitching_controller src/pitching_controller.cpp)
target_link_libraries(pitching_controller ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(rolling_controller src/rolling_controller.cpp)
target_link_libraries(rolling_controller ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(lift_drag_plugin src/lift_drag_plugin.cpp)
target_link_libraries(lift_drag_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# Add the Qt5 related settings
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Add the executable for the Qt GUI
add_executable(flight_simulator_gui src/flight_controller_gui.cpp ${QCUSTOMPLOT_SRC})

# Link the Qt5 and ROS libraries to the executable
target_link_libraries(flight_simulator_gui Qt5::Widgets Qt5::PrintSupport ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
