cmake_minimum_required(VERSION 3.0.2)
project(my_rviz_plugins)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  pluginlib
  std_msgs
  visualization_msgs
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rviz pluginlib std_msgs visualization_msgs
)

# Qt configulation
set(CMAKE_AUTOMOC ON)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED
  Core
  Widgets
)

set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

###########
## BUILD ##
###########

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/create_area_panel.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
