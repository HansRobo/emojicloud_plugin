cmake_minimum_required(VERSION 3.5.0)
project(emojicloud_plugin)

#find_package(catkin REQUIRED COMPONENTS rviz)
#catkin_package()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

#include_directories(
#  ${catkin_INCLUDE_DIRS}
#)
#
#link_directories(
#  ${catkin_LIBRARY_DIRS}
#)

set(CMAKE_AUTOMOC ON)

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

add_definitions(-DQT_NO_KEYWORDS)

set(SOURCE_FILES
  src/point_cloud.cpp
  src/point_cloud_common.cpp
  src/emoji_laser_scan_display.cpp
  src/emoji_point_cloud2_display.cpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})


#add_dependencies(${PROJECT_NAME}
#  ${${PROJECT_NAME}_EXPORTED_TARGETS}
#)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
)

install(FILES
  plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY media/
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY icons/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()

