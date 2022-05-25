cmake_minimum_required(VERSION 3.5.0)
project(emojicloud_plugin)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
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

add_definitions(-DQT_NO_KEYWORDS)

set(header_files
    include/emojicloud_plugin/emoji_point_cloud2_display.h)
set(source_files
#  src/point_cloud.cpp
#  src/point_cloud_common.cpp
#  src/emoji_laser_scan_display.cpp
  src/emoji_point_cloud2_display.cpp
)

foreach(header "${header_files}")
  qt5_wrap_cpp(moc_files "${header}")
endforeach()

#ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

#target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

add_library(${PROJECT_NAME} SHARED
        ${moc_files}
        ${source_files}
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
        ${Qt5Widgets_INCLUDE_DIRS}
        )

target_link_libraries(${PROJECT_NAME} PUBLIC
        rviz_ogre_vendor::OgreMain
        rviz_ogre_vendor::OgreOverlay
        )

target_link_libraries(${PROJECT_NAME} PRIVATE
        ignition-math6
        )

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_target_dependencies(${PROJECT_NAME}
        PUBLIC
        rclcpp
        rviz_common
        rviz_rendering
        sensor_msgs
        )
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
        rclcpp
        rviz_common
        rviz_ogre_vendor
        sensor_msgs
)

#install(FILES
#  plugin_description.xml
#  DESTINATION share/${PROJECT_NAME}
#)

#install(DIRECTORY media
#  DESTINATION share/${PROJECT_NAME}
#)

#install(DIRECTORY icons
#  DESTINATION share/${PROJECT_NAME}
#)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
        DESTINATION "share/${PROJECT_NAME}"
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(
#        CONFIG_EXTRAS "rviz_default_plugins-extras.cmake"
)

