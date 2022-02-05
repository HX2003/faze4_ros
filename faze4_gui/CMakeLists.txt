cmake_minimum_required(VERSION 3.5)
project(faze4_gui)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)

include_directories(include)

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

add_library(${PROJECT_NAME} SHARED
            src/${PROJECT_NAME}.cpp
            src/faze4_motion.cpp
            include/${PROJECT_NAME}.hpp
            include/faze4_motion.hpp
            )

# Link ament packages
ament_target_dependencies(${PROJECT_NAME}
rclcpp rviz_common
moveit_core
moveit_ros_planning
moveit_ros_planning_interface
moveit_visual_tools)

# Link non ament packages
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
  )

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# replaces catkin_package(LIBRARIES ${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})

ament_package()
