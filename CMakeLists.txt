cmake_minimum_required(VERSION 3.8)
project(rviz_fps_plugin)

#add_compile_options(-DWITH_QT:BOOL=OFF)

## System dependencies are found with CMake's conventions
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters visualization segmentation surface)
find_package(CGAL REQUIRED)
#find_package(Eigen3 REQUIRED CONFIG)
find_package(Eigen3 3.1.0 REQUIRED CONFIG)
find_package(GMP REQUIRED)
find_package(MPFR REQUIRED)
# ROS


find_package(ament_cmake_ros REQUIRED)

find_package(rviz2 REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(pluginlib REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets Test Gui)

find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)



find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vilens_slam_utils REQUIRED)
find_package(icp_odometry REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

include_directories(include
  SYSTEM ${EIGEN3_INCLUDE_DIRS})
  
  
set(SOURCE_FILES
  src/fps_motion_view_controller.cpp
  src/fps_motion_tool.cpp
  ${MOC_FILES}
)

set(CMAKE_AUTOMOC ON)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt5_wrap_cpp(SOURCE_FILES
  include/fps_motion_view_controller.h
  include/fps_motion_tool.h
)

## An rviz plugin is just a shared library, so here we declare the
## library to be called ``${PROJECT_NAME}``

add_library(${PROJECT_NAME} ${SOURCE_FILES})


target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# this seems wrong mfallon - no include directory
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
#  ${geometry_msgs_TARGETS}
#  image_transport::image_transport
#  interactive_markers::interactive_markers
#  laser_geometry::laser_geometry
#  ${map_msgs_TARGETS}
#  ${nav_msgs_TARGETS}
#  point_cloud_transport::point_cloud_transport
#  rclcpp::rclcpp
  pluginlib::pluginlib
  rviz_common::rviz_common
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  rviz_rendering::rviz_rendering
  rviz_default_plugins::rviz_default_plugins
#  ${sensor_msgs_TARGETS}
#  tf2::tf2
#  ${tf2_geometry_msgs_TARGETS}
#  tf2_ros::tf2_ros
#  urdf::urdf
#  ${visualization_msgs_TARGETS}
)


# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_FPS_PLUGIN_BUILDING_LIBRARY")


pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins_description.xml)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)


########################################
# Plane Viz library. This is reused in several tools
#add_library(plane_viz SHARED src/plane_viz/plane_viz.cpp)
#ament_target_dependencies(plane_viz
#  nav_msgs pcl_conversions rclcpp tf2_eigen visualization_msgs std_msgs icp_odometry vilens_slam_utils)
#target_link_libraries(plane_viz ${PCL_LIBRARIES})

# Tool to colorise a mesh according to the same color model
#add_executable(color_mesh_by_normals  src/tools/color_mesh_by_normals.cpp)
#ament_target_dependencies(color_mesh_by_normals)
#target_link_libraries(color_mesh_by_normals rviz_fps_plugin ${PCL_LIBRARIES})

#############
## Install ##
#############

#install(TARGETS
#  rviz_fps_plugin_node
#  DESTINATION lib/${PROJECT_NAME}
#)

## Mark libraries for installation
#install(TARGETS
#  rviz_fps_plugin plane_viz rviz_fps_plugin_node_lib
#  DESTINATION lib
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

#install(DIRECTORY config/
#        DESTINATION config)

#install(DIRECTORY launch/
#        DESTINATION launch)

#install(DIRECTORY config launch processing_scripts
#  DESTINATION share/${PROJECT_NAME}
#  FILES_MATCHING PATTERN "*procman.pmd"
#                 PATTERN "*.yaml"
#                 PATTERN "*.xml"
#                 PATTERN "*.perspective"
#                 PATTERN "*.rviz"
#                 PATTERN "*.py"
#                 PATTERN "*.launch"
#)

ament_export_include_directories(include)
ament_export_libraries(rviz_fps_plugin)
ament_export_dependencies(
#
Eigen3 PCL vilens_slam_utils icp_odometry
#
  Eigen3
  Qt5
  rviz_ogre_vendor
)

ament_package(
  CONFIG_EXTRAS "rviz_fps_plugin-extras.cmake"
)
