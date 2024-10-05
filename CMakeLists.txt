cmake_minimum_required(VERSION 3.5)
project(uvc_camera)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system)
# find_package(V4L2 REQUIRED)
set(V4L2_LIBRARIES v4l2)

# Include directories
include_directories(
  include
)

# Declare a C++ library
# add_library(nodelet_uvc_camera SHARED
#   src/nodelets.cpp
#   src/camera.cpp
#   src/uvc_cam.cpp
# )

# # Link libraries
# target_link_libraries(nodelet_uvc_camera
#   ${Boost_LIBRARIES}
#   ${V4L2_LIBRARIES}
#   rclcpp::rclcpp
#   camera_info_manager::camera_info_manager
#   image_transport::image_transport
#   rclcpp_components::component
# )

# Register the components
# rclcpp_components_register_node(nodelet_uvc_camera
#   PLUGIN "uvc_camera::camera_node"
#   EXECUTABLE "camera_node"
# )

# Add executables
add_executable(uvc_camera_node src/camera_node.cpp src/camera.cpp src/uvc_cam.cpp)
target_link_libraries(uvc_camera_node
  ${Boost_LIBRARIES}
  ${V4L2_LIBRARIES}
  rclcpp::rclcpp
)

# Install targets
install(TARGETS
  uvc_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if any
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# Ament package
ament_package()
