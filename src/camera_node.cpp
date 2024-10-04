#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "uvc_camera/camera.h"  // Ensure this matches your header file name

int main(int argc, char **argv) {
  // Initialize the ROS 2 system
  rclcpp::init(argc, argv);

  // Create a shared pointer to the node
  auto camera_node = std::make_shared<uvc_camera::Camera>();

  // Spin the node
  rclcpp::spin(camera_node);

  // Shutdown the ROS 2 system
  rclcpp::shutdown();
  return 0;
}
