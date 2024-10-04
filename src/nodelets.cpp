#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "uvc_camera/camera.hpp"

namespace uvc_camera {

class CameraNode : public rclcpp::Node {
public:
    CameraNode(const rclcpp::NodeOptions & options)
        : Node("camera_node", options) {
        auto pnode = this->get_private_node();
        camera_ = std::make_shared<Camera>(shared_from_this(), pnode);
    }

private:
    std::shared_ptr<Camera> camera_;
};

}  // namespace uvc_camera

// Register the components with the plugin system
PLUGINLIB_EXPORT_CLASS(uvc_camera::CameraNode, rclcpp::Node)
