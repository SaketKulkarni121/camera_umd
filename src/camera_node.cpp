#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "uvc_camera/camera.h"

class CameraNode : public rclcpp::Node {
public:
    CameraNode() : Node("camera_node") {
        camera_ = std::make_shared<uvc_cam::Camera>(this->shared_from_this(), this->shared_from_this());
    }

private:
    std::shared_ptr<uvc_cam::Camera> camera_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
