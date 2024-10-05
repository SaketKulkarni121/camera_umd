#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <thread>
#include <uvc_cam/uvc_cam.h>
namespace uvc_cam {

class Camera {
public:
    Camera(rclcpp::Node::SharedPtr comm_node, rclcpp::Node::SharedPtr param_node);
    void onInit();
    void sendInfo(sensor_msgs::msg::Image::SharedPtr &image, rclcpp::Time time);
    void sendInfoJpeg(rclcpp::Time time);
    void feedImages();
    ~Camera();

private:
    std::unique_ptr<uvc_cam::Cam> cam;
    void setCameraControls();  // Declaration added here

    rclcpp::Node::SharedPtr node;
    rclcpp::Node::SharedPtr pnode;
    image_transport::ImageTransport it;
    bool ok;

    int width, height, fps, skip_frames, frames_to_skip;
    std::string device, frame, format;
    bool rotate;

    camera_info_manager::CameraInfoManager info_mgr;

    image_transport::Publisher pub;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pubjpeg;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;

    std::thread image_thread;
};

} // namespace uvc_camera
