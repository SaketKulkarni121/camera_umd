#include <memory>
#include <chrono>
#include <cstring>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "uvc_cam/uvc_cam.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "image_transport/image_transport.hpp"
#include "uvc_camera/camera.h"

using namespace sensor_msgs::msg;

namespace uvc_cam {

Camera::Camera(rclcpp::Node::SharedPtr comm_node, rclcpp::Node::SharedPtr param_node)
    : node(comm_node),
      pnode(param_node),
      it(comm_node),
      ok(false),
      info_mgr(comm_node.get(), "camera_name", "camera_info_url") {

    std::string url;

    node->get_parameter("camera_info_url", url);
    
    // Default config values
    width = 640;
    height = 480;
    fps = 10;
    device = "/dev/video0";
    frame = "camera";
    format = "rgb";

    info_mgr.loadCameraInfo(url);
    
    // Pull other configuration
    node->get_parameter("device", device);
    node->get_parameter("fps", fps);
    node->get_parameter("skip_frames", skip_frames);
    node->get_parameter("width", width);
    node->get_parameter("height", height);
    node->get_parameter("frame_id", frame);
    node->get_parameter("format", format);
    
    // Advertise image streams and info streams
    if (format != "jpeg") {
        pub = it.advertise("image_raw", 1);
    } else {
        pubjpeg = node->create_publisher<sensor_msgs::msg::CompressedImage>("image_raw/compressed", 1);
    }
    info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 1);

    // Initialize the camera
    uvc_cam::Cam::mode_t mode = (format == "jpeg") ? uvc_cam::Cam::MODE_MJPG : uvc_cam::Cam::MODE_RGB;

    // Set camera controls
    setCameraControls();

    // Start the image feeding thread
    ok = true;
    image_thread = std::thread(&Camera::feedImages, this);
    cam = std::make_unique<uvc_cam::Cam>(device.c_str(), mode, width, height, fps);
}

void Camera::setCameraControls() {
    if (cam) {
    // Set various camera controls using parameters
        bool auto_focus;
        if (node->get_parameter("auto_focus", auto_focus)) {
            cam->set_v4l2_control(V4L2_CID_FOCUS_AUTO, auto_focus, "auto_focus");
        }
    
        int focus_absolute;
        if (node->get_parameter("focus_absolute", focus_absolute)) {
            cam->set_v4l2_control(V4L2_CID_FOCUS_ABSOLUTE, focus_absolute, "focus_absolute");
        }

        bool auto_exposure;
        if (node->get_parameter("auto_exposure", auto_exposure)) {
            int val = auto_exposure ? V4L2_EXPOSURE_AUTO : V4L2_EXPOSURE_MANUAL;
            cam->set_v4l2_control(V4L2_CID_EXPOSURE_AUTO, val, "auto_exposure");
        }

        int exposure_absolute;
        if (node->get_parameter("exposure_absolute", exposure_absolute)) {
            cam->set_v4l2_control(V4L2_CID_EXPOSURE_ABSOLUTE, exposure_absolute, "exposure_absolute");
        }

        int brightness;
        if (node->get_parameter("brightness", brightness)) {
            cam->set_v4l2_control(V4L2_CID_BRIGHTNESS, brightness, "brightness");
        }

        int contrast;
        if (node->get_parameter("contrast", contrast)) {
            cam->set_v4l2_control(V4L2_CID_CONTRAST, contrast, "contrast");
        }

        // Additional camera controls can be set here...

        bool h_flip;
        if (node->get_parameter("horizontal_flip", h_flip)) {
            cam->set_v4l2_control(V4L2_CID_HFLIP, h_flip, "horizontal_flip");
        }

        bool v_flip;
        if (node->get_parameter("vertical_flip", v_flip)) {
            cam->set_v4l2_control(V4L2_CID_VFLIP, v_flip, "vertical_flip");
        }
    }
}

void Camera::sendInfo(sensor_msgs::msg::Image::SharedPtr &image, rclcpp::Time time) {
    camera_info_manager::CameraInfo info(info_mgr.getCameraInfo());

    // if (info.K[0] != 0.0 && (image->width != info.width || image->height != info.height)) {
    //     info = CameraInfo();  // Reset if not calibrated
    // }

    // if (info.K[0] == 0.0) {
    //     info.width = image->width;
    //     info.height = image->height;
    // }

    info.header.stamp = time;
    info.header.frame_id = frame;
    info_pub->publish(info);
}

void Camera::sendInfoJpeg(rclcpp::Time time) {
    CameraInfo info(info_mgr.getCameraInfo());
    info.header.stamp = time;
    info.header.frame_id = frame;
    info_pub->publish(info);
}

void Camera::feedImages() {
    unsigned int pair_id = 0;

    while (ok) {
        unsigned char *img_frame = nullptr;
        uint32_t bytes_used;
        auto capture_time = node->now();

        int idx = cam->grab(&img_frame, bytes_used);

        if (skip_frames == 0 || frames_to_skip == 0) {
            if (img_frame && format != "jpeg") {
                auto image = std::make_shared<Image>();
                image->height = height;
                image->width = width;
                image->step = 3 * width;
                image->encoding = "rgb8";  // Use string literals for encodings
                image->header.stamp = capture_time;
                image->header.frame_id = frame;
                image->data.resize(image->step * image->height);
                memcpy(image->data.data(), img_frame, width * height * 3);
                pub.publish(image);
                sendInfo(image, capture_time);
                ++pair_id;
            } else if (img_frame && format == "jpeg") {
                auto image = std::make_shared<sensor_msgs::msg::CompressedImage>();
                image->header.stamp = capture_time;
                image->header.frame_id = frame;
                image->data.resize(bytes_used);
                memcpy(image->data.data(), img_frame, bytes_used);
                pubjpeg->publish(*image);
                sendInfoJpeg(capture_time);
                ++pair_id;
            }
            frames_to_skip = skip_frames;
        } else {
            frames_to_skip--;
        }

        if (img_frame) cam->release(idx);
    }
}

Camera::~Camera() {
    ok = false; // Stop the thread
    if (image_thread.joinable()) {
        image_thread.join();
    }
}

} // namespace uvc_camera
