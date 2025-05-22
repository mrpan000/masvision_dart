#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "rm_interfaces/msg/green_light_detector.hpp"

class GreenLightDetectorNode : public rclcpp::Node
{
public:
  GreenLightDetectorNode() : Node("green_light_detector_node")
  {
    // 声明参数，支持yaml加载
    this->declare_parameter("lower_h", 30);
    this->declare_parameter("lower_s", 80);
    this->declare_parameter("lower_v", 80);
    this->declare_parameter("upper_h", 100);
    this->declare_parameter("upper_s", 255);
    this->declare_parameter("upper_v", 255);

    this->get_parameter("lower_h", lower_h_);
    this->get_parameter("lower_s", lower_s_);
    this->get_parameter("lower_v", lower_v_);
    this->get_parameter("upper_h", upper_h_);
    this->get_parameter("upper_s", upper_s_);
    this->get_parameter("upper_v", upper_v_);
    this->declare_parameter("manual_offset_x", 0);
    this->declare_parameter("manual_offset_y", 0);

    this->get_parameter("manual_offset_x", manual_offset_x_);
    this->get_parameter("manual_offset_y", manual_offset_y_);

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw",
      rclcpp::SensorDataQoS(),
      std::bind(&GreenLightDetectorNode::imageCallback, this, std::placeholders::_1)
    );
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera_info",
      rclcpp::SensorDataQoS(),
      std::bind(&GreenLightDetectorNode::cameraInfoCallback, this, std::placeholders::_1)
    );

    detect_pub_ = this->create_publisher<rm_interfaces::msg::GreenLightDetector>("green_light_detect", 10);

    RCLCPP_INFO(this->get_logger(), "Green Light Detector Node started.");
  }

private:

  rclcpp::Publisher<rm_interfaces::msg::GreenLightDetector>::SharedPtr detect_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  int lower_h_, lower_s_, lower_v_;
  int upper_h_, upper_s_, upper_v_;
  int img_width_ = 0, img_height_ = 0;
  bool camera_info_received_ = false;
  int manual_offset_x_ = 0;
  int manual_offset_y_ = 0;
  cv::Point manual_point_{0, 0};

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    img_width_ = msg->width;
    img_height_ = msg->height;
    camera_info_received_ = true;
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat img;
    try {
      img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
    cv::imshow("HSV Image", hsv);

    cv::Scalar lower_green(lower_h_, lower_s_, lower_v_);
    cv::Scalar upper_green(upper_h_, upper_s_, upper_v_);

    cv::Mat mask;
    cv::inRange(hsv, lower_green, upper_green, mask);
    cv::imshow("Green Mask_before", mask);

    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);

    // 1. 先找一次轮廓并填充
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::drawContours(mask, contours, -1, cv::Scalar(255), cv::FILLED);

    // 2. 在填充后的mask上再找一次轮廓
    std::vector<std::vector<cv::Point>> filled_contours;
    cv::findContours(mask, filled_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::imshow("Green Mask_after", mask);

    rm_interfaces::msg::GreenLightDetector detect_msg;
    detect_msg.header.stamp = this->now();
    detect_msg.header.frame_id = msg->header.frame_id;
    detect_msg.detected = false;
    detect_msg.dx = 0.0;
    detect_msg.dy = 0.0;
    detect_msg.distance = 0.0;

    bool detected = false;
    for (const auto& contour : filled_contours) {
      double area = cv::contourArea(contour);
      if (area < 100 || area > 1000) continue; // 面积不能太小或太大

      // 圆度判据：4π*面积/周长^2 越接近1越圆
      double perimeter = cv::arcLength(contour, true);
      double circularity = 0;
      if (perimeter > 0)
        circularity = 4 * M_PI * area / (perimeter * perimeter);
      if (circularity < 0.5) continue; // 只要较圆的

      // 画最小外接圆
      cv::Point2f center;
      float radius;
      cv::minEnclosingCircle(contour, center, radius);
      cv::circle(img, center, static_cast<int>(radius), cv::Scalar(0,255,0), 2);

      // 计算与手动点的偏差
      if (camera_info_received_) {
        int dx = static_cast<int>(center.x) - manual_point_.x;
        int dy = static_cast<int>(center.y) - manual_point_.y;
        double manual_dist = std::sqrt(dx * dx + dy * dy);

        detect_msg.detected = true;
        detect_msg.dx = dx;
        detect_msg.dy = dy;
        detect_msg.distance = manual_dist;

        RCLCPP_INFO(this->get_logger(),
          "Green circle at [%.1f, %.1f], r=%.1f, dx=%d, dy=%d, distance=%.2f px, circularity=%.2f",
          center.x, center.y, radius, dx, dy, manual_dist, circularity);
      }
      detected = true;
      break; // 只取第一个
    }

    detect_pub_->publish(detect_msg);

    if (!detected) {
      RCLCPP_INFO(this->get_logger(), "No green light detected.");
    }

    // 画出图像中心点和手动点
    cv::Point center(img_width_ / 2, img_height_ / 2);
    if (camera_info_received_) {
      cv::circle(img, center, 5, cv::Scalar(0, 0, 255), -1); // 红色中心点
      manual_point_ = cv::Point(center.x + manual_offset_x_, center.y + manual_offset_y_);
      cv::circle(img, manual_point_, 5, cv::Scalar(0, 255, 255), -1); // 黄色手动点
    }

    cv::imshow("Green Light Detection", img);
    cv::waitKey(1);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GreenLightDetectorNode>());
  rclcpp::shutdown();
  return 0;
}