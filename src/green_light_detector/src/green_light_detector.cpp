#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

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
      "image_raw", 10,
      std::bind(&GreenLightDetectorNode::imageCallback, this, std::placeholders::_1));

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera_info", 10,
      std::bind(&GreenLightDetectorNode::cameraInfoCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Green Light Detector Node started.");
  }

private:
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

    // 画出图像中心点和手动点
    cv::Point center(img_width_ / 2, img_height_ / 2);
    if (camera_info_received_) {
      cv::circle(img, center, 5, cv::Scalar(0, 0, 255), -1); // 红色中心点
      manual_point_ = cv::Point(center.x + manual_offset_x_, center.y + manual_offset_y_);
      cv::circle(img, manual_point_, 5, cv::Scalar(0, 255, 255), -1); // 黄色手动点
    }

    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    cv::Scalar lower_green(lower_h_, lower_s_, lower_v_);
    cv::Scalar upper_green(upper_h_, upper_s_, upper_v_);

    cv::Mat mask;
    cv::inRange(hsv, lower_green, upper_green, mask);

    cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    bool detected = false;
for (const auto& contour : contours) {
      if (cv::contourArea(contour) > 100) {
        cv::Rect rect = cv::boundingRect(contour);
        cv::rectangle(img, rect, cv::Scalar(0,255,0), 2);
        cv::Point green_center(rect.x + rect.width/2, rect.y + rect.height/2);
        cv::circle(img, green_center, 5, cv::Scalar(255, 0, 0), -1);

        if (camera_info_received_) {
          int dx = green_center.x - manual_point_.x;
          int dy = green_center.y - manual_point_.y;
          double manual_dist = std::sqrt(dx * dx + dy * dy);
          RCLCPP_INFO(this->get_logger(),
            "Green light at [%d, %d], offset from manual point: dx=%d, dy=%d, distance: %.2f px",
            green_center.x, green_center.y, dx, dy, manual_dist);
        } else {
          RCLCPP_INFO(this->get_logger(), "Green light detected at [%d, %d]", green_center.x, green_center.y);
        }
        detected = true;
      }
    }

    if (!detected) {
      RCLCPP_INFO(this->get_logger(), "No green light detected.");
    }

    cv::imshow("Green Light Detection", img);
    cv::waitKey(1);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  int lower_h_, lower_s_, lower_v_;
  int upper_h_, upper_s_, upper_v_;
  int img_width_ = 0, img_height_ = 0;
  bool camera_info_received_ = false;
  int manual_offset_x_ = 0;
  int manual_offset_y_ = 0;
  cv::Point manual_point_{0, 0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GreenLightDetectorNode>());
  rclcpp::shutdown();
  return 0;
}