#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
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

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw", 10,
      std::bind(&GreenLightDetectorNode::imageCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Green Light Detector Node started.");
  }

private:
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

    // 使用参数
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
        detected = true;
        RCLCPP_INFO(this->get_logger(), "Green light detected at [%d, %d]", rect.x, rect.y);
      }
    }

    if (!detected) {
      RCLCPP_INFO(this->get_logger(), "No green light detected.");
    }

    cv::imshow("Green Light Detection", img);
    cv::waitKey(1);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  int lower_h_, lower_s_, lower_v_;
  int upper_h_, upper_s_, upper_v_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GreenLightDetectorNode>());
  rclcpp::shutdown();
  return 0;
}