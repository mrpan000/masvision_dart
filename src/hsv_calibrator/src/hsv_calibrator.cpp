#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <vector>
#include <string>
#include <iostream>

cv::Mat g_hsv_img;
int lower_h = 180, lower_s = 255, lower_v = 255;
int upper_h = 0, upper_s = 0, upper_v = 0;

void onMouse(int event, int x, int y, int, void*)
{
    if (event == cv::EVENT_LBUTTONDOWN && !g_hsv_img.empty()) {
        cv::Vec3b hsv = g_hsv_img.at<cv::Vec3b>(y, x);
        int h = hsv[0], s = hsv[1], v = hsv[2];
        lower_h = std::min(lower_h, h);
        lower_s = std::min(lower_s, s);
        lower_v = std::min(lower_v, v);
        upper_h = std::max(upper_h, h);
        upper_s = std::max(upper_s, s);
        upper_v = std::max(upper_v, v);
        std::cout << "Clicked HSV: " << h << "," << s << "," << v << std::endl;
        std::cout << "Current Range: "
                  << "H:[" << lower_h << "," << upper_h << "] "
                  << "S:[" << lower_s << "," << upper_s << "] "
                  << "V:[" << lower_v << "," << upper_v << "]" << std::endl;
    }
}

class HSVCalibrator : public rclcpp::Node
{
public:
    HSVCalibrator() : Node("hsv_calibrator")
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10,
            std::bind(&HSVCalibrator::imageCallback, this, std::placeholders::_1));
        cv::namedWindow("HSV Calib");
        cv::setMouseCallback("HSV Calib", onMouse, nullptr);
        RCLCPP_INFO(this->get_logger(), "HSV Calibrator started. Click image to sample HSV.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

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
        g_hsv_img = hsv;
        cv::imshow("HSV Calib", img);
        cv::waitKey(1);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HSVCalibrator>());
    rclcpp::shutdown();
    std::cout << "Final HSV Range: "
              << "H:[" << lower_h << "," << upper_h << "] "
              << "S:[" << lower_s << "," << upper_s << "] "
              << "V:[" << lower_v << "," << upper_v << "]" << std::endl;
    return 0;
}