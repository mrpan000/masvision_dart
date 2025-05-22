#ifndef MAS_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define MAS_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <cstdint>

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>

// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rm_interfaces/msg/green_light_detector.hpp"

namespace mas_serial_driver
{

  struct ReceivePacket
  {
    uint8_t header; //0xFf
    uint8_t heat;
    uint8_t tail; //0xD0
  }__attribute__((packed));

  struct SendPacket
  {
    uint8_t header=0xFF; 
    uint8_t detected;
    float dx;
    float dy;
    float distance;
    uint8_t tail = 0x0D;
  }__attribute__((packed));

  inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
  {
    ReceivePacket packet;
    std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
    return packet;
  }

  inline std::vector<uint8_t> toVector(const SendPacket & data)
  {
    std::vector<uint8_t> packet(sizeof(SendPacket));
    std::copy(
      reinterpret_cast<const uint8_t *>(&data),
      reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
    return packet;
  }

  class RMSerialDriver : public rclcpp::Node
  {
    public:
      explicit RMSerialDriver(const rclcpp::NodeOptions & options);

      ~RMSerialDriver() override;

    private:
      void getParams();

      void receiveData();

      void sendData(const rm_interfaces::msg::GreenLightDetector::SharedPtr msg);

      void reopenPort();

      void resetTracker();


      // Serial port
      std::unique_ptr<IoContext> owned_ctx_;
      std::string device_name_;
      std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
      std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;


      //subscriber
      rclcpp::Subscription<rm_interfaces::msg::GreenLightDetector>::SharedPtr gimbal_cmd_sub_;


      std::string target_frame_;

      std::thread receive_thread_;
  };
} 

#endif 