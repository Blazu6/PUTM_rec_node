#pragma once

#include "putm_vcl_interfaces/msg/rtd.hpp"
#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

class RecNode : public rclcpp::Node {
 public:
  RecNode();

 private:
  rclcpp::Subscription<putm_vcl_interfaces::msg::Rtd>::SharedPtr rtd_subscription;
  rclcpp::TimerBase::SharedPtr stop_timer_;
  bool recording;
  int pid_{-1};

  void rtd_callback(const putm_vcl_interfaces::msg::Rtd::SharedPtr msg);
  void start_recording();
  void stop_recording();
};
