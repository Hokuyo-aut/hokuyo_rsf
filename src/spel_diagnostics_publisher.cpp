/*
 * hokuyo_spel_master
 * 
 * Copyright (c) 2025 LOCT Co., Ltd.
 * All rights reserved.
 *
 * This software is the property of LOCT Co., Ltd.
 * It is provided solely for evaluation and joint development purposes
 * under prior agreement with LOCT Co., Ltd.
 *
 * Redistribution or use outside the agreed scope is prohibited
 * without written permission from LOCT Co., Ltd.
 */

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <chrono>
#include <string>

using namespace std::chrono_literals;

class SpelDiagnosticsPublisher : public rclcpp::Node {
 public:
  SpelDiagnosticsPublisher()
  : Node("spel_diagnostics_publisher")
  {
    pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);

    timer_ = this->create_wall_timer(
      1s, std::bind(&SpelDiagnosticsPublisher::onTimer, this));

    RCLCPP_INFO(this->get_logger(), "SPEL diagnostics publisher started (1 Hz).");
  }

 private:
  void onTimer() {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = this->now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "spel_device";
    status.hardware_id = "H0000001";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "OK";

    // ---- Key-Value entries ----
    add(status, "ip_address", "192.168.0.1");
    add(status, "ip_port", "10940");
    add(status, "product_name", "RSF-X001");
    add(status, "firmware_version", "1.0.0");
    add(status, "device_id", "H0000001");
    add(status, "device_status", "0");
    add(status, "device_temperature", "35000");
    add(status, "cpu_usage", "35");
    add(status, "elapsed_time", "10000");
    add(status, "odometry_state", "0");
    add(status, "odometry_type", "0");
    add(status, "gnss_state", "0");
    add(status, "gnss_type", "0");

    array.status.push_back(status);
    pub_->publish(array);
  }

  static void add(
    diagnostic_msgs::msg::DiagnosticStatus& status,
    const std::string& key,
    const std::string& value)
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status.values.push_back(kv);
  }

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpelDiagnosticsPublisher>());
  rclcpp::shutdown();
  return 0;
}