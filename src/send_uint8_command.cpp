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
#include <std_msgs/msg/u_int8.hpp>

#include <chrono>
#include <string>
#include <cstdlib>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // argv[1] = uint8 value
  if (argc < 2) {
    std::cerr << "Usage: ros2 run hokuyo_spel_master send_uint8 <0-255>\n"
              << "Example: ros2 run hokuyo_spel_master send_uint8 1\n";
    rclcpp::shutdown();
    return 1;
  }

  int value = std::atoi(argv[1]);
  if (value < 0 || value > 255) {
    std::cerr << "Value must be in range [0, 255]\n";
    rclcpp::shutdown();
    return 1;
  }

  auto node = std::make_shared<rclcpp::Node>("send_uint8");

  // Topic parameter (合わせやすいように param 化)
  node->declare_parameter<std::string>("topic", "/rsf/cmd_to_spel");
  std::string topic;
  node->get_parameter("topic", topic);

  rclcpp::QoS cmdQos(rclcpp::KeepLast(10));
  cmdQos.reliable();
  cmdQos.transient_local();
  auto pub = node->create_publisher<std_msgs::msg::UInt8>(topic, cmdQos);

  // Wait for DDS matching
  rclcpp::sleep_for(std::chrono::milliseconds(200));

  std_msgs::msg::UInt8 msg;
  msg.data = static_cast<uint8_t>(value);
  pub->publish(msg);

  // Flush
  rclcpp::spin_some(node);
  rclcpp::sleep_for(std::chrono::milliseconds(200));

  RCLCPP_INFO(node->get_logger(),
              "Published UInt8 value %u on topic '%s'",
              msg.data, topic.c_str());

  rclcpp::shutdown();
  return 0;
}