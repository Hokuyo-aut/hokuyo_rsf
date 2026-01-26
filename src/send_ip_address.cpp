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
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <string>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // argv[1] = IP address string
  if (argc < 2) {
    std::cerr << "Usage: ros2 run hokuyo_spel_master send_ip_address <IPv4>\n"
              << "Example: ros2 run hokuyo_spel_master send_ip_address 192.168.10.100\n";
    rclcpp::shutdown();
    return 1;
  }
  const std::string ip = argv[1];

  auto node = std::make_shared<rclcpp::Node>("send_ip_address");

  // Topic parameter (default matches your master node)
  node->declare_parameter<std::string>("topic", "/spel/ip_address");
  std::string topic;
  node->get_parameter("topic", topic);

  auto pub = node->create_publisher<std_msgs::msg::String>(topic, rclcpp::QoS(10));

  // Give DDS a moment to match subscriptions (best-effort for one-shot tools)
  rclcpp::sleep_for(std::chrono::milliseconds(200));

  std_msgs::msg::String msg;
  msg.data = ip;
  pub->publish(msg);

  // Let it flush out
  rclcpp::spin_some(node);
  rclcpp::sleep_for(std::chrono::milliseconds(200));

  RCLCPP_INFO(node->get_logger(), "Published IP address '%s' on topic '%s'",
              ip.c_str(), topic.c_str());

  rclcpp::shutdown();
  return 0;
}