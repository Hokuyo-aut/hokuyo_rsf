/*
 * hokuyo_spel_ros_node
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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nmea_msgs/msg/gpgga.hpp>
#include <nmea_msgs/msg/gprmc.hpp>
#include <nmea_msgs/msg/gpzda.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <iostream>
#include <string>
#include <cstring>
#include <vector>
 
#include <hokuyo_spel_master/spnet_utils.hpp>
 
namespace hsp {
 
class HokuyoSpelPublisher {
     
 public:
  HokuyoSpelPublisher();
     
  ~HokuyoSpelPublisher();

  void publishTf(
    const std::shared_ptr<tf2_ros::TransformBroadcaster>& broadcaster,
    uint64_t stamp,
    const std::string& frame_id,
    const std::string& child_frame_id,
    const spnet::OdomPacket& pkt);
  
  void publishOdom(
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& pub,
    uint64_t stamp,
    const std::string& frame_id,
    const std::string& child_frame_id,
    const spnet::OdomPacket& pkt);

  void publishHokuyoCloud2(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub,
    uint64_t stamp,
    const std::string& frame_id,
    const spnet::PointCloudPacketHeader& hdr,
    const spnet::PointXYZIT* points);

  void publishImu(
    const rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr& pub,
    uint64_t stamp,
    const std::string& frame_id,
    const spnet::ImuPacket& pkt);

  void publishNavSatFix(
    const rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr& pub,
    uint64_t stamp,
    const std::string& frame_id,
    const spnet::NavSatFixPacket& pkt);

  void publishGpgga(
    const rclcpp::Publisher<nmea_msgs::msg::Gpgga>::SharedPtr& pub,
    uint64_t stamp,
    const std::string& frame_id,
    const spnet::GpggaPacket& pkt);

  void publishGprmc(
    const rclcpp::Publisher<nmea_msgs::msg::Gprmc>::SharedPtr& pub,
    uint64_t stamp,
    const std::string& frame_id,
    const spnet::GprmcPacket& pkt);

  void publishGpzda(
    const rclcpp::Publisher<nmea_msgs::msg::Gpzda>::SharedPtr& pub,
    uint64_t stamp,
    const std::string& frame_id,
    const spnet::GpzdaPacket& pkt);

  void publishString(
    const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr& pub,
    const std::string& str);

  void publishDiagnostics(
    const rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr& pub,
    uint64_t stamp,
    const spnet::DiagnosticPacket& pkt);
}; // class HokuyoSpelPublisher
     
} // namespace hsp