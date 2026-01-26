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

#include <hokuyo_spel_master/spel_publisher.hpp>

namespace hsp {
 
HokuyoSpelPublisher::HokuyoSpelPublisher() {

}
 
HokuyoSpelPublisher::~HokuyoSpelPublisher() {

}

void HokuyoSpelPublisher::publishTf(
  const std::shared_ptr<tf2_ros::TransformBroadcaster>& broadcaster,
  uint64_t stamp,
  const std::string& frame_id,
  const std::string& child_frame_id,
  const spnet::OdomPacket& pkt)
{
  geometry_msgs::msg::TransformStamped tf;

  tf.header.stamp = rclcpp::Time(stamp);
  tf.header.frame_id = frame_id;
  tf.child_frame_id = child_frame_id;
  tf.transform.translation.x = pkt.x;
  tf.transform.translation.y = pkt.y;
  tf.transform.translation.z = pkt.z;
  tf.transform.rotation.x = pkt.qx;
  tf.transform.rotation.y = pkt.qy;
  tf.transform.rotation.z = pkt.qz;
  tf.transform.rotation.w = pkt.qw;

  broadcaster->sendTransform(tf);
}

void HokuyoSpelPublisher::publishOdom(
  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& pub,
  uint64_t stamp,
  const std::string& frame_id,
  const std::string& child_frame_id,
  const spnet::OdomPacket& pkt)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = rclcpp::Time(stamp);
  odom.header.frame_id = frame_id;
  odom.child_frame_id  = child_frame_id;
  odom.pose.pose.position.x = pkt.x;
  odom.pose.pose.position.y = pkt.y;
  odom.pose.pose.position.z = pkt.z;
  odom.pose.pose.orientation.x = pkt.qx;
  odom.pose.pose.orientation.y = pkt.qy;
  odom.pose.pose.orientation.z = pkt.qz;
  odom.pose.pose.orientation.w = pkt.qw;
  odom.twist.twist.linear.x = pkt.vx;
  odom.twist.twist.linear.y = pkt.vy;
  odom.twist.twist.linear.z = pkt.vz;
  odom.twist.twist.angular.x = pkt.wx;
  odom.twist.twist.angular.y = pkt.wy;
  odom.twist.twist.angular.z = pkt.wz;
  for (size_t i = 0; i < 36; ++i) {
    odom.pose.covariance[i] = pkt.pos_cov[i];
    odom.twist.covariance[i] = pkt.vel_cov[i];
  }
  pub->publish(odom);
}

void HokuyoSpelPublisher::publishHokuyoCloud2(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub,
  uint64_t stamp,
  const std::string& frame_id,
  const spnet::PointCloudPacketHeader& hdr,
  const spnet::PointXYZIT* points)
{
  sensor_msgs::msg::PointCloud2 cloud;

  const uint32_t num_points = hdr.num_points;

  cloud.header.stamp = rclcpp::Time(stamp);
  cloud.header.frame_id = frame_id;
  cloud.height = 1;
  cloud.width  = num_points;

  cloud.fields.resize(6);

  cloud.fields[0].name     = "x";
  cloud.fields[0].offset   = 0;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[0].count    = 1;

  cloud.fields[1].name     = "y";
  cloud.fields[1].offset   = 4;
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].count    = 1;

  cloud.fields[2].name     = "z";
  cloud.fields[2].offset   = 8;
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[2].count    = 1;

  cloud.fields[3].name     = "intensity";
  cloud.fields[3].offset   = 12;
  cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[3].count    = 1;

  cloud.fields[4].name     = "sec";
  cloud.fields[4].offset   = 16;
  cloud.fields[4].datatype = sensor_msgs::msg::PointField::UINT32;
  cloud.fields[4].count    = 1;

  cloud.fields[5].name     = "nsec";
  cloud.fields[5].offset   = 20;
  cloud.fields[5].datatype = sensor_msgs::msg::PointField::UINT32;
  cloud.fields[5].count    = 1;

  cloud.is_bigendian = false;
  cloud.point_step   = sizeof(spnet::PointXYZIT);
  cloud.row_step     = cloud.point_step * cloud.width;
  cloud.is_dense     = true;

  cloud.data.resize(static_cast<size_t>(cloud.row_step) * cloud.height);
  std::memcpy(cloud.data.data(), points,
  static_cast<size_t>(num_points) * sizeof(spnet::PointXYZIT));
  pub->publish(cloud);
}

void HokuyoSpelPublisher::publishImu(
  const rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr& pub,
  uint64_t stamp,
  const std::string& frame_id,
  const spnet::ImuPacket& pkt)
{
  sensor_msgs::msg::Imu imu;

  imu.header.stamp = rclcpp::Time(stamp);
  imu.header.frame_id = frame_id;

  // orientation
  imu.orientation.x = pkt.qx;
  imu.orientation.y = pkt.qy;
  imu.orientation.z = pkt.qz;
  imu.orientation.w = pkt.qw;

  // angular velocity
  imu.angular_velocity.x = pkt.wx;
  imu.angular_velocity.y = pkt.wy;
  imu.angular_velocity.z = pkt.wz;

  // linear acceleration
  imu.linear_acceleration.x = pkt.ax;
  imu.linear_acceleration.y = pkt.ay;
  imu.linear_acceleration.z = pkt.az;

  // covariances
  for (size_t i = 0; i < 9; ++i) {
    imu.orientation_covariance[i]         = pkt.ori_cov[i];
    imu.angular_velocity_covariance[i]    = pkt.ang_vel_cov[i];
    imu.linear_acceleration_covariance[i] = pkt.lin_acc_cov[i];
  }

  pub->publish(imu);
}

void HokuyoSpelPublisher::publishNavSatFix(
  const rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr& pub,
  uint64_t stamp,
  const std::string& frame_id,
  const spnet::NavSatFixPacket& pkt)
{
  sensor_msgs::msg::NavSatFix msg;
  msg.header.stamp = rclcpp::Time(stamp);
  msg.header.frame_id = frame_id;
  msg.latitude  = pkt.latitude;
  msg.longitude = pkt.longitude;
  msg.altitude  = pkt.altitude;
  for (size_t i = 0; i < 9; ++i) {
    msg.position_covariance[i] = pkt.pos_cov[i];
  }
  msg.status.status  = pkt.status_status;
  msg.status.service = pkt.status_service;
  msg.position_covariance_type = pkt.position_covariance_type;
  pub->publish(msg);
}

void HokuyoSpelPublisher::publishGpgga(
  const rclcpp::Publisher<nmea_msgs::msg::Gpgga>::SharedPtr& pub,
  uint64_t stamp,
  const std::string& frame_id,
  const spnet::GpggaPacket& pkt)
{
  nmea_msgs::msg::Gpgga msg;

  msg.header.stamp = rclcpp::Time(stamp);
  msg.header.frame_id = frame_id;

  msg.message_id = std::string(pkt.message_id);

  msg.utc_seconds = pkt.utc_seconds;
  msg.lat = pkt.lat;
  msg.lon = pkt.lon;

  msg.lat_dir          = (pkt.lat_dir          == '\0') ? "" : std::string(1, pkt.lat_dir);
  msg.lon_dir          = (pkt.lon_dir          == '\0') ? "" : std::string(1, pkt.lon_dir);
  msg.altitude_units   = (pkt.altitude_units   == '\0') ? "" : std::string(1, pkt.altitude_units);
  msg.undulation_units = (pkt.undulation_units == '\0') ? "" : std::string(1, pkt.undulation_units);

  msg.gps_qual = pkt.gps_qual;
  msg.num_sats = pkt.num_sats;

  msg.hdop = pkt.hdop;
  msg.alt = pkt.alt;
  msg.undulation = pkt.undulation;

  msg.diff_age = pkt.diff_age;
  msg.station_id = std::string(pkt.station_id);

  pub->publish(msg);
}

void HokuyoSpelPublisher::publishGprmc(
  const rclcpp::Publisher<nmea_msgs::msg::Gprmc>::SharedPtr& pub,
  uint64_t stamp,
  const std::string& frame_id,
  const spnet::GprmcPacket& pkt)
{
  nmea_msgs::msg::Gprmc msg;

  msg.header.stamp = rclcpp::Time(stamp);
  msg.header.frame_id = frame_id;

  msg.message_id = std::string(
    pkt.message_id,
    strnlen(pkt.message_id, sizeof(pkt.message_id)));

  msg.utc_seconds = pkt.utc_seconds;

  // nmea_msgs/msg/Gprmc にあるので反映
  msg.position_status = (pkt.status == '\0') ? "" : std::string(1, pkt.status);

  msg.lat = pkt.lat;
  msg.lon = pkt.lon;

  msg.lat_dir = (pkt.lat_dir == '\0') ? "" : std::string(1, pkt.lat_dir);
  msg.lon_dir = (pkt.lon_dir == '\0') ? "" : std::string(1, pkt.lon_dir);

  // msg側は float32 なので narrow（必要なら static_cast<float>）
  msg.speed = static_cast<float>(pkt.speed);   // knots
  msg.track = static_cast<float>(pkt.track);   // degrees

  msg.date = std::string(
    pkt.date,
    strnlen(pkt.date, sizeof(pkt.date)));

  msg.mag_var = static_cast<float>(pkt.mag_var);

  // 追加: 磁気偏角の向き/モード
  msg.mag_var_direction = (pkt.mag_var_dir == '\0') ? "" : std::string(1, pkt.mag_var_dir);
  msg.mode_indicator    = (pkt.mode == '\0') ? "" : std::string(1, pkt.mode);

  pub->publish(msg);
}

void HokuyoSpelPublisher::publishGpzda(
  const rclcpp::Publisher<nmea_msgs::msg::Gpzda>::SharedPtr& pub,
  uint64_t stamp,
  const std::string& frame_id,
  const spnet::GpzdaPacket& pkt)
{
  nmea_msgs::msg::Gpzda msg;
  msg.header.stamp = rclcpp::Time(stamp);
  msg.header.frame_id = frame_id;
  msg.message_id = std::string(pkt.message_id, strnlen(pkt.message_id, sizeof(pkt.message_id)));
  msg.utc_seconds = pkt.utc_seconds;
  msg.day = pkt.day;
  msg.month = pkt.month;
  msg.year = pkt.year;
  msg.hour_offset_gmt = pkt.hour_offset_gmt;
  msg.minute_offset_gmt = pkt.minute_offset_gmt;

  pub->publish(msg);
}

void HokuyoSpelPublisher::publishString(
  const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr& pub,
  const std::string& str)
{
  std_msgs::msg::String msg;
  msg.data = str;
  pub->publish(msg);
}

static inline void AddKV(
  diagnostic_msgs::msg::DiagnosticStatus& status,
  const std::string& key,
  const std::string& value)
{
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = key;
  kv.value = value;
  status.values.push_back(kv);
}

// fixed-length char[8] を std::string に（末尾の '\0' まで / 無ければ8文字）
static inline std::string Fixed8ToString(const char s[8]) {
  const size_t n = ::strnlen(s, 8);
  return std::string(s, s + n);
}

// ip u32 (network order expected) -> dotted string
static inline std::string Ipv4U32ToString(uint32_t ip_net_order) {
  in_addr a{};
  a.s_addr = ip_net_order;  // network order
  char buf[INET_ADDRSTRLEN] = {0};
  if (::inet_ntop(AF_INET, &a, buf, sizeof(buf)) == nullptr) {
    return "0.0.0.0";
  }
  return std::string(buf);
}

// stamp_ns: UNIX epoch nanoseconds (same style as your other publishers)
void HokuyoSpelPublisher::publishDiagnostics(
  const rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr& pub,
  uint64_t stamp,
  const spnet::DiagnosticPacket& pkt)
{
  diagnostic_msgs::msg::DiagnosticArray array;

  // header stamp from ns
  array.header.stamp.sec =
    static_cast<int32_t>(stamp / 1000000000ULL);
  array.header.stamp.nanosec =
    static_cast<uint32_t>(stamp % 1000000000ULL);

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "spel_device";
  status.hardware_id = Fixed8ToString(pkt.device_id);

  status.level = (pkt.device_status == 0)
    ? diagnostic_msgs::msg::DiagnosticStatus::OK
    : diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message = (pkt.device_status == 0) ? "OK" : "WARN";

  // ---- Key-Value entries ----
  AddKV(status, "ip_address", Ipv4U32ToString(pkt.ip_address));  // pkt.ip_addressはinet_pton由来でnet order想定ならこのままでOK

  AddKV(status, "ip_port", std::to_string(ntohs(pkt.port)));
  AddKV(status, "product_name", Fixed8ToString(pkt.product_name));

  const std::string fw =
    std::to_string(pkt.fw_major) + "." +
    std::to_string(pkt.fw_minor) + "." +
    std::to_string(pkt.fw_patch);
  AddKV(status, "firmware_version", fw);

  AddKV(status, "device_id", Fixed8ToString(pkt.device_id));
  AddKV(status, "device_status", std::to_string(static_cast<int>(pkt.device_status)));

  AddKV(status, "device_temperature",
        std::to_string(ntohl(pkt.device_temperature)));  // mdegC

  AddKV(status, "cpu_usage",
        std::to_string(static_cast<int>(pkt.cpu_usage)));

  AddKV(status, "elapsed_time",
        std::to_string(spnet::ntohll(pkt.elapsed_time))); // sec

  AddKV(status, "odometry_state", std::to_string(static_cast<int>(pkt.odometry_state)));
  AddKV(status, "odometry_type",  std::to_string(static_cast<int>(pkt.odometry_type)));
  AddKV(status, "gnss_state",     std::to_string(static_cast<int>(pkt.gnss_state)));
  AddKV(status, "gnss_type",      std::to_string(static_cast<int>(pkt.gnss_type)));

  array.status.push_back(status);
  pub->publish(array);
}

} // namespace hsp