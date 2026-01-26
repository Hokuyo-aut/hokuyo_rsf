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

#include <hokuyo_spel_master/payload_converter.hpp>

namespace hsp {

PayloadConverter::PayloadConverter() {

}

PayloadConverter::~PayloadConverter() {

}

void PayloadConverter::odomToPayload(
  const nav_msgs::msg::Odometry& odom,
  std::vector<uint8_t>& pl)
{
  spnet::OdomPacket pkt{};
  pkt.x = static_cast<float>(odom.pose.pose.position.x);
  pkt.y = static_cast<float>(odom.pose.pose.position.y);
  pkt.z = static_cast<float>(odom.pose.pose.position.z);
  pkt.qx = static_cast<float>(odom.pose.pose.orientation.x);
  pkt.qy = static_cast<float>(odom.pose.pose.orientation.y);
  pkt.qz = static_cast<float>(odom.pose.pose.orientation.z);
  pkt.qw = static_cast<float>(odom.pose.pose.orientation.w);
  pkt.vx = static_cast<float>(odom.twist.twist.linear.x);
  pkt.vy = static_cast<float>(odom.twist.twist.linear.y);
  pkt.vz = static_cast<float>(odom.twist.twist.linear.z);
  pkt.wx = static_cast<float>(odom.twist.twist.angular.x);
  pkt.wy = static_cast<float>(odom.twist.twist.angular.y);
  pkt.wz = static_cast<float>(odom.twist.twist.angular.z);
  for (size_t i = 0; i < 36; ++i) {
    pkt.pos_cov[i] = odom.pose.covariance[i];
    pkt.vel_cov[i] = odom.twist.covariance[i];
  }

  pl.resize(sizeof(spnet::OdomPacket));
  std::memcpy(pl.data(), &pkt, sizeof(spnet::OdomPacket));
}

void PayloadConverter::hokuyoCloud2ToPayload(
  const sensor_msgs::msg::PointCloud2& cloud,
  std::vector<uint8_t>& pl)
{
  const uint32_t num_points = cloud.width * cloud.height;
  const uint32_t header_size = sizeof(spnet::PointCloudPacketHeader);
  const uint32_t points_size = num_points * sizeof(spnet::PointXYZIT);

  pl.resize(header_size + points_size);

  uint8_t* ptr = pl.data();
  auto* hdr = reinterpret_cast<spnet::PointCloudPacketHeader*>(ptr);
  hdr->num_points = num_points;

  auto* dst_points = reinterpret_cast<spnet::PointXYZIT*>(ptr + header_size);

  const uint32_t point_step = cloud.point_step;
  const uint8_t* src = cloud.data.data();

  int offset_x = -1, offset_y = -1, offset_z = -1;
  int offset_i = -1, offset_s = -1, offset_n = -1;
  for (const auto& f : cloud.fields) {
    if (f.name == "x")              offset_x = f.offset;
    else if (f.name == "y")         offset_y = f.offset;
    else if (f.name == "z")         offset_z = f.offset;
    else if (f.name == "intensity") offset_i = f.offset;
    else if (f.name == "sec")       offset_s = f.offset;
    else if (f.name == "nsec")      offset_n = f.offset;
  }

  if (offset_x < 0 || offset_y < 0 || offset_z < 0) {
    std::cerr << "PointCloud2 does not contain xyz fields in expected form." << std::endl;
    pl.clear();
    return;
  }

  for (uint32_t i = 0; i < num_points; ++i) {
    const uint8_t* p = src + i * point_step;
    spnet::PointXYZIT& dst = dst_points[i];
    std::memcpy(&dst.x,         p + offset_x, sizeof(float));
    std::memcpy(&dst.y,         p + offset_y, sizeof(float));
    std::memcpy(&dst.z,         p + offset_z, sizeof(float));
    std::memcpy(&dst.intensity, p + offset_i, sizeof(float));
    std::memcpy(&dst.sec,       p + offset_s, sizeof(uint32_t));
    std::memcpy(&dst.nsec,      p + offset_n, sizeof(uint32_t));

    // if (offset_i >= 0) {
    //   std::memcpy(&dst.intensity, p + offset_i, sizeof(float));
    // } else {
    //   dst.intensity = 0.0f;
    // }
  }
}

void PayloadConverter::imuToPayload(
  const sensor_msgs::msg::Imu& imu,
  std::vector<uint8_t>& pl)
{
  spnet::ImuPacket pkt{};

  // orientation
  pkt.qx = static_cast<float>(imu.orientation.x);
  pkt.qy = static_cast<float>(imu.orientation.y);
  pkt.qz = static_cast<float>(imu.orientation.z);
  pkt.qw = static_cast<float>(imu.orientation.w);

  // angular velocity
  pkt.wx = static_cast<float>(imu.angular_velocity.x);
  pkt.wy = static_cast<float>(imu.angular_velocity.y);
  pkt.wz = static_cast<float>(imu.angular_velocity.z);

  // linear acceleration
  pkt.ax = static_cast<float>(imu.linear_acceleration.x);
  pkt.ay = static_cast<float>(imu.linear_acceleration.y);
  pkt.az = static_cast<float>(imu.linear_acceleration.z);

  // covariances
  for (size_t i = 0; i < 9; ++i) {
    pkt.ori_cov[i]     = static_cast<float>(imu.orientation_covariance[i]);
    pkt.ang_vel_cov[i] = static_cast<float>(imu.angular_velocity_covariance[i]);
    pkt.lin_acc_cov[i] = static_cast<float>(imu.linear_acceleration_covariance[i]);
  }

  pl.resize(sizeof(spnet::ImuPacket));
  std::memcpy(pl.data(), &pkt, sizeof(spnet::ImuPacket));
}

void PayloadConverter::navSatFixToPayload(
  const sensor_msgs::msg::NavSatFix& msg,
  std::vector<uint8_t>& pl)
{
  spnet::NavSatFixPacket pkt{};
  pkt.latitude  = msg.latitude;
  pkt.longitude = msg.longitude;
  pkt.altitude  = msg.altitude;
  for (size_t i = 0; i < 9; ++i) {
    pkt.pos_cov[i] = msg.position_covariance[i];
  }
  pkt.status_status  = msg.status.status;
  pkt.status_service = msg.status.service;
  pkt.position_covariance_type = msg.position_covariance_type;

  pl.resize(sizeof(spnet::NavSatFixPacket));
  std::memcpy(pl.data(), &pkt, sizeof(spnet::NavSatFixPacket));
}

void PayloadConverter::gpggaToPayload(
  const nmea_msgs::msg::Gpgga& msg,
  std::vector<uint8_t>& pl)
{
  spnet::GpggaPacket pkt{};
  std::memset(pkt.message_id, 0, sizeof(pkt.message_id));
  std::strncpy(pkt.message_id, msg.message_id.c_str(), sizeof(pkt.message_id) - 1);
  
  pkt.utc_seconds = msg.utc_seconds;
  pkt.lat = msg.lat;
  pkt.lon = msg.lon;
  
  pkt.lat_dir          = msg.lat_dir.empty()          ? '\0' : msg.lat_dir[0];
  pkt.lon_dir          = msg.lon_dir.empty()          ? '\0' : msg.lon_dir[0];
  pkt.altitude_units   = msg.altitude_units.empty()   ? '\0' : msg.altitude_units[0];
  pkt.undulation_units = msg.undulation_units.empty() ? '\0' : msg.undulation_units[0];
  
  pkt.gps_qual = msg.gps_qual;
  pkt.num_sats = msg.num_sats;
  
  pkt.hdop       = msg.hdop;
  pkt.alt        = msg.alt;
  pkt.undulation = msg.undulation;
  
  pkt.diff_age = msg.diff_age;
  
  std::memset(pkt.station_id, 0, sizeof(pkt.station_id));
  std::strncpy(pkt.station_id, msg.station_id.c_str(), sizeof(pkt.station_id) - 1);
  
  pl.resize(sizeof(spnet::GpggaPacket));
  std::memcpy(pl.data(), &pkt, sizeof(spnet::GpggaPacket));
}

void PayloadConverter::gprmcToPayload(
  const nmea_msgs::msg::Gprmc& msg,
  std::vector<uint8_t>& pl)
{
  spnet::GprmcPacket pkt{};
  std::memset(&pkt, 0, sizeof(pkt));

  // message_id (string -> char[6])
  std::strncpy(pkt.message_id,
               msg.message_id.c_str(),
               sizeof(pkt.message_id) - 1);

  // numeric fields
  pkt.utc_seconds = msg.utc_seconds;
  pkt.lat         = msg.lat;
  pkt.lon         = msg.lon;
  pkt.speed       = static_cast<double>(msg.speed);
  pkt.track       = static_cast<double>(msg.track);
  pkt.mag_var     = static_cast<double>(msg.mag_var);

  // direction / status / mode (string -> char)
  pkt.lat_dir     = msg.lat_dir.empty() ? '\0' : msg.lat_dir[0];
  pkt.lon_dir     = msg.lon_dir.empty() ? '\0' : msg.lon_dir[0];
  pkt.mag_var_dir = msg.mag_var_direction.empty() ? '\0' : msg.mag_var_direction[0];
  pkt.status      = msg.position_status.empty() ? '\0' : msg.position_status[0];
  pkt.mode        = msg.mode_indicator.empty() ? '\0' : msg.mode_indicator[0];

  // date (string -> char[16])
  std::strncpy(pkt.date,
               msg.date.c_str(),
               sizeof(pkt.date) - 1);

  pl.resize(sizeof(spnet::GprmcPacket));
  std::memcpy(pl.data(), &pkt, sizeof(spnet::GprmcPacket));
}

void PayloadConverter::gpzdaToPayload(
  const nmea_msgs::msg::Gpzda& gpzda,
  std::vector<uint8_t>& pl)
{
  spnet::GpzdaPacket pkt{};

  const std::string mid = gpzda.message_id.empty() ? "GPZDA" : gpzda.message_id;
  std::strncpy(pkt.message_id, mid.c_str(), sizeof(pkt.message_id) - 1);
  
  pkt.utc_seconds       = static_cast<uint32_t>(gpzda.utc_seconds);
  pkt.day               = static_cast<uint8_t>(gpzda.day);
  pkt.month             = static_cast<uint8_t>(gpzda.month);
  pkt.year              = static_cast<uint16_t>(gpzda.year);
  pkt.hour_offset_gmt   = static_cast<int8_t>(gpzda.hour_offset_gmt);
  pkt.minute_offset_gmt = static_cast<uint8_t>(gpzda.minute_offset_gmt);

  pl.resize(sizeof(spnet::GpzdaPacket));
  std::memcpy(pl.data(), &pkt, sizeof(spnet::GpzdaPacket));
}

void PayloadConverter::stringToPayload(
  const std_msgs::msg::String& msg,
  std::vector<uint8_t>& pl)
{
  const std::string& str = msg.data;
  pl.resize(str.size());
  if (!str.empty()) {
    std::memcpy(pl.data(), str.data(), str.size());
  }
}

void PayloadConverter::diagnosticsToPayload(
  const diagnostic_msgs::msg::DiagnosticArray& diag,
  std::vector<uint8_t>& pl,
  const std::string& target_status_name)
{
  // --- 0) init ---
  spnet::DiagnosticPacket pkt{};
  std::memset(&pkt, 0, sizeof(pkt));

  // --- 1) find target status ---
  const diagnostic_msgs::msg::DiagnosticStatus* st_ptr = nullptr;
  for (const auto& st : diag.status) {
    if (st.name == target_status_name) {
      st_ptr = &st;
      break;
    }
  }

  // Not found -> return zero packet
  if (!st_ptr) {
    pl.resize(sizeof(pkt));
    std::memcpy(pl.data(), &pkt, sizeof(pkt));
    return;
  }

  const auto& st = *st_ptr;
  const auto kv = ToMap(st);

  // --- helper lambdas (local) ---
  auto getStr = [&](const char* key, const std::string& def = std::string()) -> std::string {
    auto it = kv.find(key);
    if (it == kv.end()) return def;
    return Trim(it->second);
  };

  auto copyFixed8 = [](char (&dst)[8], const std::string& src) {
    std::memset(dst, 0, sizeof(dst));
    std::memcpy(dst, src.data(), std::min(src.size(), sizeof(dst)));
  };

  // --- 2) ip_address (network order) ---
  {
    const std::string ip = getStr("ip_address");
    in_addr addr{};
    if (!ip.empty() && ::inet_pton(AF_INET, ip.c_str(), &addr) == 1) {
      // addr.s_addr is already network byte order
      pkt.ip_address = addr.s_addr;
    } else {
      pkt.ip_address = 0;  // 0.0.0.0
    }
  }

  // --- 3) port (host -> network) ---
  {
    uint16_t port_host = 0;
    const std::string s = getStr("ip_port");
    if (!s.empty()) {
      ParseU16(s, port_host);
    }
    pkt.port = htons(port_host);
  }

  // --- 4) product_name (8 bytes) ---
  copyFixed8(pkt.product_name, getStr("product_name"));

  // --- 5) firmware_version "a.b.c" ---
  {
    uint8_t a=0,b=0,c=0;
    const std::string s = getStr("firmware_version");
    if (!s.empty() && ParseSemver3(s, a, b, c)) {
      pkt.fw_major = a;
      pkt.fw_minor = b;
      pkt.fw_patch = c;
    } else {
      pkt.fw_major = pkt.fw_minor = pkt.fw_patch = 0;
    }
  }

  // --- 6) device_id (8 bytes): prefer "device_id", fallback to st.hardware_id ---
  {
    std::string id = getStr("device_id");
    if (id.empty()) id = st.hardware_id;
    copyFixed8(pkt.device_id, id);
  }

  // --- 7) device_status (uint8) ---
  {
    uint8_t v = 0;
    const std::string s = getStr("device_status");
    if (!s.empty() && ParseU8(s, v)) pkt.device_status = v;
    else pkt.device_status = 0;
  }

  // --- 8) device_temperature (mdegC) host -> network u32 ---
  {
    uint32_t v = 0;
    const std::string s = getStr("device_temperature");
    if (!s.empty() && ParseU32(s, v)) {
      pkt.device_temperature = htonl(v);
    } else {
      pkt.device_temperature = htonl(0);
    }
  }

  // --- 9) cpu_usage (uint8) ---
  {
    uint8_t v = 0;
    const std::string s = getStr("cpu_usage");
    if (!s.empty() && ParseU8(s, v)) pkt.cpu_usage = v;
    else pkt.cpu_usage = 0;
  }

  // --- 10) elapsed_time (sec) host -> network u64 ---
  {
    uint64_t v = 0;
    const std::string s = getStr("elapsed_time");
    if (!s.empty() && ParseU64(s, v)) pkt.elapsed_time = spnet::htonll(v);
    else pkt.elapsed_time = spnet::htonll(0);
  }

  // --- 11) odometry/gnss states ---
  {
    uint8_t v = 0;

    const std::string s1 = getStr("odometry_state");
    pkt.odometry_state = (!s1.empty() && ParseU8(s1, v)) ? v : 0;

    const std::string s2 = getStr("odometry_type");
    pkt.odometry_type  = (!s2.empty() && ParseU8(s2, v)) ? v : 0;

    const std::string s3 = getStr("gnss_state");
    pkt.gnss_state     = (!s3.empty() && ParseU8(s3, v)) ? v : 0;

    const std::string s4 = getStr("gnss_type");
    pkt.gnss_type      = (!s4.empty() && ParseU8(s4, v)) ? v : 0;
  }

  // --- 12) serialize ---
  pl.resize(sizeof(pkt));
  std::memcpy(pl.data(), &pkt, sizeof(pkt));
}

} // namespace hsp