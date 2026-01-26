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

#include <hokuyo_spel_master/spel_parser.hpp>

namespace hsp {

HokuyoSpelParser::HokuyoSpelParser() {

}

HokuyoSpelParser::~HokuyoSpelParser() {

}

bool HokuyoSpelParser::parseOdomPayload(
  const std::vector<uint8_t>& pl,
  spnet::OdomPacket& pkt)
{
  if (pl.size() != sizeof(spnet::OdomPacket)) {
    std::cerr << "Unexpected OdomPacket size: " << pl.size()
              << " expected: " << sizeof(spnet::OdomPacket) << std::endl;
    return false;
  }

  std::memcpy(&pkt, pl.data(), sizeof(spnet::OdomPacket));
  return true;
}

bool HokuyoSpelParser::parseHokuyoCloud2Payload(
  const std::vector<uint8_t>& pl,
  spnet::PointCloudPacketHeader& hdr,
  const spnet::PointXYZIT*& points_out)
{
  if (pl.size() < sizeof(spnet::PointCloudPacketHeader)) {
    std::cerr << "PointCloud payload too small.\n";
    return false;
  }

  const uint8_t* ptr = pl.data();
  std::memcpy(&hdr, ptr, sizeof(spnet::PointCloudPacketHeader));

  const uint32_t num_points = hdr.num_points;
  const size_t expected_size = sizeof(spnet::PointCloudPacketHeader) +
                               static_cast<size_t>(num_points) * sizeof(spnet::PointXYZIT);

  if (pl.size() != expected_size) {
    std::cerr << "PointCloud payload size mismatch: got "
              << pl.size() << " expected " << expected_size << "\n";
    return false;
  }

  points_out = reinterpret_cast<const spnet::PointXYZIT*>(
    ptr + sizeof(spnet::PointCloudPacketHeader));

  return true;
}

bool HokuyoSpelParser::parseImuPayload(
  const std::vector<uint8_t>& pl,
  spnet::ImuPacket& pkt)
{
  if (pl.size() != sizeof(spnet::ImuPacket)) {
    std::cerr << "Unexpected ImuPacket size: " << pl.size()
              << " expected: " << sizeof(spnet::ImuPacket) << std::endl;
    return false;
  }

  std::memcpy(&pkt, pl.data(), sizeof(spnet::ImuPacket));
  return true;
}

bool HokuyoSpelParser::parseNavSatFixPayload(
  const std::vector<uint8_t>& pl,
  spnet::NavSatFixPacket& pkt)
{
  if (pl.size() != sizeof(spnet::NavSatFixPacket)) {
    std::cerr << "Invalid NavSatFix payload size: " << pl.size()
              << " (expected " << sizeof(spnet::NavSatFixPacket) << ")." << std::endl;
    return false;
  }

  std::memcpy(&pkt, pl.data(), sizeof(spnet::NavSatFixPacket));
  return true;
}

bool HokuyoSpelParser::parseGpggaPayload(
  const std::vector<uint8_t>& pl,
  spnet::GpggaPacket& pkt)
{
  if (pl.size() != sizeof(spnet::GpggaPacket)) {
    std::cerr << "Invalid GPGGA payload size: " << pl.size()
              << " (expected " << sizeof(spnet::GpggaPacket) << ")." << std::endl;
    return false;
  }

  std::memcpy(&pkt, pl.data(), sizeof(spnet::GpggaPacket));
  return true;
}

bool HokuyoSpelParser::parseGprmcPayload(
  const std::vector<uint8_t>& pl,
  spnet::GprmcPacket& pkt)
{
  if (pl.size() != sizeof(spnet::GprmcPacket)) {
    std::cerr << "Unexpected GprmcPacket size: " << pl.size()
              << " expected: " << sizeof(spnet::GprmcPacket) << std::endl;
    return false;
  }

  std::memcpy(&pkt, pl.data(), sizeof(spnet::GprmcPacket));
  return true;
}

bool HokuyoSpelParser::parseGpzdaPayload(
  const std::vector<uint8_t>& pl,
  spnet::GpzdaPacket& pkt)
{
  if (pl.size() != sizeof(spnet::GpzdaPacket)) {
    std::cerr << "Unexpected GpzdaPacket size: " << pl.size()
              << " expected: " << sizeof(spnet::GpzdaPacket) << std::endl;
    return false;
  }

  std::memcpy(&pkt, pl.data(), sizeof(spnet::GpzdaPacket));
  return true;
}

bool HokuyoSpelParser::parseStringPayload(
  const std::vector<uint8_t>& pl,
  std::string& str)
{
  constexpr size_t kMaxStringBytes = 1024 * 1024;  // 1MB
  if (pl.size() > kMaxStringBytes) {
    std::cerr << "Unexpected String payload size: " << pl.size()
              << " (too large)" << std::endl;
    return false;
  }

  str.assign(reinterpret_cast<const char*>(pl.data()), pl.size());
  return true;
}

bool HokuyoSpelParser::parseDiagnosticsPayload(
  const std::vector<uint8_t>& pl,
  spnet::DiagnosticPacket& pkt)
{
  if (pl.size() != sizeof(spnet::DiagnosticPacket)) {
    std::cerr << "Unexpected DiagnosticPacket size: " << pl.size()
              << " expected: " << sizeof(spnet::DiagnosticPacket) << std::endl;
    return false;
  }

  std::memcpy(&pkt, pl.data(), sizeof(spnet::DiagnosticPacket));
  return true;
}

} // namespace hsp