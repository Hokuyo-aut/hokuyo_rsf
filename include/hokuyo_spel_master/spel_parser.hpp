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

#include <iostream>
#include <cstring>
#include <vector>

#include <hokuyo_spel_master/spnet_utils.hpp>

namespace hsp {

class HokuyoSpelParser {
    
 public:
  HokuyoSpelParser();
    
  ~HokuyoSpelParser();

  bool parseOdomPayload(
    const std::vector<uint8_t>& pl,
    spnet::OdomPacket& pkt);

  bool parseHokuyoCloud2Payload(
    const std::vector<uint8_t>& pl,
    spnet::PointCloudPacketHeader& hdr,
    const spnet::PointXYZIT*& points_out);

  bool parseImuPayload(
    const std::vector<uint8_t>& pl,
    spnet::ImuPacket& pkt);

  bool parseNavSatFixPayload(
    const std::vector<uint8_t>& pl,
    spnet::NavSatFixPacket& pkt);

  bool parseGpggaPayload(
    const std::vector<uint8_t>& pl,
    spnet::GpggaPacket& pkt);

  bool parseGprmcPayload(
    const std::vector<uint8_t>& pl,
    spnet::GprmcPacket& pkt);

  bool parseGpzdaPayload(
    const std::vector<uint8_t>& pl,
    spnet::GpzdaPacket& pkt);

  bool parseStringPayload(
    const std::vector<uint8_t>& pl,
    std::string& str);

  bool parseDiagnosticsPayload(
    const std::vector<uint8_t>& pl,
    spnet::DiagnosticPacket& pkt);
}; // class HokuyoSpelParser
    
} // namespace hsp