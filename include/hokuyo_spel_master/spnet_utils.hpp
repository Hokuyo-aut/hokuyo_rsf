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

#pragma once

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>

#include <cstdint>
#include <cstddef>
#include <chrono>

namespace spnet {

// ******************** Static parameters ********************

constexpr uint16_t SP_HEAD = 0x5350;



// ******************** Types ********************

enum class MsgType : uint8_t {
  ACK   = 1, // response for CMD (success/fail)
  CMD   = 2, // command request
  DATA  = 3, // streaming data
  WARN  = 4, // small protocol/system error (optional)
  ERROR = 5, // fatal protocol/system error (optional)
};

enum class CmdType : uint8_t {
  START_STREAMING = 1,
  STOP_STREAMING  = 2,
  START_RSF       = 3,
  STOP_RSF        = 4,
  RESET_RSF       = 5,
  SET_IP_ADDRESS  = 6,
};

enum class DataType : uint8_t {
  NAV_SAT_FIX        = 1,
  GPGGA              = 2,
  GPRMC              = 3,
  GPZDA              = 4,
  HOKUYO_CLOUD2      = 5,
  IMU                = 6,
  IMU_RATE_ODOMETRY  = 7,
  NAV_SAT_FIX_SWITCH = 8,
  UTM_ODOM           = 9,
  SWITCH_ODOM        = 10,
  SWITCH_ODOM_STATE  = 11,
  SWITCH_ODOM_TYPE   = 12,
  SWITCH_FIX_STATE   = 13,
  SWITCH_FIX_TYPE    = 14,
  DIAGNOSTIC_ARRAY   = 15,
};



// ******************** Header ********************

struct Header {
  uint16_t head;       // net ('SP' = 0x5350)
  uint8_t  type;       // MsgType
  uint8_t  subtype;    // CmdType or DataType (depends on type)
  uint32_t sec;        // net (measurement time; 0 if not applicable)
  uint32_t nsec;       // net (measurement time; 0 if not applicable)
  uint32_t seq;        // net (request/response correlation)
  uint32_t len;        // net (sizeof(Header) + payload_size + 2)
} __attribute__((packed));



// ******************** Packets ********************

#pragma pack(push, 1)
struct OdomPacket {
  float x, y, z;
  float qx, qy, qz, qw;
  float pos_cov[36];
  float vx, vy, vz;
  float wx, wy, wz;
  float vel_cov[36];
};
#pragma pack(pop)

#pragma pack(push, 1)
struct PointXYZIT {
  float x;
  float y;
  float z;
  float intensity;
  uint32_t sec;
  uint32_t nsec;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct PointCloudPacketHeader {
  uint32_t num_points;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct NavSatFixPacket {
  double latitude;
  double longitude;
  double altitude;
  double pos_cov[9];
  int8_t  status_status;   // NavSatStatus::status
  uint16_t status_service; // NavSatStatus::service
  uint8_t position_covariance_type;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct GpggaPacket {
  char message_id[6];
  double utc_seconds;
  double lat;
  double lon;
  char lat_dir;
  char lon_dir;
  char altitude_units;
  char undulation_units;
  uint32_t gps_qual;
  uint32_t num_sats;
  float hdop;
  float alt;
  float undulation;
  uint32_t diff_age;
  char station_id[8];
};
#pragma pack(pop)

#pragma pack(push, 1)
struct GprmcPacket {
  char message_id[6];
  double utc_seconds;
  double lat;
  double lon;
  char lat_dir;
  char lon_dir;
  double speed;   // knots
  double track;   // degrees
  char date[16];  // string (e.g., "291125" etc) 余裕見て16
  double mag_var; // magnetic variation (dirが無い版)
  char mag_var_dir;
  char status;
  char mode;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct GpzdaPacket {
  char     message_id[6];    // "GPZDA" + '\0'
  uint32_t utc_seconds;      // msg.utc_seconds
  uint8_t  day;              // msg.day
  uint8_t  month;            // msg.month
  uint16_t year;             // msg.year
  int8_t   hour_offset_gmt;  // msg.hour_offset_gmt
  uint8_t  minute_offset_gmt;// msg.minute_offset_gmt
};
#pragma pack(pop)

#pragma pack(push, 1)
struct ImuPacket {
  float qx, qy, qz, qw;     // orientation
  float wx, wy, wz;         // angular velocity
  float ax, ay, az;         // linear acceleration
  float ori_cov[9];         // orientation covariance
  float ang_vel_cov[9];     // angular velocity covariance
  float lin_acc_cov[9];     // linear acceleration covariance
};
#pragma pack(pop)

#pragma pack(push, 1)
struct DiagnosticPacket {
  uint32_t ip_address;          // network byte order (recommended)
  uint16_t port;               // network byte order (recommended)
  char     product_name[8];     // fixed 8 bytes (no guarantee of '\0')
  uint8_t  fw_major;
  uint8_t  fw_minor;
  uint8_t  fw_patch;
  char     device_id[8];        // fixed 8 bytes (no guarantee of '\0')
  uint8_t  device_status;
  uint32_t device_temperature;  // e.g., milli-degree C
  uint8_t  cpu_usage;           // 0-100
  uint64_t elapsed_time;        // seconds (recommend network order? see note)
  uint8_t  odometry_state;
  uint8_t  odometry_type;
  uint8_t  gnss_state;
  uint8_t  gnss_type;
};
#pragma pack(pop)


// ******************** Functions ********************

static inline MsgType toMsgType(uint8_t v) {
  return static_cast<MsgType>(v);
}

static inline CmdType toCmdType(uint8_t v) {
  return static_cast<CmdType>(v);
}

static inline DataType toDataType(uint8_t v) {
  return static_cast<DataType>(v);
}

/*
static inline uint64_t htonll(uint64_t x) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  return (((uint64_t)htonl(x & 0xFFFFFFFFULL)) << 32) | htonl(x >> 32);
#else
  return x;
#endif
}
*/

static inline uint64_t htonll(uint64_t x) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  return (static_cast<uint64_t>(htonl(static_cast<uint32_t>(x & 0xFFFFFFFFULL))) << 32) |
         htonl(static_cast<uint32_t>(x >> 32));
#else
  return x;
#endif
}

static inline uint64_t ntohll(uint64_t x) {
  return htonll(x);
}

static inline ssize_t sendAll(
  int sock,
  const void* buf,
  size_t len)
{
  const uint8_t* p = static_cast<const uint8_t*>(buf);
  size_t off = 0;
  while (off < len) {
    const ssize_t n = ::send(sock, p + off, len - off, 0);
    if (n <= 0) {
      return n;
    }
    off += (size_t)n;
  }
  return (ssize_t)off;
}

/*
static inline ssize_t recvAll(
  int sock,
  void* buf,
  size_t len)
{
  uint8_t* p = static_cast<uint8_t*>(buf);
  size_t off = 0;
  while (off < len) {
    const ssize_t n = ::recv(sock, p + off, len - off, 0);
    if (n <= 0) {
      return n;
    }
    off += (size_t)n;
  }
  return (ssize_t)off;
}
*/

static inline ssize_t recvAll(int sock, void* buf, size_t len) {
  uint8_t* p = static_cast<uint8_t*>(buf);
  size_t off = 0;

  while (off < len) {
    const ssize_t n = ::recv(sock, p + off, len - off, 0);

    if (n == 0) {
      std::cerr << "recv(): peer closed connection (FIN)." << std::endl;
      return 0;
    }
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      std::cerr << "recv(): " << std::strerror(errno)
                << " (errno=" << errno << ")" << std::endl;
      return -1;
    }

    off += static_cast<size_t>(n);
  }

  return static_cast<ssize_t>(off);
}

static inline uint16_t crc16_ccitt_update(
  uint16_t crc,
  const uint8_t* data,
  size_t len)
{
  while (len--) {
    crc ^= static_cast<uint16_t>(*data++) << 8;
    for (int i = 0; i < 8; ++i) {
      if (crc & 0x8000) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

static inline uint16_t crc16_ccitt(
  const void* data,
  size_t len)
{
  const auto* p = static_cast<const uint8_t*>(data);
  uint16_t crc = 0xFFFF;
  return crc16_ccitt_update(crc, p, len);
}



// ******************** Interface functions ********************

static bool Ipv4StringToUint32(
  const std::string& ip,
  uint32_t& out_net)
{
  in_addr addr{};
  const int ret = ::inet_pton(AF_INET, ip.c_str(), &addr);
  if (ret != 1) {
    return false;  // ret==0: invalid, ret==-1: AF not supported etc.
  }

  // addr.s_addr is network byte order already
  out_net = static_cast<uint32_t>(addr.s_addr);
  return true;
}

static inline bool sendFrame(
  int sock,
  uint8_t type,
  uint8_t subtype,
  uint32_t sec,
  uint32_t nsec,
  uint32_t& seq_counter,
  const void* payload,
  uint32_t payload_size)
{
  Header h{};
  const uint32_t total = static_cast<uint32_t>(sizeof(Header)) + payload_size + 2;

  h.head    = htons(SP_HEAD);
  h.type    = type;
  h.subtype = subtype;
  h.sec     = htonl(sec);
  h.nsec    = htonl(nsec);
  h.len     = htonl(total);
  h.seq     = htonl(seq_counter++);

  uint16_t crc = 0xFFFF;
  crc = crc16_ccitt_update(crc, reinterpret_cast<const uint8_t*>(&h), sizeof(h));
  if (payload_size > 0 && payload != nullptr) {
    crc = crc16_ccitt_update(
      crc, reinterpret_cast<const uint8_t*>(payload), payload_size
    );
  }

  const uint16_t crc_net = htons(crc);

  if (sendAll(sock, &h, sizeof(h)) <= 0) {
    std::cerr << "Failed to send header." << std::endl;
    return false;
  }
  if (payload_size > 0 && payload != nullptr) {
    if (sendAll(sock, payload, payload_size) <= 0) {
      std::cerr << "Failed to send payload." << std::endl;
      return false;
    }
  }
  if (sendAll(sock, &crc_net, sizeof(crc_net)) <= 0) {
    std::cerr << "Failed to send crc16." << std::endl;
    return false;
  }

  return true;
}

static inline bool recvFrame(int sock, Header& h, std::vector<uint8_t>& payload) {
  if (recvAll(sock, &h, sizeof(h)) <= 0) {
    std::cerr << "Failed to receive header." << std::endl;
    return false;
  }

  if (ntohs(h.head) != SP_HEAD) {
    std::cerr << "The head message is different from " << SP_HEAD << "." << std::endl;
    return false;
  }

  const uint32_t total = ntohl(h.len);

  if (total < static_cast<uint32_t>(sizeof(Header)) + 0) {
    std::cerr << "The receive data size is too small." << std::endl;
    return false;
  }

  const uint32_t paylen = total - static_cast<uint32_t>(sizeof(Header)) - 2;
  // Optional
  // if (paylen > kMaxPayload) {
  //   std::cerr << "The receive data size is quite large." << std::endl;
  //   return false;
  // }

  payload.resize(paylen);
  if (paylen > 0) {
    if (recvAll(sock, payload.data(), paylen) <= 0) {
      std::cerr << "Failed to receive payload." << std::endl;
      return false;
    }
  }

  uint16_t crc_net = 0;
  if (recvAll(sock, &crc_net, sizeof(crc_net)) <= 0) {
    std::cerr << "Failed to receive crc16." << std::endl;
    return false;
  }
  const uint16_t crc_recv = ntohs(crc_net);

  uint16_t crc_calc = 0xFFFF;
  crc_calc = crc16_ccitt_update(crc_calc, reinterpret_cast<const uint8_t*>(&h), sizeof(h));

  if (paylen > 0) {
    crc_calc = crc16_ccitt_update(crc_calc, payload.data(), paylen);
  }

  if (crc_calc != crc_recv) {
    std::cerr << "CRC16 mismatches. Received: " << crc_recv
      << " Calculated: " << crc_calc << std::endl;
    return false;
  }

  return true;
}

} // namespace hspnet
