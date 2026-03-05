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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nmea_msgs/msg/gpgga.hpp>
#include <nmea_msgs/msg/gprmc.hpp>
#include <nmea_msgs/msg/gpzda.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <unistd.h>
#include <signal.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>

#include <mutex>
#include <thread>
#include <atomic>
#include <cstring>
#include <iostream>
#include <vector>
#include <chrono>
#include <cstdint>

#include <hokuyo_spel_master/spnet_utils.hpp>
#include <hokuyo_spel_master/payload_converter.hpp>
#include <hokuyo_spel_master/spel_parser.hpp>
#include <hokuyo_spel_master/spel_publisher.hpp>

// this should be a common file for master, lio, and spel_ros_node
// #include <hokuyo_spel_master/lio_status.hpp>

class HokuyoSpelRosNode : public rclcpp::Node {
 public:
  HokuyoSpelRosNode():
    Node("hokuyo_spel_ros_node")
  {
    is_set_last_imu_rate_odom_ = false;
    // Subscriber
    rclcpp::QoS cmdQos(rclcpp::KeepLast(10));
    cmdQos.reliable();
    cmdQos.transient_local();

    this->declare_parameter<std::string>("cmd_to_spel_topic", "/spel/cmd_to_spel");
    std::string cmdToSpelTopic;
    this->get_parameter("cmd_to_spel_topic", cmdToSpelTopic);
    cmdToSpelSub_ = this->create_subscription<std_msgs::msg::UInt8>(
      cmdToSpelTopic, cmdQos,
      std::bind(&HokuyoSpelRosNode::cmdToSpelCallback, this, std::placeholders::_1));

    this->declare_parameter<std::string>("ip_address_topic", "/spel/ip_address");
    std::string ipAddressTopic;
    this->get_parameter("ip_address_topic", ipAddressTopic);
    ipAddressSub_ = this->create_subscription<std_msgs::msg::String>(
      ipAddressTopic, cmdQos,
      std::bind(&HokuyoSpelRosNode::ipAddressCallback, this, std::placeholders::_1));

    // Publusher
    this->declare_parameter<std::string>("nav_sat_fix_topic", "/spel/nav_sat_fix");
    std::string navSatFixTopic;
    this->get_parameter("nav_sat_fix_topic", navSatFixTopic);
    navSatFixPub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(navSatFixTopic, 100);

    this->declare_parameter<std::string>("gpgga_topic", "/spel/gpgga");
    std::string gpggaTopic;
    this->get_parameter("gpgga_topic", gpggaTopic);
    gpggaPub_ = this->create_publisher<nmea_msgs::msg::Gpgga>(gpggaTopic, 100);

    this->declare_parameter<std::string>("gprmc_topic", "/spel/gprmc");
    std::string gprmcTopic;
    this->get_parameter("gprmc_topic", gprmcTopic);
    gprmcPub_ = this->create_publisher<nmea_msgs::msg::Gprmc>(gprmcTopic, 100);

    this->declare_parameter<std::string>("gpzda_topic", "/spel/gpzda");
    std::string gpzdaTopic;
    this->get_parameter("gpzda_topic", gpzdaTopic);
    gpzdaPub_ = this->create_publisher<nmea_msgs::msg::Gpzda>(gpzdaTopic, 100);

    this->declare_parameter<std::string>("hokuyo_cloud2_topic", "/spel/hokuyo_cloud2");
    std::string hokuyoCloud2Topic;
    this->get_parameter("hokuyo_cloud2_topic", hokuyoCloud2Topic);
    hokuyoCloud2Pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(hokuyoCloud2Topic, 100);

    this->declare_parameter<std::string>("imu_topic", "/spel/imu");
    std::string imuTopic;
    this->get_parameter("imu_topic", imuTopic);
    imuPub_ = this->create_publisher<sensor_msgs::msg::Imu>(imuTopic, 100);

    this->declare_parameter<std::string>("imu_rate_odom_topic", "/spel/imu_rate_odom");
    std::string imuRateOdomTopic;
    this->get_parameter("imu_rate_odom_topic", imuRateOdomTopic);
    imuRateOdomPub_ = this->create_publisher<nav_msgs::msg::Odometry>(imuRateOdomTopic, 100);

    this->declare_parameter<std::string>("lidar_rate_odom_topic", "/spel/lidar_rate_odom");
    std::string lidarRateOdomTopic;
    this->get_parameter("lidar_rate_odom_topic", lidarRateOdomTopic);
    lidarRateOdomPub_ = this->create_publisher<nav_msgs::msg::Odometry>(lidarRateOdomTopic, 100);

    this->declare_parameter<std::string>("nav_sat_fix_switch_topic", "/spel/nav_sat_fix_switch");
    std::string navSatFixSwitchTopic;
    this->get_parameter("nav_sat_fix_switch_topic", navSatFixSwitchTopic);
    navSatFixSwitchPub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(navSatFixSwitchTopic, 100);

    this->declare_parameter<std::string>("utm_odom_topic", "/spel/utm_odom");
    std::string utmOdomTopic;
    this->get_parameter("utm_odom_topic", utmOdomTopic);
    utmOdomPub_ = this->create_publisher<nav_msgs::msg::Odometry>(utmOdomTopic, 100);

    this->declare_parameter<std::string>("switch_odom_topic", "/spel/switch_odom");
    std::string switchOdomTopic;
    this->get_parameter("switch_odom_topic", switchOdomTopic);
    switchOdomPub_ = this->create_publisher<nav_msgs::msg::Odometry>(switchOdomTopic, 100);

    this->declare_parameter<std::string>("switch_odom_state_topic", "/spel/switch_odom_state");
    std::string switchOdomStateTopic;
    this->get_parameter("switch_odom_state_topic", switchOdomStateTopic);
    switchOdomStatePub_ = this->create_publisher<std_msgs::msg::String>(switchOdomStateTopic, 100);

    this->declare_parameter<std::string>("switch_odom_type_topic", "/spel/switch_odom_type");
    std::string switchOdomTypeTopic;
    this->get_parameter("switch_odom_type_topic", switchOdomTypeTopic);
    switchOdomTypePub_ = this->create_publisher<std_msgs::msg::String>(switchOdomTypeTopic, 100);

    this->declare_parameter<std::string>("switch_fix_state_topic", "/spel/switch_fix_state");
    std::string switchFixStateTopic;
    this->get_parameter("switch_fix_state_topic", switchFixStateTopic);
    switchFixStatePub_ = this->create_publisher<std_msgs::msg::String>(switchFixStateTopic, 100);

    this->declare_parameter<std::string>("switch_fix_type_topic", "/spel/switch_fix_type");
    std::string switchFixTypeTopic;
    this->get_parameter("switch_fix_type_topic", switchFixTypeTopic);
    switchFixTypePub_ = this->create_publisher<std_msgs::msg::String>(switchFixTypeTopic, 100);

    this->declare_parameter<std::string>("diagnostics_topic", "/spel/diagnostics");
    std::string diagnosticsTopic;
    this->get_parameter("diagnostics_topic", diagnosticsTopic);
    diagnosticsPub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnosticsTopic, 100);

    // Load SPEL parameters
    this->declare_parameter<std::string>("spel_ip_address", "127.0.0.1");
    this->declare_parameter<int>("spel_port", 10940);
    this->get_parameter("spel_ip_address", spelIpAdress_);
    this->get_parameter("spel_port", spelPort_);
    std::cout << "IP Address: " << spelIpAdress_ << " Port: " << spelPort_ << std::endl;

    this->declare_parameter<bool>("broadcast_tf", true);
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("lidr_frame", "hokuyo3d");
    this->declare_parameter<std::string>("imu_frame", "hokuyo3d_imu");
    this->declare_parameter<std::string>("gnss_frame", "gnss");
    this->declare_parameter<std::string>("utm_frame", "utm/utm_53Z");
    this->get_parameter("broadcast_tf", broadcastTf_);
    this->get_parameter("odom_frame", odomFrame_);
    this->get_parameter("lidr_frame", lidarFrame_);
    this->get_parameter("imu_frame", imuFrame_);
    this->get_parameter("gnss_frame", gnssFrame_);
    this->get_parameter("utm_frame", utmFrame_);
    if (broadcastTf_) {
      tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    // Initialization for socket communication
    setupSocket();
    clientRunning_.store(true);
    clientThread_ = std::thread(&HokuyoSpelRosNode::spelClientLoop, this);
  }

  ~HokuyoSpelRosNode() override {
    HokuyoSpelRosNode::spelClientClose(sock_);

    clientRunning_.store(false);
    ::shutdown(sock_, SHUT_RDWR);
    ::close(sock_);
    if (clientThread_.joinable()) {
      clientThread_.join();
    }
  }

 private:
  void cmdToSpelCallback(const std_msgs::msg::UInt8::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    cmdToSpel_ = *msg;
    cmdToSpelStamp_ = this->now().nanoseconds();
  }

  void ipAddressCallback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    ipAddress_ = *msg;
    ipAddressStamp_ = this->now().nanoseconds();
  }

  void spelClientLoop() {
    std::atomic<bool> run(true);

    while (rclcpp::ok()) {
      std::thread rx(&HokuyoSpelRosNode::spelClientRxLoop, this, sock_, std::ref(run));
      std::thread tx(&HokuyoSpelRosNode::spelClientTxLoop, this, sock_, std::ref(run));
      rx.join();
      run.store(false);
      tx.join();

      if (rclcpp::ok()) {
        cleanupSocket();
        setupSocket();
        run.store(true);
        RCLCPP_INFO(this->get_logger(), "Restart client loop.");
      }
    }

    rclcpp::shutdown();
  }

  void setupSocket() {
    // Ignore broken pipe error
    signal(SIGPIPE, SIG_IGN);

    sock_ = ::socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(spelPort_);
    ::inet_pton(AF_INET, spelIpAdress_.c_str(), &addr.sin_addr);
    if (connect(sock_, (sockaddr*)&addr, sizeof(addr)) < 0) {
      perror("connect");
      exit(1);
    }

    const int yes = 1;
    ::setsockopt(sock_, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
  }

  void cleanupSocket() {
    ::shutdown(sock_, SHUT_RDWR);
    ::close(sock_);
  }

  void spelClientRxLoop(
    int sock,
    std::atomic<bool>& run)
  {
    while (run.load() && rclcpp::ok()) {
      spnet::Header h{};
      std::vector<uint8_t> pl;
      if (!spnet::recvFrame(sock, h, pl)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to receive data from SPEL.");
        break;
      }

      const spnet::MsgType type = spnet::toMsgType(h.type);

      if (type == spnet::MsgType::ERROR) {
        std::string str;
        hspParser_.parseStringPayload(pl, str);
        RCLCPP_ERROR(this->get_logger(), "Got error from SPEL. %s", str.c_str());
        break;
      
      } else if (type == spnet::MsgType::WARN) {
        std::string str;
        hspParser_.parseStringPayload(pl, str);
        RCLCPP_WARN(this->get_logger(), "Got warning from SPEL. %s", str.c_str());

      } else if (type == spnet::MsgType::ACK) {
        RCLCPP_INFO(this->get_logger(), "Got ACK.");

      } else if (type == spnet::MsgType::DATA) {
        const spnet::DataType datatype = spnet::toDataType(h.subtype);
        const uint32_t sec = ntohl(h.sec);
        const uint32_t nsec = ntohl(h.nsec);
        const uint64_t stamp = static_cast<uint64_t>(sec) * 1000000000ULL + static_cast<uint64_t>(nsec);;

        if (datatype == spnet::DataType::NAV_SAT_FIX) {
          spnet::NavSatFixPacket pkt;
          if (!hspParser_.parseNavSatFixPayload(pl, pkt)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse NavSatFix payload.");
            continue;
          }
          hspPublisher_.publishNavSatFix(navSatFixPub_, stamp, gnssFrame_, pkt);

        } else if (datatype == spnet::DataType::GPGGA) {
          spnet::GpggaPacket pkt;
          if (!hspParser_.parseGpggaPayload(pl, pkt)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse GPGGA payload.");
            continue;
          }
          hspPublisher_.publishGpgga(gpggaPub_, stamp, gnssFrame_, pkt);

        } else if (datatype == spnet::DataType::GPRMC) {
          spnet::GprmcPacket pkt;
          if (!hspParser_.parseGprmcPayload(pl, pkt)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse GPRMC payload.");
            continue;
          }
          hspPublisher_.publishGprmc(gprmcPub_, stamp, gnssFrame_, pkt);

        } else if (datatype == spnet::DataType::GPZDA) {
          spnet::GpzdaPacket pkt;
          if (!hspParser_.parseGpzdaPayload(pl, pkt)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse GPZDA payload.");
            continue;
          }
          hspPublisher_.publishGpzda(gpzdaPub_, stamp, gnssFrame_, pkt);

        } else if (datatype == spnet::DataType::HOKUYO_CLOUD2) {
          spnet::PointCloudPacketHeader pcHdr{};
          const spnet::PointXYZIT* points = nullptr;
          if (!hspParser_.parseHokuyoCloud2Payload(pl, pcHdr, points)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse hokuyo cloud2 payload.");
            continue;
          }
          hspPublisher_.publishHokuyoCloud2(hokuyoCloud2Pub_, stamp, lidarFrame_, pcHdr, points);
          if(is_set_last_imu_rate_odom_){
            hspPublisher_.publishOdom(lidarRateOdomPub_, stamp, odomFrame_, lidarFrame_, last_imu_rate_odom_pkt_);
          }
        } else if (datatype == spnet::DataType::IMU) {
          spnet::ImuPacket pkt;
          if (!hspParser_.parseImuPayload(pl, pkt)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse IMU payload.");
            continue;
          }
          hspPublisher_.publishImu(imuPub_, stamp, imuFrame_, pkt);

        } else if (datatype == spnet::DataType::IMU_RATE_ODOMETRY) {
          spnet::OdomPacket pkt;
          if (!hspParser_.parseOdomPayload(pl, pkt)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse IMU rate odometry payload.");
            continue;
          }
          hspPublisher_.publishOdom(imuRateOdomPub_, stamp, odomFrame_, lidarFrame_, pkt);
          last_imu_rate_odom_pkt_ = pkt;
          is_set_last_imu_rate_odom_ = true;

          static int tfcnt = 0;
          tfcnt++;
          if (broadcastTf_ && tfcnt == 50) {
            hspPublisher_.publishTf(tfBroadcaster_, stamp, odomFrame_, lidarFrame_, pkt);
            tfcnt = 0;
          }

        } else if (datatype == spnet::DataType::NAV_SAT_FIX_SWITCH) {
          spnet::NavSatFixPacket pkt;
          if (!hspParser_.parseNavSatFixPayload(pl, pkt)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse NavSatFixSwitch payload.");
            continue;
          }
          hspPublisher_.publishNavSatFix(navSatFixSwitchPub_, stamp, gnssFrame_, pkt);

        } else if (datatype == spnet::DataType::UTM_ODOM) {
          spnet::OdomPacket pkt;
          if (!hspParser_.parseOdomPayload(pl, pkt)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse UTM odometry payload.");
            continue;
          }
          hspPublisher_.publishOdom(utmOdomPub_, stamp, utmFrame_, lidarFrame_, pkt);

        } else if (datatype == spnet::DataType::SWITCH_ODOM) {
          spnet::OdomPacket pkt;
          if (!hspParser_.parseOdomPayload(pl, pkt)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse switch odometry payload.");
            continue;
          }
          hspPublisher_.publishOdom(switchOdomPub_, stamp, odomFrame_, lidarFrame_, pkt);

        } else if (datatype == spnet::DataType::SWITCH_ODOM_STATE) {
          std::string str;
          if (!hspParser_.parseStringPayload(pl, str)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse switch odometry state payload.");
            continue;
          }
          hspPublisher_.publishString(switchOdomStatePub_, str);

        } else if (datatype == spnet::DataType::SWITCH_ODOM_TYPE) {
          std::string str;
          if (!hspParser_.parseStringPayload(pl, str)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse switch odometry type payload.");
            continue;
          }
          hspPublisher_.publishString(switchOdomTypePub_, str);

        } else if (datatype == spnet::DataType::SWITCH_FIX_STATE) {
          std::string str;
          if (!hspParser_.parseStringPayload(pl, str)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse switch fix state payload.");
            continue;
          }
          hspPublisher_.publishString(switchFixStatePub_, str);

        } else if (datatype == spnet::DataType::SWITCH_FIX_TYPE) {
          std::string str;
          if (!hspParser_.parseStringPayload(pl, str)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse switch fix type payload.");
            continue;
          }
          hspPublisher_.publishString(switchFixTypePub_, str);

        } else if (datatype == spnet::DataType::DIAGNOSTIC_ARRAY) {
          spnet::DiagnosticPacket pkt;
          if (!hspParser_.parseDiagnosticsPayload(pl, pkt)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse diagnostics payload.");
            continue;
          }
          hspPublisher_.publishDiagnostics(diagnosticsPub_, stamp, pkt);

        }
      }

    }

    run.store(false);
  }

  void spelClientTxLoop(
    int sock,
    std::atomic<bool>& run)
  {
    using namespace std::chrono_literals;

    uint32_t seq = 0;
    std::vector<uint8_t> payload;
    uint32_t payloadSize = payload.size();

    uint64_t prevCmdToSpelStamp = 0;
    uint64_t prevIpAddressStamp = 0;

    uint8_t type    = static_cast<uint8_t>(spnet::MsgType::CMD);
    uint8_t subtype = static_cast<uint8_t>(spnet::CmdType::START_STREAMING);
    uint64_t stamp  = this->now().nanoseconds();
    uint32_t sec    = static_cast<uint32_t>(stamp / 1000000000ULL);
    uint32_t nsec   = static_cast<uint32_t>(stamp % 1000000000ULL);
    payload.clear();
    payloadSize = payload.size();
    spnet::sendFrame(sock, type, subtype, sec, nsec, seq, payload.data(), payloadSize);
    subtype = static_cast<uint8_t>(spnet::CmdType::START_RSF);
    spnet::sendFrame(sock, type, subtype, sec, nsec, seq, payload.data(), payloadSize);

    while (run.load() && rclcpp::ok()) {
      std::this_thread::sleep_for(1s);

      std_msgs::msg::UInt8 cmdToSpel;
      uint64_t cmdToSpelStamp = 0;
      bool cmdToSpelUpdated = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        cmdToSpelStamp = cmdToSpelStamp_;
        if (cmdToSpelStamp != prevCmdToSpelStamp) {
          cmdToSpel = cmdToSpel_;
          prevCmdToSpelStamp = cmdToSpelStamp;
          cmdToSpelUpdated = true;
        }
      }        
      if (cmdToSpelUpdated) {
        const uint8_t type = static_cast<uint8_t>(spnet::MsgType::CMD);
        uint8_t subtype;
        if (cmdToSpel.data == 1) {
          subtype = static_cast<uint8_t>(spnet::CmdType::START_STREAMING);
          RCLCPP_INFO(this->get_logger(),
            "Hokuyo SPEL ROS2 node sends start streaming command.");
        } else if (cmdToSpel.data == 2) {
          subtype = static_cast<uint8_t>(spnet::CmdType::STOP_STREAMING);
          RCLCPP_INFO(this->get_logger(),
            "Hokuyo SPEL ROS2 node sends stop streaming command.");
        } else if (cmdToSpel.data == 3) {
          subtype = static_cast<uint8_t>(spnet::CmdType::START_RSF);
          RCLCPP_INFO(this->get_logger(),
            "Hokuyo SPEL ROS2 node sends start software command.");
        } else if (cmdToSpel.data == 4) {
          subtype = static_cast<uint8_t>(spnet::CmdType::STOP_RSF);
          RCLCPP_INFO(this->get_logger(),
            "Hokuyo SPEL ROS2 node sends stop software command.");
        } else if (cmdToSpel.data == 5) {
          subtype = static_cast<uint8_t>(spnet::CmdType::RESET_RSF);
          RCLCPP_INFO(this->get_logger(),
            "Hokuyo SPEL ROS2 node sends reset software command.");
        } else {
          subtype = 0;
          RCLCPP_WARN(this->get_logger(),
            "Hokuyo SPEL ROS2 node sends unknown command.");
        }
        const uint32_t sec = static_cast<uint32_t>(cmdToSpelStamp / 1000000000ULL);
        const uint32_t nsec = static_cast<uint32_t>(cmdToSpelStamp % 1000000000ULL);
        payload.clear();
        payloadSize = payload.size();
        if (!spnet::sendFrame(sock, type, subtype, sec, nsec, seq,
          payload.data(), payloadSize))
        {
          RCLCPP_ERROR(this->get_logger(), "Failed to send a command to SPEL.");
          run.store(false);
          break;
        }
      }

      std_msgs::msg::String ipAddress;
      uint64_t ipAddressStamp = 0;
      bool ipAddressUpdated = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        ipAddressStamp = ipAddressStamp_;
        if (ipAddressStamp != prevIpAddressStamp) {
          ipAddress = ipAddress_;
          prevIpAddressStamp = ipAddressStamp;
          ipAddressUpdated = true;
        }
      }
      if (ipAddressUpdated) {
        uint32_t ip;
        if (!spnet::Ipv4StringToUint32(ipAddress.data, ip)) {
          RCLCPP_INFO(this->get_logger(),
            "Hokuyo SPEL ROS2 node receives incorrect IP address: %s", ipAddress.data.c_str());
        } else {
          const uint8_t type = static_cast<uint8_t>(spnet::MsgType::CMD);
          const uint8_t subtype = static_cast<uint8_t>(spnet::CmdType::SET_IP_ADDRESS);
          const uint32_t sec = static_cast<uint32_t>(ipAddressStamp / 1000000000ULL);
          const uint32_t nsec = static_cast<uint32_t>(ipAddressStamp % 1000000000ULL);
          payload.resize(ipAddress.data.size());
          std::memcpy(payload.data(), ipAddress.data.data(), ipAddress.data.size());
          payloadSize = payload.size();
          RCLCPP_INFO(this->get_logger(),
            "Hokuyo SPEL ROS2 node sends IP address: %s", ipAddress.data.c_str());
          if (!spnet::sendFrame(sock, type, subtype, sec, nsec, seq,
            payload.data(), payloadSize))
          {
            RCLCPP_ERROR(this->get_logger(), "Failed to send IP address.");
            run.store(false);
            break;
          }
        }
      }
    }

    run.store(false);
  }

  void spelClientClose(
    int sock)
  {
    uint32_t seq = 0;
    std::vector<uint8_t> payload;
    uint32_t payloadSize = payload.size();

    uint8_t type    = static_cast<uint8_t>(spnet::MsgType::CMD);
    uint8_t subtype = static_cast<uint8_t>(spnet::CmdType::STOP_RSF);
    uint64_t stamp  = this->now().nanoseconds();
    uint32_t sec    = static_cast<uint32_t>(stamp / 1000000000ULL);
    uint32_t nsec   = static_cast<uint32_t>(stamp % 1000000000ULL);
    payload.clear();
    payloadSize = payload.size();
    spnet::sendFrame(sock, type, subtype, sec, nsec, seq, payload.data(), payloadSize);
    subtype = static_cast<uint8_t>(spnet::CmdType::STOP_STREAMING);
    spnet::sendFrame(sock, type, subtype, sec, nsec, seq, payload.data(), payloadSize);
    RCLCPP_INFO(this->get_logger(), "Close RSF connection.");
  }

  int sock_;
  std::thread clientThread_;
  std::atomic<bool> clientRunning_{false};
  uint64_t latestStreamingStamp_;
  spnet::OdomPacket last_imu_rate_odom_pkt_;
  bool is_set_last_imu_rate_odom_;

  std::string spelIpAdress_;
  int spelPort_;
  int spelOdomPort_;

  hsp::PayloadConverter plConv_;
  hsp::HokuyoSpelParser hspParser_;
  hsp::HokuyoSpelPublisher hspPublisher_;

  mutable std::mutex mutex_;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr cmdToSpelSub_;
  std_msgs::msg::UInt8 cmdToSpel_{};
  uint64_t cmdToSpelStamp_{0};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ipAddressSub_;
  std_msgs::msg::String ipAddress_{};
  uint64_t ipAddressStamp_{0};

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navSatFixPub_;
  rclcpp::Publisher<nmea_msgs::msg::Gpgga>::SharedPtr gpggaPub_;
  rclcpp::Publisher<nmea_msgs::msg::Gprmc>::SharedPtr gprmcPub_;
  rclcpp::Publisher<nmea_msgs::msg::Gpzda>::SharedPtr gpzdaPub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr hokuyoCloud2Pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr imuRateOdomPub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr lidarRateOdomPub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navSatFixSwitchPub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr utmOdomPub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr switchOdomPub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr switchOdomStatePub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr switchOdomTypePub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr switchFixStatePub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr switchFixTypePub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnosticsPub_;

  bool broadcastTf_{true};
  std::string odomFrame_;
  std::string lidarFrame_;
  std::string imuFrame_;
  std::string gnssFrame_;
  std::string utmFrame_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
}; // class HokuyoSpelRosNode

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HokuyoSpelRosNode>());
  rclcpp::shutdown();
  return 0;
}
