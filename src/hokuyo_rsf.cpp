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
#include <cerrno>
#include <cmath>
#include <array>
#include <string>
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

    // 受信状況のログ出力
    this->declare_parameter<double>("diag_period", 1.0);
    this->declare_parameter<double>("stall_timeout", 1.0);
    this->get_parameter("diag_period", diagPeriod_);
    this->get_parameter("stall_timeout", stallTimeout_);
    startTime_ = SteadyClock::now();
    lastAnyRx_ = startTime_;
    windowStart_ = startTime_;
    diagTimer_ = this->create_wall_timer(
      std::chrono::duration<double>(diagPeriod_), std::bind(&HokuyoSpelRosNode::diagCallback, this));
    RCLCPP_INFO(this->get_logger(), "起動: RSF %s:%d (diag_period=%.2fs, stall_timeout=%.2fs)",
      spelIpAdress_.c_str(), spelPort_, diagPeriod_, stallTimeout_);

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
    const auto connectStart = SteadyClock::now();
    if (connect(sock_, (sockaddr*)&addr, sizeof(addr)) < 0) {
      const int err = errno;
      perror("connect");
      RCLCPP_FATAL(this->get_logger(),
        "RSF への接続に失敗: %s:%d errno=%d (%s), 接続処理 %.2fs, %s. プロセスを終了する",
        spelIpAdress_.c_str(), spelPort_, err, std::strerror(err),
        elapsed(connectStart, SteadyClock::now()), sinceDisconnectText().c_str());
      rclcpp::shutdown();  // ログをファイルに書き出してから終了する
      exit(1);
    }
    const std::string sinceDisconnect = sinceDisconnectText();
    {
      std::lock_guard<std::mutex> lock(statsMutex_);
      ++connectCount_;
      connectedAt_ = SteadyClock::now();
      connPackets_ = 0;
      RCLCPP_INFO(this->get_logger(), "RSF に接続: %s:%d (接続 #%lu, 接続処理 %.2fs, %s)",
        spelIpAdress_.c_str(), spelPort_, static_cast<unsigned long>(connectCount_),
        elapsed(connectStart, connectedAt_), sinceDisconnect.c_str());
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
    std::string disconnectReason = "停止要求";
    while (run.load() && rclcpp::ok()) {
      spnet::Header h{};
      std::vector<uint8_t> pl;
      errno = 0;
      if (!spnet::recvFrame(sock, h, pl)) {
        const int err = errno;
        RCLCPP_ERROR(this->get_logger(), "Failed to receive data from SPEL.");
        // errno=0 は相手側からの切断、またはフレーム不正 (詳細は標準エラー出力)
        if (!rclcpp::ok()) {
          disconnectReason = "停止要求 (シャットダウン)";
        } else {
          disconnectReason = "受信失敗 errno=" + std::to_string(err) + " (" +
            (err == 0 ? std::string("相手側の切断またはフレーム不正") : std::string(std::strerror(err))) + ")";
        }
        break;
      }
      recordFrame(sizeof(spnet::Header) + pl.size() + 2);

      const spnet::MsgType type = spnet::toMsgType(h.type);

      if (type == spnet::MsgType::ERROR) {
        std::string str;
        hspParser_.parseStringPayload(pl, str);
        RCLCPP_ERROR(this->get_logger(), "Got error from SPEL. %s", str.c_str());
        disconnectReason = "SPEL からエラー: " + str;
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
        recordData(h.subtype, stamp * 1e-9);

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
          checkFixSwitch(pkt, stamp * 1e-9);

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
          checkStateString(static_cast<size_t>(spnet::DataType::SWITCH_ODOM_STATE), str, stamp * 1e-9);

        } else if (datatype == spnet::DataType::SWITCH_ODOM_TYPE) {
          std::string str;
          if (!hspParser_.parseStringPayload(pl, str)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse switch odometry type payload.");
            continue;
          }
          hspPublisher_.publishString(switchOdomTypePub_, str);
          checkStateString(static_cast<size_t>(spnet::DataType::SWITCH_ODOM_TYPE), str, stamp * 1e-9);

        } else if (datatype == spnet::DataType::SWITCH_FIX_STATE) {
          std::string str;
          if (!hspParser_.parseStringPayload(pl, str)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse switch fix state payload.");
            continue;
          }
          hspPublisher_.publishString(switchFixStatePub_, str);
          checkStateString(static_cast<size_t>(spnet::DataType::SWITCH_FIX_STATE), str, stamp * 1e-9);

        } else if (datatype == spnet::DataType::SWITCH_FIX_TYPE) {
          std::string str;
          if (!hspParser_.parseStringPayload(pl, str)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse switch fix type payload.");
            continue;
          }
          hspPublisher_.publishString(switchFixTypePub_, str);
          checkStateString(static_cast<size_t>(spnet::DataType::SWITCH_FIX_TYPE), str, stamp * 1e-9);

        } else if (datatype == spnet::DataType::DIAGNOSTIC_ARRAY) {
          spnet::DiagnosticPacket pkt;
          if (!hspParser_.parseDiagnosticsPayload(pl, pkt)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse diagnostics payload.");
            continue;
          }
          hspPublisher_.publishDiagnostics(diagnosticsPub_, stamp, pkt);
          checkDiagnostics(pkt);

        } else {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "未知の DataType を受信: %u", static_cast<unsigned>(h.subtype));
        }
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "未知の MsgType を受信: %u", static_cast<unsigned>(h.type));
      }

    }

    {
      std::lock_guard<std::mutex> lock(statsMutex_);
      lastDisconnect_ = SteadyClock::now();
      hasDisconnected_ = true;
      RCLCPP_WARN(this->get_logger(), "RSF との受信を終了: 理由=%s, 接続 #%lu の接続時間 %.1fs, 受信フレーム %lu",
        disconnectReason.c_str(), static_cast<unsigned long>(connectCount_),
        elapsed(connectedAt_, lastDisconnect_), static_cast<unsigned long>(connPackets_));
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

  // ******************** 受信状況のログ出力 ********************
  // RSF から何がいつ届いたかを記録し、途絶・状態変化を即時に、受信状況を diag_period ごとに出す.

  using SteadyClock = std::chrono::steady_clock;  // PC の時刻補正の影響を受けないように steady を使う

  struct RxStats {
    uint64_t total = 0;
    uint64_t window = 0;
    uint64_t backward = 0;
    SteadyClock::time_point lastRx{};
    double lastStamp = 0.0;
    double emaInterval = 0.0;  // 受信間隔の移動平均 [s]
    bool stalled = false;
  };

  struct DiagState {
    bool valid = false;
    uint8_t deviceStatus = 0;
    uint8_t odometryState = 0;
    uint8_t odometryType = 0;
    uint8_t gnssState = 0;
    uint8_t gnssType = 0;
    double temperature = 0.0;  // [℃]
    int cpuUsage = 0;
  };

  static constexpr size_t kNumDataTypes = 16;

  static const char* dataTypeName(size_t i) {
    static const char* names[kNumDataTypes] = {
      "?", "fix", "gga", "rmc", "zda", "cloud", "imu", "imu_odom",
      "fix_sw", "utm_odom", "sw_odom", "sw_odom_state", "sw_odom_type",
      "sw_fix_state", "sw_fix_type", "diag"};
    return i < kNumDataTypes ? names[i] : "?";
  }

  static double elapsed(SteadyClock::time_point from, SteadyClock::time_point to) {
    return std::chrono::duration<double>(to - from).count();
  }

  std::string sinceDisconnectText() {
    std::lock_guard<std::mutex> lock(statsMutex_);
    if (!hasDisconnected_) {
      return "初回の接続";
    }
    char buf[64];
    std::snprintf(buf, sizeof(buf), "前回の切断から %.1fs", elapsed(lastDisconnect_, SteadyClock::now()));
    return buf;
  }

  // フレームを 1 つ受信するたびに呼ぶ (種別を問わない).
  void recordFrame(size_t bytes) {
    const auto now = SteadyClock::now();
    std::lock_guard<std::mutex> lock(statsMutex_);
    if (allStalled_) {
      RCLCPP_WARN(this->get_logger(), "RSF からの受信が再開: %.2fs 何も受信していなかった",
        elapsed(lastAnyRx_, now));
      allStalled_ = false;
    }
    lastAnyRx_ = now;
    windowBytes_ += bytes;
    ++connPackets_;
  }

  // DATA フレームを受信するたびに呼ぶ.
  void recordData(uint8_t subtype, double stamp) {
    const auto now = SteadyClock::now();
    const double latency = this->now().seconds() - stamp;
    std::lock_guard<std::mutex> lock(statsMutex_);
    if (subtype == 0 || subtype >= kNumDataTypes) {
      return;
    }
    auto& s = rxStats_[subtype];
    const char* name = dataTypeName(subtype);
    if (s.total == 0) {
      RCLCPP_INFO(this->get_logger(), "[%s] 初回受信: stamp=%.3f now-stamp=%+.3fs", name, stamp, latency);
    } else {
      const double gap = elapsed(s.lastRx, now);
      if (s.stalled) {
        RCLCPP_WARN(this->get_logger(), "[%s] 受信再開: %.2fs 途絶していた (stamp %.3f -> %.3f, now-stamp=%+.3fs)",
          name, gap, s.lastStamp, stamp, latency);
        s.stalled = false;
      } else {
        s.emaInterval = (s.total == 1) ? gap : 0.95 * s.emaInterval + 0.05 * gap;
      }
      if (stamp < s.lastStamp) {
        ++s.backward;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "[%s] スタンプが巻き戻った: %.3f -> %.3f (%+.3fs)", name, s.lastStamp, stamp, stamp - s.lastStamp);
      }
    }
    s.lastRx = now;
    s.lastStamp = stamp;
    ++s.total;
    ++s.window;
    lastLatency_ = latency;
  }

  void checkFixSwitch(const spnet::NavSatFixPacket& pkt, double stamp) {
    std::lock_guard<std::mutex> lock(statsMutex_);
    // NavSatStatus: -1=NO_FIX, 0=FIX, 1=SBAS_FIX, 2=GBAS_FIX
    if (pkt.status_status != lastFixStatus_) {
      if (pkt.status_status < 0) {
        RCLCPP_WARN(this->get_logger(), "[fix_sw] status 変化: %d -> %d (stamp=%.3f)",
          lastFixStatus_, pkt.status_status, stamp);
      } else {
        RCLCPP_INFO(this->get_logger(), "[fix_sw] status 変化: %d -> %d (stamp=%.3f)",
          lastFixStatus_, pkt.status_status, stamp);
      }
      lastFixStatus_ = pkt.status_status;
    }
    if (!std::isfinite(pkt.latitude) || !std::isfinite(pkt.longitude) || !std::isfinite(pkt.altitude)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "[fix_sw] NaN/Inf を受信: lat=%f lon=%f alt=%f (stamp=%.3f)",
        pkt.latitude, pkt.longitude, pkt.altitude, stamp);
    }
  }

  void checkStateString(size_t subtype, const std::string& str, double stamp) {
    std::lock_guard<std::mutex> lock(statsMutex_);
    auto& last = lastStrings_[subtype];
    if (str != last) {
      RCLCPP_INFO(this->get_logger(), "[%s] 変化: '%s' -> '%s' (stamp=%.3f)",
        dataTypeName(subtype), last.c_str(), str.c_str(), stamp);
      last = str;
    }
  }

  void checkDiagnostics(const spnet::DiagnosticPacket& pkt) {
    DiagState d;
    d.valid = true;
    d.deviceStatus = pkt.device_status;
    d.odometryState = pkt.odometry_state;
    d.odometryType = pkt.odometry_type;
    d.gnssState = pkt.gnss_state;
    d.gnssType = pkt.gnss_type;
    d.temperature = static_cast<int32_t>(ntohl(pkt.device_temperature));  // 実機の値は ℃ 単位 (64 など)
    d.cpuUsage = pkt.cpu_usage;

    std::lock_guard<std::mutex> lock(statsMutex_);
    const auto logChange = [this](const char* key, int before, int after, bool valid) {
      if (valid && before == after) {
        return;
      }
      const std::string beforeText = valid ? std::to_string(before) : std::string("未受信");
      RCLCPP_INFO(this->get_logger(), "[diag] %s 変化: %s -> %d", key, beforeText.c_str(), after);
    };
    if (d.valid && (!diag_.valid || d.deviceStatus != diag_.deviceStatus) && d.deviceStatus != 0) {
      RCLCPP_WARN(this->get_logger(), "[diag] device_status が異常: %d", d.deviceStatus);
    }
    logChange("device_status", diag_.deviceStatus, d.deviceStatus, diag_.valid);
    logChange("odometry_state", diag_.odometryState, d.odometryState, diag_.valid);
    logChange("odometry_type", diag_.odometryType, d.odometryType, diag_.valid);
    logChange("gnss_state", diag_.gnssState, d.gnssState, diag_.valid);
    logChange("gnss_type", diag_.gnssType, d.gnssType, diag_.valid);
    diag_ = d;
  }

  // diag_period ごとに呼ぶ. 途絶を検知し、受信状況を 1 行にまとめて出す.
  // 受信スレッドが recv で止まっていても動くので、全データの途絶も検知できる.
  void diagCallback() {
    const auto now = SteadyClock::now();
    std::vector<std::string> warnings;
    std::string summary;
    {
      std::lock_guard<std::mutex> lock(statsMutex_);
      const double period = std::max(elapsed(windowStart_, now), 1e-3);
      const double anyAge = elapsed(lastAnyRx_, now);
      const auto ageOf = [&](spnet::DataType t) {
        const auto& s = rxStats_[static_cast<size_t>(t)];
        return s.total == 0 ? -1.0 : elapsed(s.lastRx, now);
      };
      char buf[512];

      if (!allStalled_ && anyAge > stallTimeout_) {
        allStalled_ = true;
        std::snprintf(buf, sizeof(buf),
          "RSF から何も受信していない: %.2fs (接続 #%lu, 受信待ちで停止している可能性)",
          anyAge, static_cast<unsigned long>(connectCount_));
        warnings.emplace_back(buf);
      }

      for (size_t i = 1; i < kNumDataTypes; ++i) {
        auto& s = rxStats_[i];
        if (s.total < 3 || s.stalled) {
          continue;
        }
        const double age = elapsed(s.lastRx, now);
        const double threshold = std::max(stallTimeout_, 5.0 * s.emaInterval);
        if (age > threshold) {
          s.stalled = true;
          std::snprintf(buf, sizeof(buf),
            "[%s] 受信が途絶: 最終受信から %.2fs (通常の間隔 %.3fs), 最終stamp=%.3f | "
            "他: cloud age=%.2fs, 全体の最終受信 %.2fs前",
            dataTypeName(i), age, s.emaInterval, s.lastStamp,
            ageOf(spnet::DataType::HOKUYO_CLOUD2), anyAge);
          warnings.emplace_back(buf);
        }
      }

      std::string counts;
      for (size_t i = 1; i < kNumDataTypes; ++i) {
        const auto& s = rxStats_[i];
        if (s.total == 0) {
          continue;
        }
        counts += " " + std::string(dataTypeName(i)) + "=" + std::to_string(s.window);
        if (s.backward > 0) {
          counts += "(backward " + std::to_string(s.backward) + ")";
        }
        if (s.stalled) {
          counts += "(途絶中)";
        }
      }
      std::snprintf(buf, sizeof(buf),
        "接続 #%lu up=%.0fs rx=%.0fKB/s any_age=%.2fs | age fix_sw=%.2f cloud=%.2f utm=%.2f sw_odom=%.2f | "
        "now-stamp=%+.3fs | fix_status=%d",
        static_cast<unsigned long>(connectCount_), elapsed(connectedAt_, now),
        windowBytes_ / 1024.0 / period, anyAge,
        ageOf(spnet::DataType::NAV_SAT_FIX_SWITCH), ageOf(spnet::DataType::HOKUYO_CLOUD2),
        ageOf(spnet::DataType::UTM_ODOM), ageOf(spnet::DataType::SWITCH_ODOM),
        lastLatency_, lastFixStatus_);
      summary = buf;
      summary += " | count" + (counts.empty() ? std::string(" なし") : counts);
      summary += " | sw_odom_type='" + lastStrings_[static_cast<size_t>(spnet::DataType::SWITCH_ODOM_TYPE)] +
        "' sw_fix_state='" + lastStrings_[static_cast<size_t>(spnet::DataType::SWITCH_FIX_STATE)] + "'";
      if (diag_.valid) {
        std::snprintf(buf, sizeof(buf), " | diag dev=%d odom=%d/%d gnss=%d/%d temp=%.0fC cpu=%d%%",
          diag_.deviceStatus, diag_.odometryState, diag_.odometryType, diag_.gnssState, diag_.gnssType,
          diag_.temperature, diag_.cpuUsage);
        summary += buf;
      }

      for (auto& s : rxStats_) {
        s.window = 0;
        s.backward = 0;
      }
      windowBytes_ = 0;
      windowStart_ = now;
    }
    for (const auto& w : warnings) {
      RCLCPP_WARN(this->get_logger(), "%s", w.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "%s", summary.c_str());
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

  // 受信状況のログ出力
  double diagPeriod_{1.0};
  double stallTimeout_{1.0};
  rclcpp::TimerBase::SharedPtr diagTimer_;
  std::mutex statsMutex_;
  std::array<RxStats, kNumDataTypes> rxStats_{};
  std::array<std::string, kNumDataTypes> lastStrings_{};
  DiagState diag_{};
  int lastFixStatus_{-128};  // 未受信
  double lastLatency_{0.0};
  SteadyClock::time_point startTime_{};
  SteadyClock::time_point lastAnyRx_{};
  SteadyClock::time_point windowStart_{};
  SteadyClock::time_point connectedAt_{};
  SteadyClock::time_point lastDisconnect_{};
  bool hasDisconnected_{false};
  bool allStalled_{false};
  uint64_t windowBytes_{0};
  uint64_t connectCount_{0};
  uint64_t connPackets_{0};
}; // class HokuyoSpelRosNode

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HokuyoSpelRosNode>());
  rclcpp::shutdown();
  return 0;
}
