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
#include <std_msgs/msg/u_int8.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nmea_msgs/msg/gpgga.hpp>
#include <nmea_msgs/msg/gprmc.hpp>
#include <nmea_msgs/msg/gpzda.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
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
#include <deque>

#include <hokuyo_spel_master/spnet_utils.hpp>
#include <hokuyo_spel_master/master_params.hpp>
#include <hokuyo_spel_master/payload_converter.hpp>
#include <hokuyo_spel_master/spel_parser.hpp>

// this should be a common file for master, lio, and spel_ros_node
// #include <hokuyo_spel_master/lio_status.hpp>

class HokuyoSpelMasterNode : public rclcpp::Node {
 public:
  HokuyoSpelMasterNode():
    Node("hokuyo_spel_master_node")
  {
    // Publisher
    this->declare_parameter<std::string>("spel_cmd_topic", "/spel_cmd");
    std::string spelCmdTopic;
    this->get_parameter("spel_cmd_topic", spelCmdTopic);
    spelCmdPub_ = this->create_publisher<std_msgs::msg::UInt8>(spelCmdTopic, 100);

    this->declare_parameter<std::string>("spel_ip_address_topic", "/spel_ip_address");
    std::string spelIpAddressTopic;
    this->get_parameter("spel_ip_address_topic", spelIpAddressTopic);
    spelIpAddressPub_ = this->create_publisher<std_msgs::msg::String>(spelIpAddressTopic, 100);

    // Subscriber
    const auto highRateDataQos = rclcpp::SensorDataQoS().keep_last(100);
    const auto highRateStringQos = rclcpp::SystemDefaultsQoS().keep_last(100);

    this->declare_parameter<bool>("stream_nav_sat_fix", true);
    this->get_parameter("stream_nav_sat_fix", streamNavSatFix_);
    if (streamNavSatFix_) {
      this->declare_parameter<std::string>("nav_sat_fix_topic", "/fix");
      std::string navSatFixTopic;
      this->get_parameter("nav_sat_fix_topic", navSatFixTopic);
      navSatFixSub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        navSatFixTopic, rclcpp::SensorDataQoS(),
        std::bind(&HokuyoSpelMasterNode::navSatFixCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Hokuyo SPEL master streams NavSatFix.");
    }

    this->declare_parameter<bool>("stream_gpgga", true);
    this->get_parameter("stream_gpgga", streamGpgga_);
    if (streamGpgga_) {
      this->declare_parameter<std::string>("gpgga_topic", "/gga");
      std::string gpggaTopic;
      this->get_parameter("gpgga_topic", gpggaTopic);
      gpggaSub_ = this->create_subscription<nmea_msgs::msg::Gpgga>(
        gpggaTopic, rclcpp::SensorDataQoS(),
        std::bind(&HokuyoSpelMasterNode::gpggaCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Hokuyo SPEL master streams GPGGA.");
    }

    this->declare_parameter<bool>("stream_gprmc", true);
    this->get_parameter("stream_gprmc", streamGprmc_);
    if (streamGprmc_) {
      this->declare_parameter<std::string>("gprmc_topic", "/rmc");
      std::string gprmcTopic;
      this->get_parameter("gprmc_topic", gprmcTopic);
      gprmcSub_ = this->create_subscription<nmea_msgs::msg::Gprmc>(
        gprmcTopic, rclcpp::SensorDataQoS(),
        std::bind(&HokuyoSpelMasterNode::gprmcCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Hokuyo SPEL master streams GPRMC.");
    }

    this->declare_parameter<bool>("stream_gpzda", true);
    this->get_parameter("stream_gpzda", streamGpzda_);
    if (streamGpzda_) {
      this->declare_parameter<std::string>("gpzda_topic", "/zda");
      std::string gpzdaTopic;
      this->get_parameter("gpzda_topic", gpzdaTopic);
      gpzdaSub_ = this->create_subscription<nmea_msgs::msg::Gpzda>(
        gpzdaTopic, rclcpp::SensorDataQoS(),
        std::bind(&HokuyoSpelMasterNode::gpzdaCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Hokuyo SPEL master streams GPZDA.");
    }

    this->declare_parameter<bool>("stream_hokuyo_cloud2", true);
    this->get_parameter("stream_hokuyo_cloud2", streamHokuyoCloud2_);
    if (streamHokuyoCloud2_) {
      this->declare_parameter<std::string>("hokuyo_cloud2_topic", "/hokuyo3d/hokuyo_cloud2");
      std::string hokuyoCloud2Topic;
      this->get_parameter("hokuyo_cloud2_topic", hokuyoCloud2Topic);
      hokuyoCloud2Sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        hokuyoCloud2Topic, rclcpp::SensorDataQoS(),
        std::bind(&HokuyoSpelMasterNode::hokuyoCloud2Callback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Hokuyo SPEL master streams hokuyo cloud2.");
    }

    this->declare_parameter<bool>("stream_imu", true);
    this->get_parameter("stream_imu", streamImu_);
    if (streamImu_) {
      this->declare_parameter<std::string>("imu_topic", "/hokuyo3d/imu");
      std::string imuTopic;
      this->get_parameter("imu_topic", imuTopic);
      imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imuTopic, highRateDataQos,
        std::bind(&HokuyoSpelMasterNode::imuCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Hokuyo SPEL master streams IMU.");
    }

    this->declare_parameter<bool>("stream_imu_rate_odom", true);
    this->get_parameter("stream_imu_rate_odom", streamImuRateOdom_);
    if (streamImuRateOdom_) {
      this->declare_parameter<std::string>("imu_rate_odom_topic", "/hokuyo_lio/imu_rate_odom");
      std::string imuRateOdomTopic;
      this->get_parameter("imu_rate_odom_topic", imuRateOdomTopic);
      imuRateOdomSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        imuRateOdomTopic, highRateDataQos,
        std::bind(&HokuyoSpelMasterNode::imuRateOdomCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Hokuyo SPEL master streams IMU rate odometry.");
    }

    this->declare_parameter<bool>("stream_nav_sta_fix_switch", true);
    this->get_parameter("stream_nav_sta_fix_switch", streamNavSatFixSwitch_);
    if (streamNavSatFixSwitch_) {
      this->declare_parameter<std::string>("nav_sat_fix_switch_topic", "/fix/switch");
      std::string navSatFixSwitchTopic;
      this->get_parameter("nav_sat_fix_switch_topic", navSatFixSwitchTopic);
      navSatFixSwitchSub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        navSatFixSwitchTopic, highRateDataQos,
        std::bind(&HokuyoSpelMasterNode::navSatFixSwitchCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Hokuyo SPEL master streams NavSatFix switch.");
    }

    this->declare_parameter<bool>("stream_utm_odom", true);
    this->get_parameter("stream_utm_odom", streamUtmOdom_);
    if (streamUtmOdom_) {
      this->declare_parameter<std::string>("utm_odom_topic", "/odometry/utm");
      std::string utmOdomTopic;
      this->get_parameter("utm_odom_topic", utmOdomTopic);
      utmOdomSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        utmOdomTopic, highRateDataQos,
        std::bind(&HokuyoSpelMasterNode::utmOdomCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Hokuyo SPEL master streams UTM odometry.");
    }

    this->declare_parameter<bool>("stream_switch_odom", true);
    this->get_parameter("stream_switch_odom", streamSwitchOdom_);
    if (streamSwitchOdom_) {
      this->declare_parameter<std::string>("switch_odom_topic", "/odometry/switch");
      std::string switchOdomTopic;
      this->get_parameter("switch_odom_topic", switchOdomTopic);
      switchOdomSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        switchOdomTopic, highRateDataQos,
        std::bind(&HokuyoSpelMasterNode::switchOdomCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Hokuyo SPEL master streams switch odometry.");
    }

    this->declare_parameter<bool>("stream_switch_odom_state", true);
    this->get_parameter("stream_switch_odom_state", streamSwitchOdomState_);
    if (streamSwitchOdomState_) {
      this->declare_parameter<std::string>("switch_odom_state_topic", "/odometry/switch/state");
      std::string switchOdomStateTopic;
      this->get_parameter("switch_odom_state_topic", switchOdomStateTopic);
      switchOdomStateSub_ = this->create_subscription<std_msgs::msg::String>(
        switchOdomStateTopic, highRateStringQos,
        std::bind(&HokuyoSpelMasterNode::switchOdomStateCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Hokuyo SPEL master streams switch odometry state.");
    }

    this->declare_parameter<bool>("stream_switch_odom_type", true);
    this->get_parameter("stream_switch_odom_type", streamSwitchOdomType_);
    if (streamSwitchOdomType_) {
      this->declare_parameter<std::string>("switch_odom_type_topic", "/odometry/switch/type");
      std::string switchOdomTypeTopic;
      this->get_parameter("switch_odom_type_topic", switchOdomTypeTopic);
      switchOdomTypeSub_ = this->create_subscription<std_msgs::msg::String>(
        switchOdomTypeTopic, highRateStringQos,
        std::bind(&HokuyoSpelMasterNode::switchOdomTypeCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Hokuyo SPEL master streams switch odometry type.");
    }

    this->declare_parameter<bool>("stream_switch_fix_state", true);
    this->get_parameter("stream_switch_fix_state", streamSwitchFixState_);
    if (streamSwitchFixState_) {
      this->declare_parameter<std::string>("switch_fix_state_topic", "/fix/switch/state");
      std::string switchFixStateTopic;
      this->get_parameter("switch_fix_state_topic", switchFixStateTopic);
      switchFixStateSub_ = this->create_subscription<std_msgs::msg::String>(
        switchFixStateTopic, highRateStringQos,
        std::bind(&HokuyoSpelMasterNode::switchFixStateCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Hokuyo SPEL master streams switch fix state.");
    }

    this->declare_parameter<bool>("stream_switch_fix_type", true);
    this->get_parameter("stream_switch_fix_type", streamSwitchFixType_);
    if (streamSwitchFixType_) {
      this->declare_parameter<std::string>("switch_fix_type_topic", "/fix/switch/type");
      std::string switchFixTypeTopic;
      this->get_parameter("switch_fix_type_topic", switchFixTypeTopic);
      switchFixTypeSub_ = this->create_subscription<std_msgs::msg::String>(
        switchFixTypeTopic, highRateStringQos,
        std::bind(&HokuyoSpelMasterNode::switchFixTypeCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Hokuyo SPEL master streams switch fix type.");
    }

    this->declare_parameter<bool>("stream_diagnostics", true);
    this->get_parameter("stream_diagnostics", streamDiagnostics_);
    if (streamDiagnostics_) {
      this->declare_parameter<std::string>("diagnostics_topic", "/diagnostics");
      std::string diagnosticsTopic;
      this->get_parameter("diagnostics_topic", diagnosticsTopic);
      diagnosticsSub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        diagnosticsTopic, rclcpp::SystemDefaultsQoS(),
        std::bind(&HokuyoSpelMasterNode::diagnosticsCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Hokuyo SPEL master streams diagnosticsTopic.");
    }

    // Initialization for socket communication
    setupSocket();
    spelServerRunning_.store(true);
    spelServerThread_ = std::thread(&HokuyoSpelMasterNode::spelServerLoop, this);
  }

  ~HokuyoSpelMasterNode() override {
    spelServerRunning_.store(false);
    ::shutdown(listenFd_, SHUT_RDWR);
    ::close(listenFd_);
    if (spelServerThread_.joinable()) {
      spelServerThread_.join();
    }
  }

 private:
  // ******************** Callbacks ********************
  void navSatFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    navSatFix_ = *msg;
    navSatFixStamp_ = rclcpp::Time(msg->header.stamp).nanoseconds();
  }

  void gpggaCallback(const nmea_msgs::msg::Gpgga::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    gpgga_ = *msg;
    gpggaStamp_ = rclcpp::Time(msg->header.stamp).nanoseconds();
  }

  void gprmcCallback(const nmea_msgs::msg::Gprmc::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    gprmc_ = *msg;
    gprmcStamp_ = rclcpp::Time(msg->header.stamp).nanoseconds();
  }

  void gpzdaCallback(const nmea_msgs::msg::Gpzda::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    gpzda_ = *msg;
    gpzdaStamp_ = rclcpp::Time(msg->header.stamp).nanoseconds();
  }

  void hokuyoCloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    hokuyoCloud2_ = *msg;
    hokuyoCloud2Stamp_ = rclcpp::Time(msg->header.stamp).nanoseconds();
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    imuBuf_.push_back(*msg);
    while (imuBuf_.size() > 1000) {
      imuBuf_.pop_front();
    }
    imuStamp_ = rclcpp::Time(msg->header.stamp).nanoseconds();
  }

  void lidarRateOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    lidarRateOdom_ = *msg;
    lidarRateOdomStamp_ = rclcpp::Time(msg->header.stamp).nanoseconds();
  }

  void imuRateOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    imuRateOdomBuf_.push_back(*msg);
    while (imuRateOdomBuf_.size() > 1000) {
      imuRateOdomBuf_.pop_front();
    }
    imuRateOdomStamp_ = rclcpp::Time(msg->header.stamp).nanoseconds();
  }

  void navSatFixSwitchCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    navSatFixSwitchBuf_.push_back(*msg);
    while (navSatFixSwitchBuf_.size() > 1000) {
      navSatFixSwitchBuf_.pop_front();
    }
    navSatFixSwitchStamp_ = rclcpp::Time(msg->header.stamp).nanoseconds();
  }

  void utmOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    utmOdomBuf_.push_back(*msg);
    while (utmOdomBuf_.size() > 1000) {
      utmOdomBuf_.pop_front();
    }
    utmOdomStamp_ = rclcpp::Time(msg->header.stamp).nanoseconds();
  }

  void switchOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    switchOdomBuf_.push_back(*msg);
    while (switchOdomBuf_.size() > 1000) {
      switchOdomBuf_.pop_front();
    }
    switchOdomStamp_ = rclcpp::Time(msg->header.stamp).nanoseconds();
  }

  void switchOdomStateCallback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    switchOdomStateBuf_.push_back(*msg);
    while (switchOdomStateBuf_.size() > 1000) {
      switchOdomStateBuf_.pop_front();
    }
    switchOdomStateStamp_ = this->now().nanoseconds();
  }

  void switchOdomTypeCallback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    switchOdomTypeBuf_.push_back(*msg);
    while (switchOdomTypeBuf_.size() > 1000) {
      switchOdomTypeBuf_.pop_front();
    }
    switchOdomTypeStamp_ = this->now().nanoseconds();
  }

  void switchFixStateCallback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    switchFixStateBuf_.push_back(*msg);
    while (switchFixStateBuf_.size() > 1000) {
      switchFixStateBuf_.pop_front();
    }
    switchFixStateStamp_ = this->now().nanoseconds();
  }

  void switchFixTypeCallback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    switchFixTypeBuf_.push_back(*msg);
    while (switchFixTypeBuf_.size() > 1000) {
      switchFixTypeBuf_.pop_front();
    }
    switchFixTypeStamp_ = this->now().nanoseconds();
  }

  void diagnosticsCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    diagnostics_ = *msg;
    diagnosticsStamp_ = rclcpp::Time(msg->header.stamp).nanoseconds();
  }
  // ******************** Callbacks ********************

  void setupSocket() {
    // Ignore broken pipe error
    signal(SIGPIPE, SIG_IGN);

    listenFd_ = ::socket(AF_INET, SOCK_STREAM, 0);

    const int yes = 1;
    ::setsockopt(listenFd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(hsp::kMasterPort);
    ::bind(listenFd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    ::listen(listenFd_, 1);
    RCLCPP_INFO(this->get_logger(), "Hokuyo SPEL master waiting...");
  }

  void spelServerLoop() {
    while (spelServerRunning_.load() && rclcpp::ok()) {
      sockaddr_in cli{};
      socklen_t cl = sizeof(cli);
      int sock = ::accept(listenFd_, (sockaddr*)&cli, &cl);
      if (sock < 0) {
        if (!spelServerRunning_.load()) {
          break;
        }
        perror("accept");
        continue;
      }

      RCLCPP_INFO(this->get_logger(), "Hokuyo SPEL master connected.");

      std::atomic<bool> run(true);
      std::thread rx(&HokuyoSpelMasterNode::spelServerRxLoop, this, sock, std::ref(run));
      std::thread tx(&HokuyoSpelMasterNode::spelServerTxLoop, this, sock, std::ref(run));

      rx.join();
      run.store(false);
      ::shutdown(sock, SHUT_RDWR);
      tx.join();
      ::close(sock);

      streamData_.store(false);
      RCLCPP_INFO(this->get_logger(),
        "Client disconnected, Hokuyo SPEL master waiting new connection...");
    }
  }

  void spelServerRxLoop(
    int sock,
    std::atomic<bool>& run)
  {
    uint32_t seq = 0;
    std::vector<uint8_t> emptyPayload(0);

    while (run.load() && rclcpp::ok()) {

      spnet::Header h{};
      std::vector<uint8_t> pl;
      if (!spnet::recvFrame(sock, h, pl)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to receive data in rxLoop.");
        break;
      }

      const spnet::MsgType type = spnet::toMsgType(h.type);
      // const uint32_t seq = ntohl(h.seq);

      // Protocol note:
      // - Client sends CMD subtype = START/STOP/RESET/SET_IP_ADDRESS
      // - Master replies ACK on success, WARN with string payload on validation error
      // - SET_IP_ADDRESS payload is ASCII IPv4 string (e.g., "192.168.10.100")
      if (type == spnet::MsgType::CMD) {
        const spnet::CmdType cmdtype = spnet::toCmdType(h.subtype);
        const uint8_t ackcmd = static_cast<uint8_t>(spnet::MsgType::ACK);
        uint64_t stamp = this->now().nanoseconds();
        const uint32_t sec = static_cast<uint32_t>(stamp / 1000000000ULL);
        const uint32_t nsec = static_cast<uint32_t>(stamp % 1000000000ULL);

        if (cmdtype == spnet::CmdType::START_STREAMING) {
          streamData_.store(true);
          spnet::sendFrame(sock, ackcmd, 0, sec, nsec, seq, emptyPayload.data(), 0);
          RCLCPP_INFO(this->get_logger(), "SPEL master starts data streaming.");
        
        } else if (cmdtype == spnet::CmdType::STOP_STREAMING) {
          streamData_.store(false);
          spnet::sendFrame(sock, ackcmd, 0, sec, nsec, seq, emptyPayload.data(), 0);
          RCLCPP_INFO(this->get_logger(), "SPEL master stops data streaming.");
  
        } else if (cmdtype == spnet::CmdType::RESET_SOFTWARE) {
          std_msgs::msg::UInt8 cmd;
          cmd.data = 1;
          spelCmdPub_->publish(cmd);
          spnet::sendFrame(sock, ackcmd, 0, sec, nsec, seq, emptyPayload.data(), 0);
          RCLCPP_INFO(this->get_logger(), "SPEL master publishes reset signal topic.");
          
        } else if (cmdtype == spnet::CmdType::SET_IP_ADDRESS) {
          std::string str;
          hspParser_.parseStringPayload(pl, str);
          uint32_t ip;

          if (!spnet::Ipv4StringToUint32(str, ip)) {
            const std::string msg = "SPEL master receives incorrect IP address: " + str;
            const uint8_t warncmd = static_cast<uint8_t>(spnet::MsgType::WARN);
            pl.resize(msg.size());
            std::memcpy(pl.data(), msg.data(), msg.size());
            spnet::sendFrame(sock, warncmd, 0, sec, nsec, seq, pl.data(), pl.size());
            RCLCPP_WARN(this->get_logger(),
              "SPEL master receives incorrect IP address: %s", str.c_str());

          } else {
            std_msgs::msg::String ipAddress;
            ipAddress.data = str;
            spelIpAddressPub_->publish(ipAddress);
            spnet::sendFrame(sock, ackcmd, 0, sec, nsec, seq, emptyPayload.data(), 0);
            RCLCPP_INFO(this->get_logger(), "SPEL master publishes IP address: %s", str.c_str());

          }

        } else {
          const std::string msg = "SPEL master receives unknown command.";
          const uint8_t warncmd = static_cast<uint8_t>(spnet::MsgType::WARN);
          pl.resize(msg.size());
          std::memcpy(pl.data(), msg.data(), msg.size());
          spnet::sendFrame(sock, warncmd, 0, sec, nsec, seq, pl.data(), pl.size());
          RCLCPP_WARN(this->get_logger(), "SPEL master receives unknown command.");

        }
      }
    }

    run.store(false);
  }

  void spelServerTxLoop(
    int sock,
    std::atomic<bool>& run)
  {
    uint32_t navSatFixSeq = 0;
    uint32_t gpggaSeq = 0;
    uint32_t gprmcSeq = 0;
    uint32_t gpzdaSeq = 0;
    uint32_t hokuyoCloud2Seq = 0;
    uint32_t imuSeq = 0;
    // uint32_t lidarRateOdomSeq = 0;
    uint32_t imuRateOdomSeq = 0;
    uint32_t navSatFixSwitchSeq = 0;
    uint32_t utmOdomSeq = 0;
    uint32_t switchOdomSeq = 0;
    uint32_t switchOdomStateSeq = 0;
    uint32_t switchOdomTypeSeq = 0;
    uint32_t switchFixStateSeq = 0;
    uint32_t switchFixTypeSeq = 0;
    uint32_t diagnosticsSeq = 0;

    uint64_t prevNavSatFixStamp = 0;
    uint64_t prevGpggaStamp = 0;
    uint64_t prevGprmcStamp = 0;
    uint64_t prevGpzdaStamp = 0;
    uint64_t prevHokuyoCloud2Stamp = 0;
    // uint64_t prevImuStamp = 0;
    // uint64_t prevLidarRateOdomStamp = 0;
    // uint64_t prevImuRateOdomStamp = 0;
    // uint64_t prevNavSatFixSwitchStamp = 0;
    // uint64_t prevUtmOdomStamp = 0;
    // uint64_t prevSwitchOdomStamp = 0;
    // uint64_t prevSwitchOdomStateStamp = 0;
    // uint64_t prevSwitchOdomTypeStamp = 0;
    // uint64_t prevSwitchFixStateStamp = 0;
    // uint64_t prevSwitchFixTypeStamp = 0;
    uint64_t prevDiagnosticsStamp = 0;

    while (run.load() && rclcpp::ok()) {
      // Just sleep if streaming is disabled.
      if (!streamData_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }

      // Streaming
      // Concurrency note:
      // - Callbacks only update latest message + stamp under mutex.
      // - txLoop streams only when stamp differs from previous to avoid resending duplicates.
      if (streamNavSatFix_) {
        sensor_msgs::msg::NavSatFix navSatFix;
        uint64_t stamp = 0;
        bool updated = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          stamp = navSatFixStamp_;
          if (stamp != prevNavSatFixStamp) {
            navSatFix = navSatFix_;
            prevNavSatFixStamp = stamp;
            updated = true;
          }
        }        
        if (updated) {
          const uint8_t type = static_cast<uint8_t>(spnet::MsgType::DATA);
          const uint8_t subtype = static_cast<uint8_t>(spnet::DataType::NAV_SAT_FIX);
          const uint32_t sec = navSatFix.header.stamp.sec;
          const uint32_t nsec = navSatFix.header.stamp.nanosec;
          std::vector<uint8_t> payload;
          plConv_.navSatFixToPayload(navSatFix, payload);
          const uint32_t payloadSize = static_cast<uint32_t>(payload.size());
          if (!spnet::sendFrame(sock, type, subtype, sec, nsec,
            navSatFixSeq, payload.data(), payloadSize))
          {
            RCLCPP_ERROR(this->get_logger(), "Failed to send NavSatFix.");
            run.store(false);
            break;
          }
        }
      }

      if (streamGpgga_) {
        nmea_msgs::msg::Gpgga gpgga;
        uint64_t stamp = 0;
        bool updated = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          stamp = gpggaStamp_;
          if (stamp != prevGpggaStamp) {
            gpgga = gpgga_;
            prevGpggaStamp = stamp;
            updated = true;
          }
        }        
        if (updated) {
          const uint8_t type = static_cast<uint8_t>(spnet::MsgType::DATA);
          const uint8_t subtype = static_cast<uint8_t>(spnet::DataType::GPGGA);
          const uint32_t sec = gpgga.header.stamp.sec;
          const uint32_t nsec = gpgga.header.stamp.nanosec;
          std::vector<uint8_t> payload;
          plConv_.gpggaToPayload(gpgga, payload);
          const uint32_t payloadSize = static_cast<uint32_t>(payload.size());
          if (!spnet::sendFrame(sock, type, subtype, sec, nsec,
            gpggaSeq, payload.data(), payloadSize))
          {
            RCLCPP_ERROR(this->get_logger(), "Failed to send GPGGA.");
            run.store(false);
            break;
          }
        }
      }

      if (streamGprmc_) {
        nmea_msgs::msg::Gprmc gprmc;
        uint64_t stamp = 0;
        bool updated = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          stamp = gprmcStamp_;
          if (stamp != prevGprmcStamp) {
            gprmc = gprmc_;
            prevGprmcStamp = stamp;
            updated = true;
          }
        }
        if (updated) {
          const uint8_t type = static_cast<uint8_t>(spnet::MsgType::DATA);
          const uint8_t subtype = static_cast<uint8_t>(spnet::DataType::GPRMC);
          const uint32_t sec = gprmc.header.stamp.sec;
          const uint32_t nsec = gprmc.header.stamp.nanosec;
          std::vector<uint8_t> payload;
          plConv_.gprmcToPayload(gprmc, payload);
          const uint32_t payloadSize = static_cast<uint32_t>(payload.size());
          if (!spnet::sendFrame(sock, type, subtype, sec, nsec,
            gprmcSeq, payload.data(), payloadSize))
          {
            RCLCPP_ERROR(this->get_logger(), "Failed to send GPRMC.");
            run.store(false);
            break;
          }
        }
      }

      if (streamGpzda_) {
        nmea_msgs::msg::Gpzda gpzda;
        uint64_t stamp = 0;
        bool updated = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          stamp = gpzdaStamp_;
          if (stamp != prevGpzdaStamp) {
            gpzda = gpzda_;
            prevGpzdaStamp = stamp;
            updated = true;
          }
        }        
        if (updated) {
          const uint8_t type = static_cast<uint8_t>(spnet::MsgType::DATA);
          const uint8_t subtype = static_cast<uint8_t>(spnet::DataType::GPZDA);
          const uint32_t sec = gpzda.header.stamp.sec;
          const uint32_t nsec = gpzda.header.stamp.nanosec;
          std::vector<uint8_t> payload;
          plConv_.gpzdaToPayload(gpzda, payload);
          const uint32_t payloadSize = static_cast<uint32_t>(payload.size());
          if (!spnet::sendFrame(sock, type, subtype, sec, nsec,
            gpzdaSeq, payload.data(), payloadSize))
          {
            RCLCPP_ERROR(this->get_logger(), "Failed to send GPZDA.");
            run.store(false);
            break;
          }
        }
      }

      if (streamHokuyoCloud2_) {
        sensor_msgs::msg::PointCloud2 hokuyoCloud2;
        uint64_t stamp = 0;
        bool updated = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          stamp = hokuyoCloud2Stamp_;
          if (stamp != prevHokuyoCloud2Stamp) {
            hokuyoCloud2 = hokuyoCloud2_;
            prevHokuyoCloud2Stamp = stamp;
            updated = true;
          }
        }
        if (updated) {
          const uint8_t type = static_cast<uint8_t>(spnet::MsgType::DATA);
          const uint8_t subtype = static_cast<uint8_t>(spnet::DataType::HOKUYO_CLOUD2);
          const uint32_t sec = hokuyoCloud2.header.stamp.sec;
          const uint32_t nsec = hokuyoCloud2.header.stamp.nanosec;
          std::vector<uint8_t> payload;
          plConv_.hokuyoCloud2ToPayload(hokuyoCloud2, payload);
          const uint32_t payloadSize = static_cast<uint32_t>(payload.size());
          if (!spnet::sendFrame(sock, type, subtype, sec, nsec,
             hokuyoCloud2Seq, payload.data(), payloadSize))
          {
            RCLCPP_ERROR(this->get_logger(), "Failed to send HokuyoCloud2.");
            run.store(false);
            break;
          }
        }
      }

      if (streamImu_) {
        std::vector<sensor_msgs::msg::Imu> imus;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          const size_t n = std::min(imuBuf_.size(), static_cast<size_t>(50));
          imus.reserve(n);
          for (size_t i = 0; i < n; ++i) {
            imus.push_back(imuBuf_.front());
            imuBuf_.pop_front();
          }
        }
        for (const auto& imu : imus) {
          const uint8_t type = static_cast<uint8_t>(spnet::MsgType::DATA);
          const uint8_t subtype = static_cast<uint8_t>(spnet::DataType::IMU);
          const uint32_t sec = imu.header.stamp.sec;
          const uint32_t nsec = imu.header.stamp.nanosec;
          std::vector<uint8_t> payload;
          plConv_.imuToPayload(imu, payload);
          const uint32_t payloadSize = static_cast<uint32_t>(payload.size());
          if (!spnet::sendFrame(sock, type, subtype, sec, nsec,
            imuSeq, payload.data(), payloadSize))
          {
            RCLCPP_ERROR(this->get_logger(), "Failed to send IMU.");
            run.store(false);
            break;
          }
        }
      }

      if (streamImuRateOdom_) {
        std::vector<nav_msgs::msg::Odometry> imuRateOdoms;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          const size_t n = std::min(imuRateOdomBuf_.size(), static_cast<size_t>(50));
          imuRateOdoms.reserve(n);
          for (size_t i = 0; i < n; ++i) {
            imuRateOdoms.push_back(imuRateOdomBuf_.front());
            imuRateOdomBuf_.pop_front();
          }
        }
        for (const auto& imuRateOdom : imuRateOdoms) {
          const uint8_t type = static_cast<uint8_t>(spnet::MsgType::DATA);
          const uint8_t subtype = static_cast<uint8_t>(spnet::DataType::IMU_RATE_ODOMETRY);
          const uint32_t sec = imuRateOdom.header.stamp.sec;
          const uint32_t nsec = imuRateOdom.header.stamp.nanosec;
          std::vector<uint8_t> payload;
          plConv_.odomToPayload(imuRateOdom, payload);
          const uint32_t payloadSize = static_cast<uint32_t>(payload.size());
          if (!spnet::sendFrame(sock, type, subtype, sec, nsec,
            imuRateOdomSeq, payload.data(), payloadSize))
          {
            RCLCPP_ERROR(this->get_logger(), "Failed to send IMU rate odometry.");
            run.store(false);
            break;
          }
        }
      }

      if (streamNavSatFixSwitch_) {
        std::vector<sensor_msgs::msg::NavSatFix> navSatFixSwitchs;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          const size_t n = std::min(navSatFixSwitchBuf_.size(), static_cast<size_t>(50));
          navSatFixSwitchs.reserve(n);
          for (size_t i = 0; i < n; ++i) {
            navSatFixSwitchs.push_back(navSatFixSwitchBuf_.front());
            navSatFixSwitchBuf_.pop_front();
          }
        }
        for (const auto& navSatFixSwitch : navSatFixSwitchs) {
          const uint8_t type = static_cast<uint8_t>(spnet::MsgType::DATA);
          const uint8_t subtype = static_cast<uint8_t>(spnet::DataType::NAV_SAT_FIX_SWITCH);
          const uint32_t sec = navSatFixSwitch.header.stamp.sec;
          const uint32_t nsec = navSatFixSwitch.header.stamp.nanosec;
          std::vector<uint8_t> payload;
          plConv_.navSatFixToPayload(navSatFixSwitch, payload);
          const uint32_t payloadSize = static_cast<uint32_t>(payload.size());
          if (!spnet::sendFrame(sock, type, subtype, sec, nsec,
            navSatFixSwitchSeq, payload.data(), payloadSize))
          {
            RCLCPP_ERROR(this->get_logger(), "Failed to send NavSatFix switch.");
            run.store(false);
            break;
          }
        }
      }

      if (streamUtmOdom_) {
        std::vector<nav_msgs::msg::Odometry> utmOdoms;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          const size_t n = std::min(utmOdomBuf_.size(), static_cast<size_t>(50));
          utmOdoms.reserve(n);
          for (size_t i = 0; i < n; ++i) {
            utmOdoms.push_back(utmOdomBuf_.front());
            utmOdomBuf_.pop_front();
          }
        }
        for (const auto& utmOdom : utmOdoms) {
          const uint8_t type = static_cast<uint8_t>(spnet::MsgType::DATA);
          const uint8_t subtype = static_cast<uint8_t>(spnet::DataType::UTM_ODOM);
          const uint32_t sec = utmOdom.header.stamp.sec;
          const uint32_t nsec = utmOdom.header.stamp.nanosec;
          std::vector<uint8_t> payload;
          plConv_.odomToPayload(utmOdom, payload);
          const uint32_t payloadSize = static_cast<uint32_t>(payload.size());
          if (!spnet::sendFrame(sock, type, subtype, sec, nsec,
            utmOdomSeq, payload.data(), payloadSize))
          {
            RCLCPP_ERROR(this->get_logger(), "Failed to send UTM odometry.");
            run.store(false);
            break;
          }
        }
      }

      if (streamSwitchOdom_) {
        std::vector<nav_msgs::msg::Odometry> switchOdoms;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          const size_t n = std::min(switchOdomBuf_.size(), static_cast<size_t>(50));
          switchOdoms.reserve(n);
          for (size_t i = 0; i < n; ++i) {
            switchOdoms.push_back(switchOdomBuf_.front());
            switchOdomBuf_.pop_front();
          }
        }
        for (const auto& switchOdom : switchOdoms) {
          const uint8_t type = static_cast<uint8_t>(spnet::MsgType::DATA);
          const uint8_t subtype = static_cast<uint8_t>(spnet::DataType::SWITCH_ODOM);
          const uint32_t sec = switchOdom.header.stamp.sec;
          const uint32_t nsec = switchOdom.header.stamp.nanosec;
          std::vector<uint8_t> payload;
          plConv_.odomToPayload(switchOdom, payload);
          const uint32_t payloadSize = static_cast<uint32_t>(payload.size());
          if (!spnet::sendFrame(sock, type, subtype, sec, nsec,
            switchOdomSeq, payload.data(), payloadSize))
          {
            RCLCPP_ERROR(this->get_logger(), "Failed to send switch odometry.");
            run.store(false);
            break;
          }
        }
      }

      if (streamSwitchOdomState_) {
        uint64_t stamp = 0;
        std::vector<std_msgs::msg::String> switchOdomStates;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          stamp = switchOdomStateStamp_;
          const size_t n = std::min(switchOdomStateBuf_.size(), static_cast<size_t>(50));
          switchOdomStates.reserve(n);
          for (size_t i = 0; i < n; ++i) {
            switchOdomStates.push_back(switchOdomStateBuf_.front());
            switchOdomStateBuf_.pop_front();
          }
        }
        const uint32_t sec = static_cast<uint32_t>(stamp / 1000000000ULL);
        const uint32_t nsec = static_cast<uint32_t>(stamp % 1000000000ULL);
        for (const auto& switchOdomState : switchOdomStates) {
          const uint8_t type = static_cast<uint8_t>(spnet::MsgType::DATA);
          const uint8_t subtype = static_cast<uint8_t>(spnet::DataType::SWITCH_ODOM_STATE);
          std::vector<uint8_t> payload;
          plConv_.stringToPayload(switchOdomState, payload);
          const uint32_t payloadSize = static_cast<uint32_t>(payload.size());
          if (!spnet::sendFrame(sock, type, subtype, sec, nsec,
            switchOdomStateSeq, payload.data(), payloadSize))
          {
            RCLCPP_ERROR(this->get_logger(), "Failed to send swtich odometry state.");
            run.store(false);
            break;
          }
        }
      }

      if (streamSwitchOdomType_) {
        uint64_t stamp = 0;
        std::vector<std_msgs::msg::String> switchOdomTypes;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          stamp = switchOdomTypeStamp_;
          const size_t n = std::min(switchOdomTypeBuf_.size(), static_cast<size_t>(50));
          switchOdomTypes.reserve(n);
          for (size_t i = 0; i < n; ++i) {
            switchOdomTypes.push_back(switchOdomTypeBuf_.front());
            switchOdomTypeBuf_.pop_front();
          }
        }
        const uint32_t sec = static_cast<uint32_t>(stamp / 1000000000ULL);
        const uint32_t nsec = static_cast<uint32_t>(stamp % 1000000000ULL);
        for (const auto& switchOdomType : switchOdomTypes) {
          const uint8_t type = static_cast<uint8_t>(spnet::MsgType::DATA);
          const uint8_t subtype = static_cast<uint8_t>(spnet::DataType::SWITCH_ODOM_TYPE);
          std::vector<uint8_t> payload;
          plConv_.stringToPayload(switchOdomType, payload);
          const uint32_t payloadSize = static_cast<uint32_t>(payload.size());
          if (!spnet::sendFrame(sock, type, subtype, sec, nsec,
            switchOdomTypeSeq, payload.data(), payloadSize))
          {
            RCLCPP_ERROR(this->get_logger(), "Failed to send swtich odometry type.");
            run.store(false);
            break;
          }
        }
      }

      if (streamSwitchFixState_) {
        uint64_t stamp = 0;
        std::vector<std_msgs::msg::String> switchFixStates;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          stamp = switchFixStateStamp_;
          const size_t n = std::min(switchFixStateBuf_.size(), static_cast<size_t>(50));
          switchFixStates.reserve(n);
          for (size_t i = 0; i < n; ++i) {
            switchFixStates.push_back(switchFixStateBuf_.front());
            switchFixStateBuf_.pop_front();
          }
        }
        const uint32_t sec = static_cast<uint32_t>(stamp / 1000000000ULL);
        const uint32_t nsec = static_cast<uint32_t>(stamp % 1000000000ULL);
        for (const auto& switchFixState : switchFixStates) {
          const uint8_t type = static_cast<uint8_t>(spnet::MsgType::DATA);
          const uint8_t subtype = static_cast<uint8_t>(spnet::DataType::SWITCH_FIX_STATE);
          std::vector<uint8_t> payload;
          plConv_.stringToPayload(switchFixState, payload);
          const uint32_t payloadSize = static_cast<uint32_t>(payload.size());
          if (!spnet::sendFrame(sock, type, subtype, sec, nsec,
            switchFixStateSeq, payload.data(), payloadSize))
          {
            RCLCPP_ERROR(this->get_logger(), "Failed to send swtich fix state.");
            run.store(false);
            break;
          }
        }
      }

      if (streamSwitchFixType_) {
        uint64_t stamp = 0;
        std::vector<std_msgs::msg::String> switchFixTypes;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          stamp = switchFixTypeStamp_;
          const size_t n = std::min(switchFixTypeBuf_.size(), static_cast<size_t>(50));
          switchFixTypes.reserve(n);
          for (size_t i = 0; i < n; ++i) {
            switchFixTypes.push_back(switchFixTypeBuf_.front());
            switchFixTypeBuf_.pop_front();
          }
        }
        const uint32_t sec = static_cast<uint32_t>(stamp / 1000000000ULL);
        const uint32_t nsec = static_cast<uint32_t>(stamp % 1000000000ULL);
        for (const auto& switchFixType : switchFixTypes) {
          const uint8_t type = static_cast<uint8_t>(spnet::MsgType::DATA);
          const uint8_t subtype = static_cast<uint8_t>(spnet::DataType::SWITCH_FIX_TYPE);
          std::vector<uint8_t> payload;
          plConv_.stringToPayload(switchFixType, payload);
          const uint32_t payloadSize = static_cast<uint32_t>(payload.size());
          if (!spnet::sendFrame(sock, type, subtype, sec, nsec,
            switchFixTypeSeq, payload.data(), payloadSize))
          {
            RCLCPP_ERROR(this->get_logger(), "Failed to send swtich fix type.");
            run.store(false);
            break;
          }
        }
      }

      if (streamDiagnostics_) {
        diagnostic_msgs::msg::DiagnosticArray diagnostics;
        uint64_t stamp = 0;
        bool updated = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          stamp = diagnosticsStamp_;
          if (stamp != prevDiagnosticsStamp) {
            diagnostics = diagnostics_;
            prevDiagnosticsStamp = stamp;
            updated = true;
          }
        }
        if (updated) {
          const uint8_t type = static_cast<uint8_t>(spnet::MsgType::DATA);
          const uint8_t subtype = static_cast<uint8_t>(spnet::DataType::DIAGNOSTIC_ARRAY);
          const uint32_t sec = diagnostics.header.stamp.sec;
          const uint32_t nsec = diagnostics.header.stamp.nanosec;
          std::vector<uint8_t> payload;
          plConv_.diagnosticsToPayload(diagnostics, payload, "spel_device");
          const uint32_t payloadSize = static_cast<uint32_t>(payload.size());
          if (!spnet::sendFrame(sock, type, subtype, sec, nsec,
            diagnosticsSeq, payload.data(), payloadSize))
          {
            RCLCPP_ERROR(this->get_logger(), "Failed to send diagnostics.");
            run.store(false);
            break;
          }
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      // std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
  }

  int listenFd_;
  std::thread spelServerThread_;
  std::atomic<bool> spelServerRunning_{false};

  hsp::PayloadConverter plConv_;
  hsp::HokuyoSpelParser hspParser_;

  mutable std::mutex mutex_;

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr spelCmdPub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr spelIpAddressPub_;

  std::atomic<bool> streamData_{false};

  bool streamNavSatFix_{false};
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navSatFixSub_;
  sensor_msgs::msg::NavSatFix navSatFix_{};
  uint64_t navSatFixStamp_{0};

  bool streamGpgga_{false};
  rclcpp::Subscription<nmea_msgs::msg::Gpgga>::SharedPtr gpggaSub_;
  nmea_msgs::msg::Gpgga gpgga_{};
  uint64_t gpggaStamp_{0};

  bool streamGprmc_{false};
  rclcpp::Subscription<nmea_msgs::msg::Gprmc>::SharedPtr gprmcSub_;
  nmea_msgs::msg::Gprmc gprmc_{};
  uint64_t gprmcStamp_{0};

  bool streamGpzda_{false};
  rclcpp::Subscription<nmea_msgs::msg::Gpzda>::SharedPtr gpzdaSub_;
  nmea_msgs::msg::Gpzda gpzda_{};
  uint64_t gpzdaStamp_{0};

  bool streamHokuyoCloud2_{false};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr hokuyoCloud2Sub_;
  sensor_msgs::msg::PointCloud2 hokuyoCloud2_{};
  uint64_t hokuyoCloud2Stamp_{0};

  bool streamImu_{false};
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
  std::deque<sensor_msgs::msg::Imu> imuBuf_;
  uint64_t imuStamp_{0};

  bool streamLidarRateOdom_{false};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidarRateOdomSub_;
  nav_msgs::msg::Odometry lidarRateOdom_{};
  uint64_t lidarRateOdomStamp_{0};

  bool streamImuRateOdom_{false};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr imuRateOdomSub_;
  std::deque<nav_msgs::msg::Odometry> imuRateOdomBuf_;
  uint64_t imuRateOdomStamp_{0};

  bool streamNavSatFixSwitch_{false};
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navSatFixSwitchSub_;
  std::deque<sensor_msgs::msg::NavSatFix> navSatFixSwitchBuf_;
  uint64_t navSatFixSwitchStamp_{0};

  bool streamUtmOdom_{false};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr utmOdomSub_;
  std::deque<nav_msgs::msg::Odometry> utmOdomBuf_;
  uint64_t utmOdomStamp_{0};

  bool streamSwitchOdom_{false};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr switchOdomSub_;
  std::deque<nav_msgs::msg::Odometry> switchOdomBuf_;
  uint64_t switchOdomStamp_{0};

  bool streamSwitchOdomState_{false};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr switchOdomStateSub_;
  std::deque<std_msgs::msg::String> switchOdomStateBuf_;
  uint64_t switchOdomStateStamp_{0};

  bool streamSwitchOdomType_{false};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr switchOdomTypeSub_;
  std::deque<std_msgs::msg::String> switchOdomTypeBuf_;
  uint64_t switchOdomTypeStamp_{0};

  bool streamSwitchFixState_{false};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr switchFixStateSub_;
  std::deque<std_msgs::msg::String> switchFixStateBuf_;
  uint64_t switchFixStateStamp_{0};

  bool streamSwitchFixType_{false};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr switchFixTypeSub_;
  std::deque<std_msgs::msg::String> switchFixTypeBuf_;
  uint64_t switchFixTypeStamp_{0};

  bool streamDiagnostics_{false};
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnosticsSub_;
  diagnostic_msgs::msg::DiagnosticArray diagnostics_{};
  uint64_t diagnosticsStamp_{0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HokuyoSpelMasterNode>());
  rclcpp::shutdown();
  return 0;
}
