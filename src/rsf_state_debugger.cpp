#include <rclcpp/rclcpp.hpp>
#include <jsk_rviz_plugin_msgs/msg/overlay_text.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nmea_msgs/msg/gpgga.hpp>
#include <nav_msgs/msg/odometry.hpp> 
#include <cmath>
#include <string>
#include <sstream>
#include <iomanip>
#include <vector> 
#include <numeric> 

using std::placeholders::_1;

constexpr size_t WINDOW_SIZE = 1000; 

class OverlayTextNode : public rclcpp::Node
{
public:
  OverlayTextNode()
  : Node("sensor_status_display_node"), last_lidar_odom_time_(this->now()) 
  {
    this->declare_parameter("sub_gnss_topic", "/spel/nav_sat_fix");
    this->declare_parameter("sub_gnss_state_topic", "/spel/switch_fix_state");
    this->declare_parameter("sub_gnss_type_topic", "/spel/switch_fix_type");
    this->declare_parameter("sub_gpgga_topic", "/spel/gpgga");
    this->declare_parameter("sub_odometry_type_topic", "/spel/switch_odom_type");
    this->declare_parameter("sub_odometry_state_topic", "/spel/switch_odom_state");
    this->declare_parameter("sub_lidar_odom_topic", "/spel/lidar_rate_odom");
    this->declare_parameter("pub_status_summary_topic", "/spel/status_summary");
    this->declare_parameter("sub_diagnostics_topic", "/spel/diagnostics");
    this->declare_parameter("pub_audio_warning_topic", "/spel/audio_warning");
    this->declare_parameter("cpu_limit", 90);
    this->declare_parameter("temp_limit", 75.0);

    std::string sub_gnss_topic, sub_gnss_state_topic, sub_gnss_type_topic, sub_gpgga_topic, pub_audio_warning_topic;
    std::string sub_odom_type_topic, sub_odom_state_topic, sub_lidar_odom_topic, pub_status_summary_topic, sub_diagnostics_topic;
    this->get_parameter("sub_gnss_topic", sub_gnss_topic);
    this->get_parameter("sub_gnss_state_topic", sub_gnss_state_topic);
    this->get_parameter("sub_gnss_type_topic", sub_gnss_type_topic);
    this->get_parameter("sub_gpgga_topic", sub_gpgga_topic);
    this->get_parameter("sub_odometry_type_topic", sub_odom_type_topic);
    this->get_parameter("sub_odometry_state_topic", sub_odom_state_topic);
    this->get_parameter("sub_lidar_odom_topic", sub_lidar_odom_topic);
    this->get_parameter("pub_status_summary_topic", pub_status_summary_topic);
    this->get_parameter("sub_diagnostics_topic", sub_diagnostics_topic);
    this->get_parameter("pub_audio_warning_topic", pub_audio_warning_topic);
    this->get_parameter("cpu_limit", cpu_limit_);
    this->get_parameter("temp_limit", temp_limit_);

    // 負荷軽減のため、統合表示（summary_table）とアラート表示、システム間通信用トピックに絞る
    summary_table_pub_ = this->create_publisher<jsk_rviz_plugin_msgs::msg::OverlayText>("summary_table_text", 10);
    alert_text_pub_ = this->create_publisher<jsk_rviz_plugin_msgs::msg::OverlayText>("alert_text", 10);
    audio_warning_pub_ = this->create_publisher<std_msgs::msg::String>(pub_audio_warning_topic, 10);
    status_summary_pub_ = this->create_publisher<std_msgs::msg::String>(pub_status_summary_topic, 10);
    float_publisher_ = this->create_publisher<std_msgs::msg::Float32>("gnss_fix_float", 10);
    
    gnss_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      sub_gnss_topic, 10, std::bind(&OverlayTextNode::navSatStatusCallBack, this, _1));
    gnss_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      sub_gnss_state_topic, 10, std::bind(&OverlayTextNode::gnssStateCallBack, this, _1));
    gnss_type_sub_ = this->create_subscription<std_msgs::msg::String>(
      sub_gnss_type_topic, 10, std::bind(&OverlayTextNode::gnssTypeCallBack, this, _1));
    gpgga_sub_ = this->create_subscription<nmea_msgs::msg::Gpgga>(
      sub_gpgga_topic, 10, std::bind(&OverlayTextNode::gpggaCallBack, this, _1));
    odometry_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      sub_odom_type_topic, 10, std::bind(&OverlayTextNode::odometrySwitchTypeCallBack, this, _1));
    odom_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      sub_odom_state_topic, 10, std::bind(&OverlayTextNode::odometrySwitchStateCallBack, this, _1));
    lidar_odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        sub_lidar_odom_topic, 10, std::bind(&OverlayTextNode::lidarOdomCallBack, this, _1));
    diagnostics_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      sub_diagnostics_topic, 10, std::bind(&OverlayTextNode::diagnosticsCallBack, this, _1));

    // 0.5秒タイマー
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), 
      std::bind(&OverlayTextNode::timer_callback, this));

    // 統合テーブルの設定
    summary_table_text_.action = jsk_rviz_plugin_msgs::msg::OverlayText::ADD;
    summary_table_text_.left = 10; summary_table_text_.top = 300;
    summary_table_text_.width = 400; summary_table_text_.height = 250;
    summary_table_text_.bg_color = createColor(0.0, 0.0, 0.0, 0.7);

    // アラートの設定
    alert_text_.action = jsk_rviz_plugin_msgs::msg::OverlayText::ADD;
    alert_text_.left = 500; alert_text_.top = 300;
    alert_text_.width = 800; alert_text_.height = 200;
    alert_text_.bg_color = createColor(0.0, 0.0, 0.0, 0.0); 
    alert_text_.fg_color = createColor(1.0, 0.0, 0.0, 1.0); 
    alert_text_.font = "Ubuntu Bold";
  }

private:
  jsk_rviz_plugin_msgs::msg::OverlayText summary_table_text_;
  jsk_rviz_plugin_msgs::msg::OverlayText alert_text_;

  std::string gnss_state_val_ = "N/A";
  std::string gnss_type_val_ = "N/A";
  std::string gnss_acc_val_ = "N/A";
  int gnss_qual_int_ = -1;
  std::string gnss_qual_val_ = "N/A";
  std::string odom_type_val_ = "N/A";
  std::string odom_state_val_ = "N/A";
  std::string cpu_usage_val_ = "N/A";
  std::string temperature_val_ = "N/A";
  int cpu_limit_;
  double temp_limit_;
  bool prev_gnss_fix_ = false;
  bool prev_cpu_alert_ = false;
  bool prev_temp_alert_ = false;

  rclcpp::Publisher<jsk_rviz_plugin_msgs::msg::OverlayText>::SharedPtr summary_table_pub_;
  rclcpp::Publisher<jsk_rviz_plugin_msgs::msg::OverlayText>::SharedPtr alert_text_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr audio_warning_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_summary_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr float_publisher_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gnss_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gnss_type_sub_;
  rclcpp::Subscription<nmea_msgs::msg::Gpgga>::SharedPtr gpgga_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr odom_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_subscriber_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  
  rclcpp::Time last_lidar_odom_time_;
  std::vector<double> lidar_odom_periods_; 
  rclcpp::TimerBase::SharedPtr timer_;

  std_msgs::msg::ColorRGBA createColor(double r, double g, double b, double a) {
      std_msgs::msg::ColorRGBA color;
      color.r = r; color.g = g; color.b = b; color.a = a;
      return color;
  }

  void lidarOdomCallBack(const nav_msgs::msg::Odometry::SharedPtr /* msg */)
  {
    rclcpp::Time current_time = this->now();
    rclcpp::Duration period_duration = current_time - last_lidar_odom_time_;
    last_lidar_odom_time_ = current_time;
    
    double current_period = period_duration.seconds();

    if (lidar_odom_periods_.empty() && current_period < 0.001) {
        // 無視
    } else {
        lidar_odom_periods_.push_back(current_period);
    }

    if (lidar_odom_periods_.size() > WINDOW_SIZE) {
        lidar_odom_periods_.erase(lidar_odom_periods_.begin());
    }
  }

  // 0.5秒ごとに集約されたデータのみをパブリッシュ
  void timer_callback()
  {
    double avg_period = 0.0;
    double frequency_hz = 0.0;
    
    if (!lidar_odom_periods_.empty()) {
        double sum_of_periods = std::accumulate(lidar_odom_periods_.begin(), lidar_odom_periods_.end(), 0.0);
        avg_period = sum_of_periods / lidar_odom_periods_.size();
        if (avg_period > 0.0) {
            frequency_hz = 1.0 / avg_period;
        }
    }

    // 1. 統合ステータスサマリーのパブリッシュ
    std_msgs::msg::String summary_msg;
    std::stringstream ss_sum;
    ss_sum << "GNSS: " << gnss_state_val_ << " (" << gnss_type_val_ << ") [Qual:" << gnss_qual_val_ << "] (Prec: " << gnss_acc_val_ << ") | "
           << "Odom: " << odom_type_val_ << " [" << odom_state_val_ << "] | "
           << "LIO Rate: " << std::fixed << std::setprecision(1) << frequency_hz << " Hz"
           << " | CPU: " << cpu_usage_val_ << "% | Temp: " << temperature_val_;
    summary_msg.data = ss_sum.str();
    status_summary_pub_->publish(summary_msg);

    // 2. 統合テーブルの作成 (HTMLライク)
    std::string qual_color = "white";
    if (gnss_qual_int_ == 4) qual_color = "#00FF00"; // RTK Fix
    else if (gnss_qual_int_ == 5) qual_color = "yellow";
    else if (gnss_qual_int_ == 1 || gnss_qual_int_ == 2) qual_color = "cyan";
    else if (gnss_qual_int_ != -1) qual_color = "red";

    // LIOのタイプに応じて色変更（元の個別表示のロジックを統合テーブルに移植）
    std::string odom_type_color = "white";
    if (odom_type_val_ == "LIO (switch)") odom_type_color = "#00FFFF";
    else if (odom_type_val_ == "GNSS (switch)") odom_type_color = "#00FF00";
    else if (odom_type_val_ == "LIO (raw)") odom_type_color = "#FFFF00";

    // LIOレートに応じた色変更
    std::string rate_color = "white";
    if (frequency_hz >= 9.5) rate_color = "#00FF00";
    else if (frequency_hz >= 5.0) rate_color = "#FFFF00";
    else if (frequency_hz > 0.0) rate_color = "#FF0000";

    std::stringstream ss_table;
    ss_table << "<span style='font-size: 14pt; color: white;'>RSF System Status</span><br>"
             << "<hr>"
             << "<table>"
             << "<tr><td>Accuracy:</td><td>" << gnss_acc_val_ << "</td></tr>"
             << "<tr><td>GPGGA Qual:</td><td><span style='color: " << qual_color << ";'>" << gnss_qual_val_ << "</span></td></tr>"
             << "<tr><td>GNSS Type:</td><td>" << gnss_type_val_ << "</td></tr>"
             << "<tr><td>GNSS State:</td><td>" << gnss_state_val_ << "</td></tr>"
             << "<tr><td>Odom Type:</td><td><span style='color: " << odom_type_color << ";'>" << odom_type_val_ << "</span></td></tr>"
             << "<tr><td>Odom State:</td><td>" << odom_state_val_ << "</td></tr>"
             << "<tr><td>LIO Rate:</td><td><span style='color: " << rate_color << ";'>" << std::fixed << std::setprecision(1) << frequency_hz << " Hz</span></td></tr>"
             << "<tr><td>CPU Usage:</td><td>" << cpu_usage_val_ << " %</td></tr>"
             << "<tr><td>Device Temp:</td><td>" << temperature_val_ << "</td></tr>"
             << "</table>";
    summary_table_text_.text = ss_table.str();
    summary_table_pub_->publish(summary_table_text_);

    // 3. アラート判定
    std::string alerts = "";
    try {
      if (!cpu_usage_val_.empty() && cpu_usage_val_ != "N/A") {
        bool current_cpu_alert = (std::stoi(cpu_usage_val_) > cpu_limit_);
        if (current_cpu_alert && !prev_cpu_alert_) {
          std_msgs::msg::String msg; msg.data = "cpu_overload"; audio_warning_pub_->publish(msg);
        }
        else if (!current_cpu_alert && prev_cpu_alert_) {
          std_msgs::msg::String msg; msg.data = "cpu_normal"; audio_warning_pub_->publish(msg);
        }
        prev_cpu_alert_ = current_cpu_alert;
        if (current_cpu_alert) alerts += "!!! HIGH CPU LOAD !!!<br>";
      }
      if (!temperature_val_.empty() && temperature_val_ != "N/A") {
        double t = std::stod(temperature_val_.substr(0, temperature_val_.find(" ")));
        bool current_temp_alert = (t > temp_limit_);
        if (current_temp_alert && !prev_temp_alert_) {
          std_msgs::msg::String msg; msg.data = "temperature_error"; audio_warning_pub_->publish(msg);
        }
        else if (!current_temp_alert && prev_temp_alert_) {
          std_msgs::msg::String msg; msg.data = "temperature_normal"; audio_warning_pub_->publish(msg);
        }
        prev_temp_alert_ = current_temp_alert;
        if (current_temp_alert) alerts += "!!! DEVICE OVERHEAT !!!<br>";
      }
    } catch (...) {}

    bool current_gnss_fix = (gnss_state_val_.find("FIX") != std::string::npos);
    if (prev_gnss_fix_ && !current_gnss_fix && gnss_state_val_ != "N/A") {
        std_msgs::msg::String msg; msg.data = "gnss_lost"; audio_warning_pub_->publish(msg);
    }
    else if (!prev_gnss_fix_ && current_gnss_fix && gnss_state_val_ != "N/A") {
        std_msgs::msg::String msg; msg.data = "gnss_recovered"; audio_warning_pub_->publish(msg);
    }
    prev_gnss_fix_ = current_gnss_fix;

    if (!alerts.empty()) {
      alert_text_.text = "<span style='font-size: 30pt;'>" + alerts + "</span>";
    } else {
      alert_text_.text = ""; 
    }
    alert_text_pub_->publish(alert_text_);
  }

  // 各コールバック内ではメンバ変数の更新のみに留め、OverlayTextのパブリッシュを削除
  void navSatStatusCallBack(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    double gnss_status = std::sqrt(msg->position_covariance[0]);
    gnss_acc_val_ = std::isnan(gnss_status) ? "N/A" : std::to_string(gnss_status).substr(0,4) + "m";

    std_msgs::msg::Float32 float_data;
    float_data.data = static_cast<float>(gnss_status);
    float_publisher_->publish(float_data); // 数値トピックは軽量なため維持
  }

  void gnssStateCallBack(const std_msgs::msg::String::SharedPtr msg)
  {
    gnss_state_val_ = msg->data;
  }

  void gnssTypeCallBack(const std_msgs::msg::String::SharedPtr msg)
  {
    gnss_type_val_ = msg->data;
  }

  void gpggaCallBack(const nmea_msgs::msg::Gpgga::SharedPtr msg)
  {
    gnss_qual_int_ = msg->gps_qual;
    switch(msg->gps_qual) {
      case 0: gnss_qual_val_ = "Invalid";   break;
      case 1: gnss_qual_val_ = "GPS";       break;
      case 2: gnss_qual_val_ = "DGPS";      break;
      case 4: gnss_qual_val_ = "RTK Fix";   break;
      case 5: gnss_qual_val_ = "RTK Float"; break;
      case 6: gnss_qual_val_ = "Estimated"; break;
      default: gnss_qual_val_ = std::to_string(msg->gps_qual); break;
    }
  }

  void odometrySwitchTypeCallBack(const std_msgs::msg::String::SharedPtr msg)
  {
    odom_type_val_ = msg->data;
  }

  void odometrySwitchStateCallBack(const std_msgs::msg::String::SharedPtr msg)
  {
    odom_state_val_ = msg->data;
  }

  void diagnosticsCallBack(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
  {
    for (const auto& status : msg->status) {
      if (status.name == "spel_device") {
        for (const auto& kv : status.values) {
          if (kv.key == "cpu_usage") {
            cpu_usage_val_ = kv.value;
          } else if (kv.key == "device_temperature") {
            try {
              double temp_degc = std::stod(kv.value);
              std::stringstream ss_temp;
              ss_temp << std::fixed << std::setprecision(1) << temp_degc << " degC";
              temperature_val_ = ss_temp.str();
            } catch (...) {
              temperature_val_ = "N/A";
            }
          }
        }
        break;
      }
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OverlayTextNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}