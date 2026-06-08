#include <rclcpp/rclcpp.hpp>
#include <jsk_rviz_plugin_msgs/msg/overlay_text.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp> 
#include <cmath>
#include <string>
#include <sstream>
#include <iomanip>
#include <vector> // std::vector のために追加
#include <numeric> // std::accumulate のために追加

using std::placeholders::_1;

// 移動平均を計算する際の周期のサンプル数
// 平均周波数を計算するために、直近の周期を保持するウィンドウサイズを1000に設定
constexpr size_t WINDOW_SIZE = 1000; 

class OverlayTextNode : public rclcpp::Node
{
public:
  OverlayTextNode()
  : Node("sensor_status_display_node"), last_lidar_odom_time_(this->now()) 
  {
    this->declare_parameter("sub_gnss_topic", "/spel/nav_sat_fix");
    this->declare_parameter("sub_gnss_state_topic", "/spel/switch_fix_state");
    this->declare_parameter("sub_odometry_type_topic", "/spel/switch_odom_type");
    this->declare_parameter("sub_odometry_state_topic", "/spel/switch_odom_state");
    this->declare_parameter("sub_lidar_odom_topic", "/spel/lidar_rate_odom");
    this->declare_parameter("pub_status_summary_topic", "/spel/status_summary");
    this->declare_parameter("sub_diagnostics_topic", "/spel/diagnostics");
    this->declare_parameter("pub_audio_warning_topic", "/spel/audio_warning");
    this->declare_parameter("cpu_limit", 90);
    this->declare_parameter("temp_limit", 75.0);

    std::string sub_gnss_topic, sub_gnss_state_topic, pub_audio_warning_topic;
    std::string sub_odom_type_topic, sub_odom_state_topic, sub_lidar_odom_topic, pub_status_summary_topic, sub_diagnostics_topic;
    this->get_parameter("sub_gnss_topic", sub_gnss_topic);
    this->get_parameter("sub_gnss_state_topic", sub_gnss_state_topic);
    this->get_parameter("sub_odometry_type_topic", sub_odom_type_topic);
    this->get_parameter("sub_odometry_state_topic", sub_odom_state_topic);
    this->get_parameter("sub_lidar_odom_topic", sub_lidar_odom_topic);
    this->get_parameter("pub_status_summary_topic", pub_status_summary_topic);
    this->get_parameter("sub_diagnostics_topic", sub_diagnostics_topic);
    this->get_parameter("pub_audio_warning_topic", pub_audio_warning_topic);
    this->get_parameter("cpu_limit", cpu_limit_);
    this->get_parameter("temp_limit", temp_limit_);

    gnss_text_publisher_ = this->create_publisher<jsk_rviz_plugin_msgs::msg::OverlayText>("gnss_precision_text", 10);
    gnss_state_text_pub_ = this->create_publisher<jsk_rviz_plugin_msgs::msg::OverlayText>("gnss_state_text", 10);
    odometry_text_publisher_ = this->create_publisher<jsk_rviz_plugin_msgs::msg::OverlayText>("odometry_type_text", 10);
    odom_state_text_pub_ = this->create_publisher<jsk_rviz_plugin_msgs::msg::OverlayText>("odometry_state_text", 10);
    lidar_odom_rate_text_publisher_ = this->create_publisher<jsk_rviz_plugin_msgs::msg::OverlayText>("lidar_odom_rate_text", 10);
    cpu_usage_text_pub_ = this->create_publisher<jsk_rviz_plugin_msgs::msg::OverlayText>("cpu_usage_text", 10);
    temperature_text_pub_ = this->create_publisher<jsk_rviz_plugin_msgs::msg::OverlayText>("temperature_text", 10);
    summary_table_pub_ = this->create_publisher<jsk_rviz_plugin_msgs::msg::OverlayText>("summary_table_text", 10);
    alert_text_pub_ = this->create_publisher<jsk_rviz_plugin_msgs::msg::OverlayText>("alert_text", 10);
    audio_warning_pub_ = this->create_publisher<std_msgs::msg::String>(pub_audio_warning_topic, 10);
    status_summary_pub_ = this->create_publisher<std_msgs::msg::String>(pub_status_summary_topic, 10);
    float_publisher_ = this->create_publisher<std_msgs::msg::Float32>("gnss_fix_float", 10);
    
    gnss_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      sub_gnss_topic, 10, std::bind(&OverlayTextNode::navSatStatusCallBack, this, _1));
    gnss_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      sub_gnss_state_topic, 10, std::bind(&OverlayTextNode::gnssStateCallBack, this, _1));
    odometry_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      sub_odom_type_topic, 10, std::bind(&OverlayTextNode::odometrySwitchTypeCallBack, this, _1));
    odom_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      sub_odom_state_topic, 10, std::bind(&OverlayTextNode::odometrySwitchStateCallBack, this, _1));
    lidar_odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        sub_lidar_odom_topic, 10, std::bind(&OverlayTextNode::lidarOdomCallBack, this, _1));
    diagnostics_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      sub_diagnostics_topic, 10, std::bind(&OverlayTextNode::diagnosticsCallBack, this, _1));

    // 0.5秒タイマーの初期化
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), 
      std::bind(&OverlayTextNode::timer_callback, this));
    
    // レイアウト設定: GNSS (緑系背景) と Odom (青系背景) で分離
    std_msgs::msg::ColorRGBA gnss_bg = createColor(0.0, 0.15, 0.0, 0.5);
    std_msgs::msg::ColorRGBA odom_bg = createColor(0.0, 0.0, 0.2, 0.5);
    std_msgs::msg::ColorRGBA diag_bg = createColor(0.1, 0.1, 0.1, 0.5);

    auto initOT = [&](jsk_rviz_plugin_msgs::msg::OverlayText& ot, int top, const std_msgs::msg::ColorRGBA& bg, const std::string& label) {
      ot.action = jsk_rviz_plugin_msgs::msg::OverlayText::ADD;
      ot.font = "Ubuntu"; ot.left = 10; ot.width = 400; ot.height = 30;
      ot.top = top; ot.bg_color = bg; ot.fg_color = odometry_default_color_;
      ot.text = label + ": N/A";
    };

    initOT(gnss_state_text_, 40, gnss_bg, "GNSS Fix State");
    initOT(gnss_text_, 70, gnss_bg, "GNSS Precision");
    initOT(lidar_odom_rate_text_, 110, odom_bg, "Lidar Odom Rate");
    initOT(odometry_text_, 150, odom_bg, "Odometry Type");
    initOT(odom_state_text_, 180, odom_bg, "Odometry State");
    initOT(cpu_usage_text_, 220, diag_bg, "CPU Usage");
    initOT(temperature_text_, 250, diag_bg, "Device Temp");

    // 統合テーブルの設定
    summary_table_text_.action = jsk_rviz_plugin_msgs::msg::OverlayText::ADD;
    summary_table_text_.left = 10; summary_table_text_.top = 300;
    summary_table_text_.width = 400; summary_table_text_.height = 250;
    summary_table_text_.bg_color = createColor(0.0, 0.0, 0.0, 0.7);

    // アラートの設定 (画面中央付近)
    alert_text_.action = jsk_rviz_plugin_msgs::msg::OverlayText::ADD;
    alert_text_.left = 500; alert_text_.top = 300;
    alert_text_.width = 800; alert_text_.height = 200;
    alert_text_.bg_color = createColor(0.0, 0.0, 0.0, 0.0); // 背景透明
    alert_text_.fg_color = createColor(1.0, 0.0, 0.0, 1.0); // 強烈な赤
    alert_text_.font = "Ubuntu Bold";
  }

private:
  jsk_rviz_plugin_msgs::msg::OverlayText gnss_text_;
  jsk_rviz_plugin_msgs::msg::OverlayText gnss_state_text_;
  jsk_rviz_plugin_msgs::msg::OverlayText odometry_text_;
  jsk_rviz_plugin_msgs::msg::OverlayText odom_state_text_;
  jsk_rviz_plugin_msgs::msg::OverlayText lidar_odom_rate_text_;
  jsk_rviz_plugin_msgs::msg::OverlayText cpu_usage_text_;
  jsk_rviz_plugin_msgs::msg::OverlayText temperature_text_;
  jsk_rviz_plugin_msgs::msg::OverlayText summary_table_text_;
  jsk_rviz_plugin_msgs::msg::OverlayText alert_text_;

  std::string gnss_state_val_ = "N/A";
  std::string gnss_acc_val_ = "N/A";
  std::string odom_type_val_ = "N/A";
  std::string odom_state_val_ = "N/A";
  std::string cpu_usage_val_ = "N/A";
  std::string temperature_val_ = "N/A";
  int cpu_limit_;
  double temp_limit_;
  bool prev_gnss_fix_ = false;
  bool prev_cpu_alert_ = false;
  bool prev_temp_alert_ = false;

  std_msgs::msg::ColorRGBA odometry_lio_switch_color_ = createColor(0.0, 1.0, 1.0, 0.8);
  std_msgs::msg::ColorRGBA odometry_gnss_switch_color_ = createColor(0.0, 1.0, 0.0, 0.8);
  std_msgs::msg::ColorRGBA odometry_lio_raw_color_ = createColor(1.0, 1.0, 0.0, 0.8);
  std_msgs::msg::ColorRGBA odometry_default_color_ = createColor(1.0, 1.0, 1.0, 0.8);

  rclcpp::Publisher<jsk_rviz_plugin_msgs::msg::OverlayText>::SharedPtr gnss_text_publisher_;
  rclcpp::Publisher<jsk_rviz_plugin_msgs::msg::OverlayText>::SharedPtr gnss_state_text_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr float_publisher_;
  rclcpp::Publisher<jsk_rviz_plugin_msgs::msg::OverlayText>::SharedPtr odometry_text_publisher_;
  rclcpp::Publisher<jsk_rviz_plugin_msgs::msg::OverlayText>::SharedPtr odom_state_text_pub_;
  rclcpp::Publisher<jsk_rviz_plugin_msgs::msg::OverlayText>::SharedPtr lidar_odom_rate_text_publisher_;
  rclcpp::Publisher<jsk_rviz_plugin_msgs::msg::OverlayText>::SharedPtr cpu_usage_text_pub_;
  rclcpp::Publisher<jsk_rviz_plugin_msgs::msg::OverlayText>::SharedPtr temperature_text_pub_;
  rclcpp::Publisher<jsk_rviz_plugin_msgs::msg::OverlayText>::SharedPtr summary_table_pub_;
  rclcpp::Publisher<jsk_rviz_plugin_msgs::msg::OverlayText>::SharedPtr alert_text_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr audio_warning_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_summary_pub_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gnss_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr odom_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_subscriber_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  
  rclcpp::Time last_lidar_odom_time_;
  std::vector<double> lidar_odom_periods_; 
  rclcpp::TimerBase::SharedPtr timer_;

  // 色を簡単に作成するためのヘルパー関数 (変更なし)
  std_msgs::msg::ColorRGBA createColor(double r, double g, double b, double a) {
      std_msgs::msg::ColorRGBA color;
      color.r = r;
      color.g = g;
      color.b = b;
      color.a = a;
      return color;
  }

  // Odometry 用のコールバック関数 (周期計測と移動平均へのデータ追加)
  void lidarOdomCallBack(const nav_msgs::msg::Odometry::SharedPtr /* msg */)
  {
    rclcpp::Time current_time = this->now();
    rclcpp::Duration period_duration = current_time - last_lidar_odom_time_;
    last_lidar_odom_time_ = current_time;
    
    double current_period = period_duration.seconds();

    // 1. 新しい周期をリストに追加
    if (lidar_odom_periods_.empty() && current_period < 0.001) {
        // 初回起動時の異常な短い周期は無視
    } else {
        lidar_odom_periods_.push_back(current_period);
    }

    // 2. リストのサイズを制限 (移動窓の維持)
    if (lidar_odom_periods_.size() > WINDOW_SIZE) {
        lidar_odom_periods_.erase(lidar_odom_periods_.begin()); // 最も古い要素を削除
    }
  }

  // 0.5秒ごとに呼び出されるタイマーコールバック関数 (平均周波数計算とパブリッシュを実行)
  void timer_callback()
  {
    double avg_period = 0.0;
    double frequency_hz = 0.0;
    
    // 1. 移動平均を計算
    if (lidar_odom_periods_.size() > 0) {
        // リスト内の全ての周期を合計
        double sum_of_periods = std::accumulate(lidar_odom_periods_.begin(), lidar_odom_periods_.end(), 0.0);
        
        // 平均周期を計算
        avg_period = sum_of_periods / lidar_odom_periods_.size();

        // 平均周波数 (Hz) を計算 (周波数を表示したいので、周期の逆数をとる)
        if (avg_period > 0.0) {
            frequency_hz = 1.0 / avg_period;
        }
    }

    // 1. 統合ステータスサマリーのパブリッシュ
    std_msgs::msg::String summary_msg;
    std::stringstream ss_sum;
    ss_sum << "GNSS: " << gnss_state_val_ << " (Prec: " << gnss_acc_val_ << ") | "
           << "Odom: " << odom_type_val_ << " [" << odom_state_val_ << "] | "
           << "Lidar Rate: " << std::fixed << std::setprecision(1) << frequency_hz << " Hz"
           << " | CPU: " << cpu_usage_val_ << "% | Temp: " << temperature_val_;
    summary_msg.data = ss_sum.str();
    status_summary_pub_->publish(summary_msg);

    // 2. 統合テーブルの作成 (HTMLライク)
    std::stringstream ss_table;
    ss_table << "<span style='font-size: 14pt; color: white;'>RSF System Status</span><br>"
             << "<hr>"
             << "<table>"
             << "<tr><td>GNSS State:</td><td>" << gnss_state_val_ << "</td></tr>"
             << "<tr><td>Accuracy:</td><td>" << gnss_acc_val_ << "</td></tr>"
             << "<tr><td>Odom Type:</td><td>" << odom_type_val_ << "</td></tr>"
             << "<tr><td>Odom State:</td><td>" << odom_state_val_ << "</td></tr>"
             << "<tr><td>Lidar Rate:</td><td>" << std::fixed << std::setprecision(1) << frequency_hz << " Hz</td></tr>"
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
          std_msgs::msg::String msg;
          msg.data = "cpu_overload";
          audio_warning_pub_->publish(msg);
        }
        else if (!current_cpu_alert && prev_cpu_alert_) {
          std_msgs::msg::String msg;
          msg.data = "cpu_normal";
          audio_warning_pub_->publish(msg);
        }
        prev_cpu_alert_ = current_cpu_alert;
        if (current_cpu_alert) alerts += "!!! HIGH CPU LOAD !!!<br>";
      }
      if (!temperature_val_.empty() && temperature_val_ != "N/A") {
        // "35.0 degC" から数値抽出
        double t = std::stod(temperature_val_.substr(0, temperature_val_.find(" ")));
        bool current_temp_alert = (t > temp_limit_);
        if (current_temp_alert && !prev_temp_alert_) {
          std_msgs::msg::String msg;
          msg.data = "temperature_error";
          audio_warning_pub_->publish(msg);
        }
        else if (!current_temp_alert && prev_temp_alert_) {
          std_msgs::msg::String msg;
          msg.data = "temperature_normal";
          audio_warning_pub_->publish(msg);
        }
        prev_temp_alert_ = current_temp_alert;
        if (current_temp_alert) alerts += "!!! DEVICE OVERHEAT !!!<br>";
      }
    } catch (...) {}

    // GNSS ロスト判定 (Fixから非Fixへの遷移)
    bool current_gnss_fix = (gnss_state_val_.find("FIX") != std::string::npos);
    if (prev_gnss_fix_ && !current_gnss_fix && gnss_state_val_ != "N/A") {
        std_msgs::msg::String msg;
        msg.data = "gnss_lost";
        audio_warning_pub_->publish(msg);
    }
    else if (!prev_gnss_fix_ && current_gnss_fix && gnss_state_val_ != "N/A") {
        std_msgs::msg::String msg;
        msg.data = "gnss_recovered";
        audio_warning_pub_->publish(msg);
    }
    prev_gnss_fix_ = current_gnss_fix;

    if (!alerts.empty()) {
      alert_text_.text = "<span style='font-size: 30pt;'>" + alerts + "</span>";
    } else {
      alert_text_.text = ""; // 正常時は非表示
    }
    alert_text_pub_->publish(alert_text_);

    // 4. 個別テキストの整形とパブリッシュ (後方互換用)
    std::stringstream ss;
    if (frequency_hz > 0.0) {
        // 周波数 (Hz) を表示
        ss << "Lidar Odom Rate: " << std::fixed << std::setprecision(2) << frequency_hz << " Hz";

        // 周波数に応じて文字色を変更 (10Hzを基準)
        std_msgs::msg::ColorRGBA rate_color;
        if (frequency_hz >= 9.5) { 
            rate_color = createColor(0.0, 1.0, 0.0, 0.8); // 緑 (良好)
        } else if (frequency_hz >= 5.0) { 
            rate_color = createColor(1.0, 1.0, 0.0, 0.8); // 黄 (許容範囲)
        } else { 
            rate_color = createColor(1.0, 0.0, 0.0, 0.8); // 赤 (要確認)
        }
        lidar_odom_rate_text_.fg_color = rate_color;
    } else {
        // データがまだ十分でない、またはトピックが流れていない場合
        ss << "Lidar Odom Rate: N/A";
        lidar_odom_rate_text_.fg_color = odometry_default_color_;
    }
    
    lidar_odom_rate_text_.text = ss.str();
    lidar_odom_rate_text_publisher_->publish(lidar_odom_rate_text_);
  }

  void navSatStatusCallBack(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    std_msgs::msg::ColorRGBA color;
    double gnss_status = std::sqrt(msg->position_covariance[0]);
    gnss_acc_val_ = std::isnan(gnss_status) ? "N/A" : std::to_string(gnss_status).substr(0,4) + "m";

    std_msgs::msg::Float32 float_data;
    float_data.data = static_cast<float>(gnss_status);

    if (gnss_status <= 0.1) {
      color = createColor(0.0, 1.0, 0.0, 0.8);
      gnss_text_.text = "GNSSの精度は良好です。";
    } else if (gnss_status > 0.1 && gnss_status <= 4) {
      color = createColor(237.0 / 255.0, 212.0 / 255.0, 0.0, 0.8);
      gnss_text_.text = "GNSSの精度は中程度です。";
    } else if (gnss_status > 4) {
      color = createColor(1.0, 0.0, 0.0, 0.8);
      gnss_text_.text = "GNSSの精度が低い状態です。";
    } else {
        color = createColor(0.1, 1.0, 0.9, 0.8);
        gnss_text_.text = "GNSS Precision: N/A";
    }
    
    gnss_text_.fg_color = color;
    if (gnss_text_.text.find("Precision") == std::string::npos) gnss_text_.text = "GNSS Accuracy: " + gnss_acc_val_;
    gnss_text_publisher_->publish(gnss_text_);
    float_publisher_->publish(float_data);
  }

  void gnssStateCallBack(const std_msgs::msg::String::SharedPtr msg)
  {
    gnss_state_val_ = msg->data;
    gnss_state_text_.text = "GNSS Fix State: " + gnss_state_val_;
    if (gnss_state_val_.find("FIX") != std::string::npos) gnss_state_text_.fg_color = createColor(0, 1, 0, 0.8);
    else if (gnss_state_val_.find("FLOAT") != std::string::npos) gnss_state_text_.fg_color = createColor(1, 1, 0, 0.8);
    else gnss_state_text_.fg_color = createColor(1, 0, 0, 0.8);
    gnss_state_text_pub_->publish(gnss_state_text_);
  }

  void odometrySwitchTypeCallBack(const std_msgs::msg::String::SharedPtr msg)
  {
    odom_type_val_ = msg->data;
    odometry_text_.text = "Odometry Type: " + msg->data;

    if (msg->data == "LIO (switch)") {
      odometry_text_.fg_color = odometry_lio_switch_color_;
    } else if (msg->data == "GNSS (switch)") {
      odometry_text_.fg_color = odometry_gnss_switch_color_;
    } else if (msg->data == "LIO (raw)") {
      odometry_text_.fg_color = odometry_lio_raw_color_;
    } else {
      odometry_text_.fg_color = odometry_default_color_;
    }

    odometry_text_publisher_->publish(odometry_text_);
  }

  void odometrySwitchStateCallBack(const std_msgs::msg::String::SharedPtr msg)
  {
    odom_state_val_ = msg->data;
    odom_state_text_.text = "Odometry State: " + odom_state_val_;
    if (odom_state_val_ == "steady") odom_state_text_.fg_color = createColor(0, 1, 0, 0.8);
    else odom_state_text_.fg_color = createColor(1, 1, 0, 0.8);
    odom_state_text_pub_->publish(odom_state_text_);
  }

  void diagnosticsCallBack(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
  {
    for (const auto& status : msg->status) {
      // spel_device のステータスを探す
      if (status.name == "spel_device") {
        for (const auto& kv : status.values) {
          if (kv.key == "cpu_usage") {
            cpu_usage_val_ = kv.value;
            cpu_usage_text_.text = "CPU Usage: " + cpu_usage_val_ + "%";
            try {
              int cpu = std::stoi(cpu_usage_val_);
              if (cpu < cpu_limit_ * 0.77) cpu_usage_text_.fg_color = createColor(0, 1, 0, 0.8);
              else if (cpu < cpu_limit_) cpu_usage_text_.fg_color = createColor(1, 1, 0, 0.8);
              else cpu_usage_text_.fg_color = createColor(1, 0, 0, 0.8);
            } catch (...) {}
            cpu_usage_text_pub_->publish(cpu_usage_text_);
          } else if (kv.key == "device_temperature") {
            try {
              // degC として表示
              double temp_degc = std::stod(kv.value);
              std::stringstream ss_temp;
              ss_temp << std::fixed << std::setprecision(1) << temp_degc << " degC";
              temperature_val_ = ss_temp.str();
              temperature_text_.text = "Device Temp: " + temperature_val_;
              
              if (temp_degc < temp_limit_ * 0.73) temperature_text_.fg_color = createColor(0, 1, 0, 0.8);
              else if (temp_degc < temp_limit_) temperature_text_.fg_color = createColor(1, 1, 0, 0.8);
              else temperature_text_.fg_color = createColor(1, 0, 0, 0.8);
            } catch (...) {
              temperature_val_ = "N/A";
              temperature_text_.text = "Device Temp: N/A";
            }
            temperature_text_pub_->publish(temperature_text_);
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