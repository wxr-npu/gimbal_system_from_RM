#include "ai_msgs/msg/perception_targets.hpp"
#include "rclcpp/rclcpp.hpp"

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace {

// 下位机回传的诊断帧格式定义（固定 46 字节）。
constexpr uint8_t kVisionDiagHead0 = 0xD1;
constexpr uint8_t kVisionDiagHead1 = 0x5B;
constexpr uint8_t kVisionDiagTail0 = 0x6B;
constexpr uint8_t kVisionDiagTail1 = 0x1D;
constexpr size_t kVisionDiagFrameSize = 48;
constexpr uint8_t kVisionDiagFlagVisionEnabled = 0x01;
constexpr uint8_t kVisionDiagFlagTargetValid = 0x02;
constexpr uint8_t kVisionDiagFlagLinkOnline = 0x04;
constexpr uint8_t kVisionDiagFlagRcError = 0x08;
constexpr uint8_t kVisionDiagFlagDbusToe = 0x10;

// 目标候选结构体
struct TargetCandidate {
  std::string type;
  double center_x;
  double center_y;
  double confidence;
  double distance_to_center;
};


// 下位机回传的诊断帧结构体
struct VisionDiagFrame {
  uint8_t flags;
  uint8_t seq;
  uint16_t raw_x;
  uint16_t raw_y;
  int16_t error_x;
  int16_t error_y;
  int16_t yaw_add_mrad;
  int16_t pitch_add_mrad;
  uint16_t parsed_frames;
  uint16_t rx_bytes;
  uint8_t rc_sw0;
  uint8_t rc_sw1;
  int16_t rc_ch0;
  int16_t rc_ch1;
  int16_t rc_ch2;
  int16_t rc_ch3;
  uint8_t behaviour;
  int16_t manual_yaw_add_mrad;
  int16_t manual_pitch_add_mrad;
  uint8_t yaw_mode;
  uint8_t pitch_mode;
  int16_t yaw_set_mrad;
  int16_t pitch_set_mrad;
  int16_t yaw_given_current;
  int16_t pitch_given_current;
};

// 将整数波特率映射为 termios 常量；不支持时默认 921600。
speed_t ToBaudRate(int baud_rate) {
  switch (baud_rate) {
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      return B921600;
  }
}

// 将浮点值限制并四舍五入到 uint16 范围，防止坐标越界---被OnTargets调用
uint16_t ClampToUInt16(double value) {
  if (value < 0.0) {
    return 0;
  }
  if (value > static_cast<double>(std::numeric_limits<uint16_t>::max())) {
    return std::numeric_limits<uint16_t>::max();
  }
  return static_cast<uint16_t>(std::lround(value));
}

// 从小端字节序读取无符号 16 位整数。
uint16_t ReadLe16(const uint8_t *data) {
  return static_cast<uint16_t>(static_cast<uint16_t>(data[0]) |
                               (static_cast<uint16_t>(data[1]) << 8));
}

// 从小端字节序读取有符号 16 位整数。
int16_t ReadLeI16(const uint8_t *data) {
  return static_cast<int16_t>(ReadLe16(data));
}

// 计算诊断帧校验值（按协议对指定字节区间异或）。
uint8_t VisionDiagChecksum(const uint8_t *frame) {
  // 协议定义：校验范围为 [2..42]，与 frame[43] 比较。
  uint8_t checksum = 0;
  for (size_t i = 2; i <= 44; ++i) {
    checksum ^= frame[i];
  }
  return checksum;
}

}  // namespace

class GimbalSerialBridgeNode : public rclcpp::Node {
 public:
  // 构造节点：加载参数、打开串口、创建订阅与诊断轮询定时器。
  GimbalSerialBridgeNode() : Node("rm_gimbal_bridge") {
    // 视觉输入、串口与目标筛选参数。
    input_topic_ = declare_parameter<std::string>("input_topic", "/dnn_node_sample");
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyS1");
    baud_rate_ = declare_parameter<int>("baud_rate", 921600);

    image_width_ = declare_parameter<int>("image_width", 1280);
    image_height_ = declare_parameter<int>("image_height", 1024);
    image_center_x_ = declare_parameter<double>("image_center_x", image_width_ / 2.0);
    image_center_y_ = declare_parameter<double>("image_center_y", image_height_ / 2.0);
    min_confidence_ = declare_parameter<double>("min_confidence", 0.5);
    enemy_prefix_ = declare_parameter<std::string>("enemy_prefix", "");

    selection_mode_ = declare_parameter<std::string>("selection_mode", "closest");
    log_selected_target_ = declare_parameter<bool>("log_selected_target", true);
    log_diag_feedback_ = declare_parameter<bool>("log_diag_feedback", true);
    require_lower_vision_enabled_ =
      declare_parameter<bool>("require_lower_vision_enabled", true);
    lower_diag_timeout_ms_ = declare_parameter<int>("lower_diag_timeout_ms", 500);
    lower_vision_latch_ms_ = declare_parameter<int>("lower_vision_latch_ms", 5000);
    target_hold_ms_ = declare_parameter<int>("target_hold_ms", 300);


    if (!OpenSerialPort()) {
      RCLCPP_ERROR(get_logger(), "Serial port init failed: %s", serial_port_.c_str());
    }
    // 订阅视觉目标，触发回调函数---OnTargets
    subscription_ = create_subscription<ai_msgs::msg::PerceptionTargets>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&GimbalSerialBridgeNode::OnTargets, this, std::placeholders::_1));

    // 周期轮询下位机诊断反馈，与目标回调解耦，触发定时器回调函数---PollDiagnostics
    diag_timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&GimbalSerialBridgeNode::PollDiagnostics, this));
  }

  // 析构节点：关闭串口文件描述符，释放系统资源。
  ~GimbalSerialBridgeNode() override {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
    }
  }

 private:
  // 配置串口+打开串口---发送到下位机时调用
  bool OpenSerialPort() {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
    }

    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "Open serial port failed: %s", std::strerror(errno));
      return false;
    }

    termios tty {};
    if (tcgetattr(serial_fd_, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "Read serial attributes failed: %s", std::strerror(errno));
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }

    cfsetospeed(&tty, ToBaudRate(baud_rate_));
    cfsetispeed(&tty, ToBaudRate(baud_rate_));
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;
    // 非阻塞读取：由定时器轮询 read() 聚合数据。
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "Apply serial attributes failed: %s", std::strerror(errno));
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }

    return true;
  }

  // 串口写失败后的恢复入口：记录告警并重新打开串口。---发送到下位机时调用
  bool ReopenSerialPort() {
    RCLCPP_WARN(get_logger(), "Reopening serial port: %s", serial_port_.c_str());
    return OpenSerialPort();
  }

  // 根据下位机视觉使能门控判断当前是否允许下发控制帧。---被OnTargets调用
  bool LowerVisionControlAllowed() const {
    // 可选“下位机视觉使能门控”：未使能时只看反馈不下发目标。
    if (!require_lower_vision_enabled_) {
      return true;
    }
    return lower_vision_latched_;
  }

  // 视觉目标回调
  void OnTargets(const ai_msgs::msg::PerceptionTargets::SharedPtr msg) {
    // 1) 基础保护：消息空或串口未就绪时直接返回。
    if (!msg || serial_fd_ < 0) {
      return;
    }

    // 2) 先按当前策略从本帧检测结果中选目标---调用函数SelectTarget---从检测结果中挑选一个最优目标。
    auto selected = SelectTarget(*msg);
    const auto stamp = now();

    // 3) 若本帧选到目标，刷新“最近有效目标”；若未选到则尝试短时沿用上一目标。
    if (selected.has_value()) {
      last_target_ = selected;
      last_target_stamp_ = stamp;
    } else if (last_target_.has_value()) {
      // 短暂丢检时保留上一帧目标，降低抖动。
      const auto age = stamp - last_target_stamp_;
      if (age <= rclcpp::Duration::from_seconds(static_cast<double>(target_hold_ms_) / 1000.0)) {
        selected = last_target_;
      }
    }

    // 4) 仍无可用目标则结束本次回调。
    if (!selected.has_value()) {
      return;
    }

    // 5) 将中心坐标转换为 uint16（带边界裁剪）。
    const uint16_t x = ClampToUInt16(selected->center_x);
    const uint16_t y = ClampToUInt16(selected->center_y);

    // 6) 门控判定：若下位机未反馈视觉使能，则本次不下发控制帧。
    if (!LowerVisionControlAllowed()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "feedback-only: lower vision not enabled yet, skip send target=%s center=(%u,%u) conf=%.3f",
        selected->type.c_str(), x, y, selected->confidence);
      return;
    }

    // 7) 发送到下位机。
    SendFrame(x, y);

    // 8) 日志
    if (log_selected_target_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "target=%s center=(%u,%u) conf=%.3f dist=%.2f",
        selected->type.c_str(), x, y, selected->confidence, selected->distance_to_center);
    }
  }

  // 定时器触发回调---解析下位机传来的数据包
  void PollDiagnostics() {
    std::array<uint8_t, 256> buffer {};
    ssize_t count;
    if (serial_fd_ < 0) {
      return;
    }

    do {
      count = read(serial_fd_, buffer.data(), buffer.size());
      if (count > 0) {
        // 将串口分片数据拼接到环外缓冲，后续按协议帧解析。
        diag_rx_buffer_.insert(diag_rx_buffer_.end(), buffer.begin(), buffer.begin() + count);
      }
    } while (count > 0);
    // 解析累积的诊断数据。
    ParseDiagnostics();
  }

  // 解析下位机数据
  void ParseDiagnostics() {
    while (diag_rx_buffer_.size() >= kVisionDiagFrameSize) {
      if (diag_rx_buffer_[0] != kVisionDiagHead0 || diag_rx_buffer_[1] != kVisionDiagHead1) {
        // 帧头不同步时逐字节丢弃，直到重新对齐。
        diag_rx_buffer_.erase(diag_rx_buffer_.begin());
        continue;
      }

      const auto *frame = diag_rx_buffer_.data();
      if (frame[44] != kVisionDiagTail0 || frame[45] != kVisionDiagTail1 ||
          VisionDiagChecksum(frame) != frame[43]) {
        // 帧尾或校验失败同样逐字节重同步，避免整包误丢。
        diag_rx_buffer_.erase(diag_rx_buffer_.begin());
        continue;
      }

      // 按小端格式解包诊断数据。
      const VisionDiagFrame diag {
        frame[2],
        frame[3],
        ReadLe16(&frame[4]),
        ReadLe16(&frame[6]),
        ReadLeI16(&frame[8]),
        ReadLeI16(&frame[10]),
        ReadLeI16(&frame[12]),
        ReadLeI16(&frame[14]),
        ReadLe16(&frame[16]),
        ReadLe16(&frame[18]),
        frame[20],
        frame[21],
        ReadLeI16(&frame[22]),
        ReadLeI16(&frame[24]),
        ReadLeI16(&frame[26]),
        ReadLeI16(&frame[28]),
        frame[30],
        ReadLeI16(&frame[31]),
        ReadLeI16(&frame[33]),
        frame[35],
        frame[36],
        ReadLeI16(&frame[37]),
        ReadLeI16(&frame[39]),
        ReadLeI16(&frame[41]),
        ReadLeI16(&frame[43]),
      };

      last_diag_ = diag;
      last_diag_stamp_ = now();

      const bool vision_enabled = (diag.flags & kVisionDiagFlagVisionEnabled) != 0;
      if (vision_enabled) {
        // 仅在收到“视觉使能”反馈时放行上位机下发目标。
        lower_vision_latched_ = true;
        lower_vision_latched_stamp_ = last_diag_stamp_;
      } else {
        lower_vision_latched_ = false;
      }

      if (log_diag_feedback_) {
        const bool target_valid = (diag.flags & kVisionDiagFlagTargetValid) != 0;
        const bool link_online = (diag.flags & kVisionDiagFlagLinkOnline) != 0;
        const bool rc_error = (diag.flags & kVisionDiagFlagRcError) != 0;
        const bool dbus_toe = (diag.flags & kVisionDiagFlagDbusToe) != 0;
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 200,
          "diag vision=%d latched=%d target=%d link=%d rc_error=%d dbus_toe=%d behaviour=%u sw=(%u,%u) ch=(%d,%d,%d,%d) seq=%u raw=(%u,%u) err=(%d,%d) vision_add=(%d,%d) manual_add=(%d,%d) mode=(%u,%u) set=(%d,%d) current=(%d,%d) parsed=%u rx=%u",
          vision_enabled, lower_vision_latched_, target_valid, link_online, rc_error, dbus_toe, diag.behaviour,
          diag.rc_sw0, diag.rc_sw1, diag.rc_ch0, diag.rc_ch1, diag.rc_ch2, diag.rc_ch3,
          diag.seq, diag.raw_x, diag.raw_y, diag.error_x, diag.error_y, diag.yaw_add_mrad, diag.pitch_add_mrad,
          diag.manual_yaw_add_mrad, diag.manual_pitch_add_mrad,
          diag.yaw_mode, diag.pitch_mode, diag.yaw_set_mrad, diag.pitch_set_mrad,
          diag.yaw_given_current, diag.pitch_given_current,
          diag.parsed_frames, diag.rx_bytes);

        if (vision_enabled && target_valid) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 200,
            "tune err=(%d,%d) add=(%d,%d)mrad pitch_set=%dmrad current=(%d,%d) seq=%u",
            diag.error_x, diag.error_y, diag.yaw_add_mrad, diag.pitch_add_mrad,
            diag.pitch_set_mrad, diag.yaw_given_current, diag.pitch_given_current, diag.seq);
        }
      }

      diag_rx_buffer_.erase(diag_rx_buffer_.begin(), diag_rx_buffer_.begin() + kVisionDiagFrameSize);
    }
  }

  // 从检测结果中挑选一个最优目标---被OnTargets调用
  std::optional<TargetCandidate> SelectTarget(const ai_msgs::msg::PerceptionTargets &msg) const {
    std::optional<TargetCandidate> best;
    for (const auto &target : msg.targets) {
      // 支持按敌我前缀过滤类型名（例如 red_ / blue_）。
      if (!enemy_prefix_.empty() && target.type.rfind(enemy_prefix_, 0) != 0) {
        continue;
      }
      if (target.rois.empty()) {
        continue;
      }
      const auto &roi = target.rois.front();
      if (roi.confidence < min_confidence_) {
        continue;
      }

      const double center_x = static_cast<double>(roi.rect.x_offset) +
                              static_cast<double>(roi.rect.width) / 2.0;
      const double center_y = static_cast<double>(roi.rect.y_offset) +
                              static_cast<double>(roi.rect.height) / 2.0;
      const double dx = center_x - image_center_x_;
      const double dy = center_y - image_center_y_;
      const double distance = std::hypot(dx, dy);

      TargetCandidate current {target.type, center_x, center_y, roi.confidence, distance};
      if (!best.has_value()) {
        best = current;
        continue;
      }
      // 二选一策略：最高置信度 / 最接近图像中心。
      if (selection_mode_ == "highest_confidence") {
        if (current.confidence > best->confidence) {
          best = current;
        }
      } else if (current.distance_to_center < best->distance_to_center) {
        best = current;
      }
    }
    return best;
  }

  // 发送到下位机
  void SendFrame(uint16_t x, uint16_t y) {
    // 上行控制帧：FA FB xL xH yL yH FC FD。
    const std::array<uint8_t, 8> frame = {
      0xFA, 0xFB,
      static_cast<uint8_t>(x & 0xFF),
      static_cast<uint8_t>((x >> 8) & 0xFF),
      static_cast<uint8_t>(y & 0xFF),
      static_cast<uint8_t>((y >> 8) & 0xFF),
      0xFC, 0xFD,
    };

    if (serial_fd_ < 0 && !OpenSerialPort()) {
      return;
    }

    auto written = write(serial_fd_, frame.data(), frame.size());
    if (written == static_cast<ssize_t>(frame.size())) {
      return;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Serial write incomplete: %zd/%zu", written, frame.size());

    if (!ReopenSerialPort()) {
      return;
    }

    // 一次重连后重发，兼顾实时性与简单容错。
    written = write(serial_fd_, frame.data(), frame.size());
    if (written != static_cast<ssize_t>(frame.size())) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Serial write incomplete after reopen: %zd/%zu", written, frame.size());
    }
  }

  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
  std::vector<uint8_t> diag_rx_buffer_;
  std::optional<VisionDiagFrame> last_diag_;
  rclcpp::Time last_diag_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time lower_vision_latched_stamp_{0, 0, RCL_ROS_TIME};
  std::string input_topic_;
  std::string serial_port_;
  std::string enemy_prefix_;
  std::string selection_mode_;
  int baud_rate_ = 921600;
  int image_width_ = 1280;
  int image_height_ = 1024;
  int lower_diag_timeout_ms_ = 500;
  int lower_vision_latch_ms_ = 5000;
  int target_hold_ms_ = 300;
  double image_center_x_ = 640.0;
  double image_center_y_ = 512.0;
  double min_confidence_ = 0.5;
  bool log_selected_target_ = true;
  bool log_diag_feedback_ = true;
  bool require_lower_vision_enabled_ = true;
  bool lower_vision_latched_ = false;
  std::optional<TargetCandidate> last_target_;
  rclcpp::Time last_target_stamp_{0, 0, RCL_ROS_TIME};
  int serial_fd_ = -1;
};

// ROS2 程序入口：初始化、运行节点、自旋退出后清理。
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalSerialBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
