// ak45_node.cpp
//
// CubeMars AK45-36 6개를 하나의 노드로 다룬다.
//   Phase 1 : CAN 피드백을 폴링해 /joint_states 와 /diagnostics 로 발행 (읽기)
//   Phase 2 : /ak45/command 를 구독해 ak45_set_position() 으로 재송신 (쓰기)
//
// 노드가 하나인 이유: 라이브러리가 flock(/tmp/ak45_ctrl.lock)으로 프로세스당
// 1개만 ak45_init() 에 성공시킨다 (ros2.md 부록 B C1). 상태 발행과 명령 수신을
// 별도 프로세스로 나누면 나중에 뜬 쪽이 반드시 죽는다.

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include "ak45_36_socketcan_control.h"

class Ak45Node : public rclcpp::Node
{
public:
  Ak45Node();
  ~Ak45Node() override;

private:
  // Phase 1 — 상태 발행
  void onStateTimer();   // 50Hz, /joint_states
  void onDiagTimer();    // 2Hz,  /diagnostics

  // Phase 2 — 명령 수신
  void onCommand(const sensor_msgs::msg::JointState::SharedPtr msg);  // 목표값 저장만
  void onCommandTimer();                                              // 10Hz 재송신

  diagnostic_msgs::msg::DiagnosticStatus makeStatus(
    std::size_t index, uint8_t motor_id,
    const MotorState & st, bool watchdog_ok) const;

  static std::string fmt2(double v);   // 소수 2자리 (snprintf)

  static constexpr std::size_t kNumMotors = 6;
  static constexpr uint8_t kMotorIds[kNumMotors] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};

  std::vector<std::string> joint_names_;
  double publish_rate_hz_{50.0};
  double diagnostics_rate_hz_{2.0};
  double command_resend_rate_hz_{10.0};
  double max_command_deg_{15.0};
  double command_timeout_sec_{0.0};
  double max_temperature_c_{60.0};
  bool can_ready_{false};   // ak45_init() 성공 여부. 소멸자 가드에 사용 (부록 B C7)

  // 목표값은 deg 로 저장한다. 라이브러리 ak45_set_position() 이 deg 를 받기 때문이다.
  // 구독 콜백과 command_timer_ 가 공유하지만 둘 다 기본 콜백 그룹
  // (MutuallyExclusive)이라 동시에 실행되지 않는다 → 잠금 불필요 (§6.3).
  std::array<double, kNumMotors> target_deg_{};
  std::array<bool, kNumMotors> has_target_{};      // 전부 false. 첫 명령 전 송신 금지
  std::array<bool, kNumMotors> temp_locked_{};     // S2 래치. 자동 복귀 없음
  std::array<bool, kNumMotors> command_active_{};  // 진단 표시용
  rclcpp::Time last_command_time_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
  rclcpp::TimerBase::SharedPtr command_timer_;
};

Ak45Node::Ak45Node()
: rclcpp::Node("ak45_node"), last_command_time_(this->now())
{
  // 2. 파라미터 선언·읽기 (§4.6)
  const std::vector<std::string> default_joint_names = {
    "ak45_1", "ak45_2", "ak45_3", "ak45_4", "ak45_5", "ak45_6"};

  publish_rate_hz_     = this->declare_parameter<double>("publish_rate_hz", 50.0);
  diagnostics_rate_hz_ = this->declare_parameter<double>("diagnostics_rate_hz", 2.0);
  joint_names_         = this->declare_parameter<std::vector<std::string>>(
    "joint_names", default_joint_names);
  command_resend_rate_hz_ = this->declare_parameter<double>("command_resend_rate_hz", 10.0);
  max_command_deg_        = this->declare_parameter<double>("max_command_deg", 15.0);
  command_timeout_sec_    = this->declare_parameter<double>("command_timeout_sec", 0.0);
  max_temperature_c_      = this->declare_parameter<double>("max_temperature_c", 60.0);

  // 3. 파라미터 검증 — ak45_init() 보다 먼저 (§A.3).
  //    순서를 바꾸면 throw 시 소켓과 flock 이 프로세스 종료까지 남는다.
  if (!(publish_rate_hz_ > 0.0 && publish_rate_hz_ <= 500.0)) {
    RCLCPP_FATAL(
      this->get_logger(),
      "publish_rate_hz는 0보다 크고 500 이하여야 합니다. 입력값=%.3f", publish_rate_hz_);
    throw std::runtime_error("invalid publish_rate_hz");
  }
  if (!(diagnostics_rate_hz_ > 0.0 && diagnostics_rate_hz_ <= 50.0)) {
    RCLCPP_FATAL(
      this->get_logger(),
      "diagnostics_rate_hz는 0보다 크고 50 이하여야 합니다. 입력값=%.3f", diagnostics_rate_hz_);
    throw std::runtime_error("invalid diagnostics_rate_hz");
  }
  if (joint_names_.size() != kNumMotors) {
    RCLCPP_FATAL(
      this->get_logger(),
      "joint_names는 정확히 6개여야 합니다(AK45 ID 0x01~0x06 순서). 입력 개수=%zu",
      joint_names_.size());
    throw std::runtime_error("invalid joint_names size");
  }
  if (!(command_resend_rate_hz_ > 0.0 && command_resend_rate_hz_ <= 50.0)) {
    RCLCPP_FATAL(
      this->get_logger(),
      "command_resend_rate_hz는 0보다 크고 50 이하여야 합니다. 입력값=%.3f",
      command_resend_rate_hz_);
    throw std::runtime_error("invalid command_resend_rate_hz");
  }
  if (!(max_command_deg_ > 0.0 && max_command_deg_ <= 360.0)) {
    RCLCPP_FATAL(
      this->get_logger(),
      "max_command_deg는 0보다 크고 360 이하여야 합니다. 입력값=%.3f", max_command_deg_);
    throw std::runtime_error("invalid max_command_deg");
  }
  if (!(command_timeout_sec_ >= 0.0)) {
    RCLCPP_FATAL(
      this->get_logger(),
      "command_timeout_sec는 0 이상이어야 합니다(0=타임아웃 없음). 입력값=%.3f",
      command_timeout_sec_);
    throw std::runtime_error("invalid command_timeout_sec");
  }

  // 4. CAN 초기화
  if (ak45_init() != 0) {
    RCLCPP_FATAL(this->get_logger(), "ak45_init() 실패. 다음을 확인하세요:");
    RCLCPP_FATAL(
      this->get_logger(),
      "  1) can0 활성화: sudo ip link set can0 up type can bitrate 1000000");
    RCLCPP_FATAL(
      this->get_logger(),
      "  2) 다른 AK45 프로세스 실행 여부: fuser /tmp/ak45_ctrl.lock");
    throw std::runtime_error("ak45_init failed");
  }
  can_ready_ = true;

  // 5. 퍼블리셔·구독자 (상대 이름, 앞에 '/' 금지)
  joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "joint_states", rclcpp::QoS(rclcpp::KeepLast(10)));
  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "diagnostics", rclcpp::QoS(rclcpp::KeepLast(10)));
  command_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "ak45/command", rclcpp::QoS(rclcpp::KeepLast(10)),
    std::bind(&Ak45Node::onCommand, this, std::placeholders::_1));

  // 6. 타이머 3개. 전부 기본 콜백 그룹(MutuallyExclusive).
  const auto to_ns = [](double hz) {
      return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / hz));
    };
  state_timer_ = this->create_wall_timer(
    to_ns(publish_rate_hz_), std::bind(&Ak45Node::onStateTimer, this));
  diag_timer_ = this->create_wall_timer(
    to_ns(diagnostics_rate_hz_), std::bind(&Ak45Node::onDiagTimer, this));
  command_timer_ = this->create_wall_timer(
    to_ns(command_resend_rate_hz_), std::bind(&Ak45Node::onCommandTimer, this));

  // 7.
  RCLCPP_INFO(
    this->get_logger(),
    "ak45_node 시작. 인터페이스=can0, 상태=%.1fHz, 진단=%.1fHz, 명령 재송신=%.1fHz, "
    "목표각 상한=±%.1f도, 온도 상한=%.1f도, 명령 타임아웃=%s",
    publish_rate_hz_, diagnostics_rate_hz_, command_resend_rate_hz_,
    max_command_deg_, max_temperature_c_,
    (command_timeout_sec_ > 0.0) ? "켜짐" : "없음(마지막 목표 홀딩)");
  RCLCPP_INFO(
    this->get_logger(),
    "첫 명령을 받기 전까지 명령 프레임을 보내지 않습니다. 발행 예: "
    "ros2 topic pub -1 /ak45/command sensor_msgs/msg/JointState "
    "\"{name: ['%s'], position: [0.0]}\"", joint_names_[1].c_str());
}

Ak45Node::~Ak45Node()
{
  // can_ready_ 가드는 필수다. ak45_close()는 pthread_join()을 무조건 호출하므로
  // init 실패로 스레드가 없으면 미정의 동작이 된다 (부록 B C7).
  if (can_ready_) {
    ak45_close();   // 내부에서 브레이크 0A 6프레임 → 소켓/락 해제 (§7.5)
    can_ready_ = false;
  }
}

std::string Ak45Node::fmt2(double v)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.2f", v);
  return std::string(buf);
}

void Ak45Node::onStateTimer()
{
  sensor_msgs::msg::JointState js;
  js.header.stamp = this->now();
  // js.header.frame_id 는 기본값 "" 그대로 둔다 (§4.5)
  // js.velocity, js.effort 도 손대지 않는다 (§7.4)

  for (std::size_t i = 0; i < kNumMotors; ++i) {
    const MotorState st = ak45_get_state(kMotorIds[i]);
    if (st.valid != 0) {
      js.name.push_back(joint_names_[i]);
      js.position.push_back(static_cast<double>(st.position_deg) * M_PI / 180.0);
    }
  }

  joint_pub_->publish(js);
}

// 목표값을 멤버에 저장만 한다. CAN 송신은 onCommandTimer() 가 전담한다.
// 여기서 바로 보내면 재송신 주기가 발행자 쪽 주기에 끌려다니게 된다 (부록 B C3).
void Ak45Node::onCommand(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // ── 패스 1: 메시지 전체 검증. 하나라도 걸리면 메시지 전체를 버린다. ──
  if (msg->name.size() != msg->position.size()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "name과 position 길이가 다릅니다 (name=%zu, position=%zu). 메시지를 무시합니다.",
      msg->name.size(), msg->position.size());
    return;
  }

  std::vector<std::size_t> indices;
  indices.reserve(msg->name.size());

  for (std::size_t k = 0; k < msg->name.size(); ++k) {
    // 이름 매칭. 인덱스 기반으로 하면 /joint_states 가 valid==0 모터를 빼서
    // 배열 길이가 변하는 것과 같은 함정에 빠진다 (§4.3).
    std::size_t idx = kNumMotors;
    for (std::size_t i = 0; i < kNumMotors; ++i) {
      if (joint_names_[i] == msg->name[k]) {
        idx = i;
        break;
      }
    }
    if (idx == kNumMotors) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "알 수 없는 관절 이름: %s. 메시지를 무시합니다.", msg->name[k].c_str());
      return;
    }
    if (!std::isfinite(msg->position[k])) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "유효하지 않은 목표값 (관절 %s). 메시지를 무시합니다.", msg->name[k].c_str());
      return;
    }
    indices.push_back(idx);
  }

  // ── 패스 2: 관절별 적용. S1 은 메시지 전체가 아니라 그 관절만 거부한다. ──
  last_command_time_ = this->now();

  for (std::size_t k = 0; k < indices.size(); ++k) {
    const std::size_t i = indices[k];
    const double deg = msg->position[k] * 180.0 / M_PI;

    // S1: 클램프가 아니라 거부. 클램프하면 사람이 잘못 보낸 것을 모르는 채로
    //     모터가 상한까지 움직여버린다.
    if (std::fabs(deg) > max_command_deg_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "목표각 %.1f도가 상한 %.1f도를 넘습니다. 무시합니다. (기준축 판별 전)",
        deg, max_command_deg_);
      continue;   // target_deg_[i] 는 이전 값을 유지한다
    }

    target_deg_[i] = deg;
    has_target_[i] = true;
  }
}

// 100ms 주기 재송신. 라이브러리에 재송신 루프가 없고
// ak45_set_position() 1회 호출 = CAN 프레임 1개다 (부록 B C3).
// 재송신을 멈추면 모터 쪽 명령 타임아웃으로 홀딩 토크가 풀린다
// (demo_ramp.cpp:141-143 주석 근거).
void Ak45Node::onCommandTimer()
{
  // command_timeout_sec_ == 0.0 이면 타임아웃 없음 = 마지막 목표를 계속 홀딩한다.
  // 강화학습 정책이 끊겨도 팔이 갑자기 힘을 놓으면 중력으로 떨어지기 때문이다.
  if (command_timeout_sec_ > 0.0) {
    const double elapsed = (this->now() - last_command_time_).seconds();
    if (elapsed > command_timeout_sec_) {
      command_active_.fill(false);
      return;
    }
  }

  for (std::size_t i = 0; i < kNumMotors; ++i) {
    // 제약 3: 첫 명령을 받기 전에는 어떤 명령 프레임도 보내지 않는다.
    // S2 래치: 한 번 걸리면 온도가 내려가도 자동 복귀하지 않는다.
    if (!has_target_[i] || temp_locked_[i]) {
      command_active_[i] = false;
      continue;
    }

    const uint8_t id = kMotorIds[i];
    const MotorState st = ak45_get_state(id);
    const bool wd_ok = (ak45_is_watchdog_ok(id) != 0);

    // S3: 피드백 없는 모터에는 명령을 보내지 않는다.
    //     지금 버스에 ID 0x02 밖에 없으므로 이게 없으면 없는 모터 5대에
    //     100ms마다 프레임을 쏘게 된다.
    if (st.valid == 0 || !wd_ok) {
      command_active_[i] = false;
      continue;
    }

    // S2: 온도. 전류 0A에서도 계속 오른 이력이 있어 원인 미상이므로 보수적으로 간다.
    if (static_cast<double>(st.temperature_c) >= max_temperature_c_) {
      temp_locked_[i] = true;
      command_active_[i] = false;
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "ID 0x%02X 온도 %d도. 명령 송신을 중단합니다.",
        id, static_cast<int>(st.temperature_c));
      continue;
    }

    // S4: error_code != 0 이면 라이브러리가 -1 을 반환하고 프레임을 쏘지 않는다
    //     (부록 B C5). 반환값을 삼키지 않는다.
    if (ak45_set_position(id, static_cast<float>(target_deg_[i])) < 0) {
      command_active_[i] = false;
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "ID 0x%02X 명령 송신 실패 (에러코드 %d: %s)",
        id, static_cast<int>(st.error_code), ak45_error_str(st.error_code));
      continue;
    }

    command_active_[i] = true;
  }
}

void Ak45Node::onDiagTimer()
{
  diagnostic_msgs::msg::DiagnosticArray da;
  da.header.stamp = this->now();
  da.status.reserve(kNumMotors);

  int valid_count = 0;

  for (std::size_t i = 0; i < kNumMotors; ++i) {
    const uint8_t id = kMotorIds[i];
    const MotorState st = ak45_get_state(id);
    const bool wd_ok = (ak45_is_watchdog_ok(id) != 0);
    if (st.valid != 0) {++valid_count;}
    da.status.push_back(makeStatus(i, id, st, wd_ok));   // 항상 6개
  }

  diag_pub_->publish(da);

  if (valid_count == 0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "6개 모터 모두 피드백 없음. CubeMarsTool의 'Send status over CAN' Rate(Hz)가 0인지 확인하세요.");
  }
}

diagnostic_msgs::msg::DiagnosticStatus Ak45Node::makeStatus(
  std::size_t index, uint8_t motor_id,
  const MotorState & st, bool watchdog_ok) const
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "ak45/" + joint_names_[index];

  char hw_id[32];
  std::snprintf(hw_id, sizeof(hw_id), "ak45_id_0x%02X", motor_id);
  status.hardware_id = hw_id;

  // §8.2 판정: 위에서부터 먼저 걸리는 것 하나만.
  // valid 를 워치독보다 먼저 보아야 "아직 안 붙음"과 "붙었다가 끊김"이 구분된다.
  if (st.valid == 0) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "피드백 수신 없음 (CubeMarsTool의 Send status over CAN Rate(Hz) 확인)";
  } else if (!watchdog_ok) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "피드백 끊김: 200ms 초과";
  } else if (st.error_code != 0) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = ak45_error_str(st.error_code);
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "정상";
  }

  status.values.reserve(9);
  diagnostic_msgs::msg::KeyValue kv;

  kv.key = "position_deg";
  kv.value = fmt2(static_cast<double>(st.position_deg));
  status.values.push_back(kv);

  kv.key = "speed_erpm";
  kv.value = fmt2(static_cast<double>(st.speed_erpm));
  status.values.push_back(kv);

  kv.key = "current_a";
  kv.value = fmt2(static_cast<double>(st.current_a));
  status.values.push_back(kv);

  kv.key = "temperature_c";
  kv.value = std::to_string(static_cast<int>(st.temperature_c));
  status.values.push_back(kv);

  kv.key = "error_code";
  kv.value = std::to_string(static_cast<int>(st.error_code));
  status.values.push_back(kv);

  kv.key = "watchdog_ok";
  kv.value = watchdog_ok ? "1" : "0";
  status.values.push_back(kv);

  // Phase 2 추가 3개
  kv.key = "target_deg";
  kv.value = has_target_[index] ? fmt2(target_deg_[index]) : "none";
  status.values.push_back(kv);

  kv.key = "command_active";
  kv.value = command_active_[index] ? "1" : "0";
  status.values.push_back(kv);

  kv.key = "position_error_deg";
  kv.value = has_target_[index]
    ? fmt2(target_deg_[index] - static_cast<double>(st.position_deg))
    : "none";
  status.values.push_back(kv);

  return status;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int ret = 0;
  try {
    rclcpp::spin(std::make_shared<Ak45Node>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("ak45_node"), "노드 시작 실패: %s", e.what());
    ret = 1;
  }
  rclcpp::shutdown();
  return ret;
}
