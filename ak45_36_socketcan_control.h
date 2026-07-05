#pragma once

#include <stdint.h>
#include <time.h>
#include <pthread.h>

// ─── 프로토콜 상수 (매뉴얼 5.1~5.2절) ───────────────────────────────────────
#define CONTROLLER_ID_1      0x01       // 1번 모터 CAN ID
#define CONTROLLER_ID_2      0x02       // 2번 모터 CAN ID
#define NUM_MOTORS           2
#define CAN_INTERFACE        "can0"
#define CAN_BITRATE          1000000    // 1 Mbps (고정)

#define FEEDBACK_FUNC_ID     0x29
// 피드백 CAN ID = (0x29 << 8) | Controller_ID (모터별로 다름)
#define FEEDBACK_CAN_ID(controller_id)  (((uint32_t)FEEDBACK_FUNC_ID << 8) | (controller_id))

// ─── 소프트 리밋 (CLAUDE.md §4~5) ────────────────────────────────────────────
// 전류: 정격/피크 미확정 → 확인 전까지 ±5A
#define SOFT_LIMIT_CURRENT_A     5.0f
// ERPM: NPP 미확정 → 확인 전까지 보수적 상한. 확정 후 아래 공식으로 갱신.
//   출력축 6 rad/s ≈ 57 RPM → ERPM = 57 × NPP × 36 (NPP 확인 후 수정)
#define SOFT_LIMIT_ERPM          10000
// 위치: 프로토콜 최대 ±36000° 이내에서 운영 범위 제한
#define SOFT_LIMIT_POS_DEG       360.0f  // ±1회전

// ─── 워치독 타임아웃 ─────────────────────────────────────────────────────────
#define WATCHDOG_TIMEOUT_MS  200

// ─── CAN 패킷 모드 (매뉴얼 5.1) ─────────────────────────────────────────────
typedef enum {
    CAN_PACKET_SET_DUTY         = 0,  // int32, /100000 = duty
    CAN_PACKET_SET_CURRENT      = 1,  // int32, /1000 = A
    CAN_PACKET_SET_CURRENT_BRAKE = 2, // int32, /1000 = A (0~60A)
    CAN_PACKET_SET_RPM          = 3,  // int32, ERPM 그대로
    CAN_PACKET_SET_POS          = 4,  // int32, /10000 = deg
    CAN_PACKET_SET_ORIGIN       = 5,  // uint8, 0=임시/1=영구
    CAN_PACKET_SET_POS_SPD      = 6,  // int32+int16+int16, 8B
} CanPacketId;

// ─── 에러 코드 (매뉴얼 §2.3) ────────────────────────────────────────────────
typedef enum {
    ERR_NONE            = 0,
    ERR_MOTOR_OVERHEAT  = 1,
    ERR_OVERCURRENT     = 2,
    ERR_OVERVOLTAGE     = 3,
    ERR_UNDERVOLTAGE    = 4,
    ERR_ENCODER_FAULT   = 5,
    ERR_MOSFET_OVERHEAT = 6,
    ERR_MOTOR_STALL     = 7,
} MotorError;

// ─── 모터 상태 구조체 ────────────────────────────────────────────────────────
typedef struct {
    float    position_deg;     // int16 × 0.1 deg
    float    speed_erpm;       // int16 × 10 ERPM
    float    current_a;        // int16 × 0.01 A
    int8_t   temperature_c;    // int8 ℃ (오프셋 없음, MIT의 -40과 다름)
    uint8_t  error_code;       // MotorError
    struct timespec last_rx;   // 마지막 피드백 수신 시각
    int      valid;            // 피드백 수신 여부
} MotorState;

// ─── 공개 API ────────────────────────────────────────────────────────────────
int  ak45_init(void);          // SocketCAN 소켓 열기 + 피드백 스레드 시작
void ak45_close(void);         // 안전 정지 후 소켓 닫기

// 명령 함수 (클램핑 포함). controller_id는 CONTROLLER_ID_1/CONTROLLER_ID_2 중 하나.
int  ak45_set_duty(uint8_t controller_id, float duty);                  // 0.005~0.95
int  ak45_set_current(uint8_t controller_id, float current_a);          // ±SOFT_LIMIT_CURRENT_A
int  ak45_set_current_brake(uint8_t controller_id, float current_a);    // 0~SOFT_LIMIT_CURRENT_A
int  ak45_set_rpm(uint8_t controller_id, int32_t erpm);                 // ±SOFT_LIMIT_ERPM
int  ak45_set_position(uint8_t controller_id, float deg);               // ±SOFT_LIMIT_POS_DEG
int  ak45_set_origin(uint8_t controller_id, uint8_t permanent);         // 0=임시, 1=영구(듀얼 엔코더)
int  ak45_set_pos_spd(uint8_t controller_id, float deg, int16_t spd_erpm_div10, int16_t acc);

// 안전 정지 (Current Brake 0A)
int  ak45_emergency_stop(void);                       // 등록된 모든 모터 정지
int  ak45_emergency_stop_one(uint8_t controller_id);   // 지정 모터만 정지

// 상태 읽기
MotorState ak45_get_state(uint8_t controller_id);
int        ak45_is_watchdog_ok(uint8_t controller_id);   // 0=타임아웃, 1=정상
const char *ak45_error_str(uint8_t code);
