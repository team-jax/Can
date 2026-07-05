#include "ak45_36_socketcan_control.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <signal.h>
#include <time.h>
#include <pthread.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// ─── 내부 상태 ────────────────────────────────────────────────────────────────
static int              can_sock = -1;
static volatile int     g_running = 0;
static MotorState       g_state[NUM_MOTORS];
static pthread_mutex_t  g_state_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_t        g_rx_thread;

// controller_id → g_state 배열 인덱스. 등록되지 않은 ID면 -1.
static int motor_index(uint8_t controller_id)
{
    if (controller_id == CONTROLLER_ID_1) return 0;
    if (controller_id == CONTROLLER_ID_2) return 1;
    return -1;
}

static uint8_t get_error_code(uint8_t controller_id)
{
    int idx = motor_index(controller_id);
    if (idx < 0) return ERR_NONE;
    pthread_mutex_lock(&g_state_mutex);
    uint8_t err = g_state[idx].error_code;
    pthread_mutex_unlock(&g_state_mutex);
    return err;
}

// ─── 유틸리티 ─────────────────────────────────────────────────────────────────
static void buffer_append_int32(uint8_t *buf, int32_t val, int *idx)
{
    buf[(*idx)++] = (val >> 24) & 0xFF;
    buf[(*idx)++] = (val >> 16) & 0xFF;
    buf[(*idx)++] = (val >>  8) & 0xFF;
    buf[(*idx)++] =  val        & 0xFF;
}

static long ms_elapsed(const struct timespec *t)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec  - t->tv_sec)  * 1000L +
           (now.tv_nsec - t->tv_nsec) / 1000000L;
}

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static int32_t clampi(int32_t v, int32_t lo, int32_t hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

// ─── CAN 송신 (Extended Frame) ────────────────────────────────────────────────
static int can_transmit_eid(uint32_t eid, const uint8_t *data, uint8_t len)
{
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id  = (eid & CAN_EFF_MASK) | CAN_EFF_FLAG;
    frame.can_dlc = len;
    memcpy(frame.data, data, len);

    if (write(can_sock, &frame, sizeof(frame)) != sizeof(frame)) {
        perror("[ak45] CAN write");
        return -1;
    }
    return 0;
}

// ─── 피드백 파싱 (매뉴얼 5.2절) ──────────────────────────────────────────────
static void parse_feedback(uint8_t controller_id, const uint8_t *data, uint8_t dlc)
{
    int idx = motor_index(controller_id);
    if (idx < 0) return;
    if (dlc < 8) return;

    int16_t pos_raw = (int16_t)((data[0] << 8) | data[1]);
    int16_t spd_raw = (int16_t)((data[2] << 8) | data[3]);
    int16_t cur_raw = (int16_t)((data[4] << 8) | data[5]);

    MotorState s;
    s.position_deg   = pos_raw * 0.1f;      // deg
    s.speed_erpm     = spd_raw * 10.0f;     // ERPM
    s.current_a      = cur_raw * 0.01f;     // A
    s.temperature_c  = (int8_t)data[6];     // ℃, 오프셋 없음
    s.error_code     = data[7];
    s.valid          = 1;
    clock_gettime(CLOCK_MONOTONIC, &s.last_rx);

    pthread_mutex_lock(&g_state_mutex);
    g_state[idx] = s;
    pthread_mutex_unlock(&g_state_mutex);

    // 에러 코드 비정상 시 경고 출력 (§5 규칙 3)
    if (s.error_code != ERR_NONE) {
        fprintf(stderr, "[ak45] ID=0x%02X ERROR code=%d (%s), 명령 중단 권고\n",
                controller_id, s.error_code, ak45_error_str(s.error_code));
    }
}

// ─── 수신 스레드 ──────────────────────────────────────────────────────────────
static void *rx_thread(void *arg)
{
    (void)arg;
    struct can_frame frame;

    while (g_running) {
        ssize_t nbytes = read(can_sock, &frame, sizeof(frame));
        if (nbytes < 0) {
            if (errno == EINTR) continue;
            perror("[ak45] CAN read");
            break;
        }
        if ((size_t)nbytes < sizeof(frame)) continue;

        // Extended 프레임만 처리
        if (!(frame.can_id & CAN_EFF_FLAG)) continue;

        uint32_t eid = frame.can_id & CAN_EFF_MASK;
        uint8_t  rx_func_id     = (eid >> 8) & 0xFF;
        uint8_t  rx_controller_id = eid & 0xFF;
        if (rx_func_id == FEEDBACK_FUNC_ID) {
            parse_feedback(rx_controller_id, frame.data, frame.can_dlc);
        }
    }
    return NULL;
}

// ─── 워치독 확인 ──────────────────────────────────────────────────────────────
int ak45_is_watchdog_ok(uint8_t controller_id)
{
    int idx = motor_index(controller_id);
    if (idx < 0) return 0;

    pthread_mutex_lock(&g_state_mutex);
    int valid = g_state[idx].valid;
    long elapsed = valid ? ms_elapsed(&g_state[idx].last_rx) : WATCHDOG_TIMEOUT_MS + 1;
    pthread_mutex_unlock(&g_state_mutex);

    return valid && (elapsed < WATCHDOG_TIMEOUT_MS);
}

// ─── 초기화 ───────────────────────────────────────────────────────────────────
int ak45_init(void)
{
    can_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_sock < 0) {
        perror("[ak45] socket");
        return -1;
    }

    struct ifreq ifr;
    strncpy(ifr.ifr_name, CAN_INTERFACE, IFNAMSIZ - 1);
    if (ioctl(can_sock, SIOCGIFINDEX, &ifr) < 0) {
        perror("[ak45] ioctl SIOCGIFINDEX");
        close(can_sock);
        can_sock = -1;
        return -1;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("[ak45] bind");
        close(can_sock);
        can_sock = -1;
        return -1;
    }

    memset(g_state, 0, sizeof(g_state));
    g_running = 1;

    if (pthread_create(&g_rx_thread, NULL, rx_thread, NULL) != 0) {
        perror("[ak45] pthread_create");
        close(can_sock);
        can_sock = -1;
        return -1;
    }

    printf("[ak45] 초기화 완료. 인터페이스=%s, Controller_ID=0x%02X,0x%02X\n",
           CAN_INTERFACE, CONTROLLER_ID_1, CONTROLLER_ID_2);
    return 0;
}

// ─── 종료 ─────────────────────────────────────────────────────────────────────
void ak45_close(void)
{
    ak45_emergency_stop();
    g_running = 0;
    if (can_sock >= 0) {
        // 블로킹 read 해제를 위해 소켓 닫기
        close(can_sock);
        can_sock = -1;
    }
    pthread_join(g_rx_thread, NULL);
    printf("[ak45] 종료.\n");
}

// ─── 명령 함수들 ──────────────────────────────────────────────────────────────

int ak45_set_duty(uint8_t controller_id, float duty)
{
    if (motor_index(controller_id) < 0) return -1;

    duty = clampf(duty, -0.95f, 0.95f);
    int32_t val = (int32_t)(duty * 100000.0f);
    uint8_t buf[4];
    int idx = 0;
    buffer_append_int32(buf, val, &idx);
    uint32_t eid = ((uint32_t)CAN_PACKET_SET_DUTY << 8) | controller_id;
    return can_transmit_eid(eid, buf, 4);
}

int ak45_set_current(uint8_t controller_id, float current_a)
{
    if (motor_index(controller_id) < 0) return -1;

    // 에러 상태면 명령 차단
    uint8_t err = get_error_code(controller_id);
    if (err != ERR_NONE) {
        fprintf(stderr, "[ak45] ID=0x%02X 에러(%d) 상태에서 명령 차단\n", controller_id, err);
        return -1;
    }

    current_a = clampf(current_a, -SOFT_LIMIT_CURRENT_A, SOFT_LIMIT_CURRENT_A);
    int32_t val = (int32_t)(current_a * 1000.0f);
    uint8_t buf[4];
    int idx = 0;
    buffer_append_int32(buf, val, &idx);
    uint32_t eid = ((uint32_t)CAN_PACKET_SET_CURRENT << 8) | controller_id;
    return can_transmit_eid(eid, buf, 4);
}

int ak45_set_current_brake(uint8_t controller_id, float current_a)
{
    if (motor_index(controller_id) < 0) return -1;

    current_a = clampf(current_a, 0.0f, SOFT_LIMIT_CURRENT_A);
    int32_t val = (int32_t)(current_a * 1000.0f);
    uint8_t buf[4];
    int idx = 0;
    buffer_append_int32(buf, val, &idx);
    uint32_t eid = ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8) | controller_id;
    return can_transmit_eid(eid, buf, 4);
}

int ak45_set_rpm(uint8_t controller_id, int32_t erpm)
{
    if (motor_index(controller_id) < 0) return -1;

    uint8_t err = get_error_code(controller_id);
    if (err != ERR_NONE) {
        fprintf(stderr, "[ak45] ID=0x%02X 에러(%d) 상태에서 명령 차단\n", controller_id, err);
        return -1;
    }

    erpm = clampi(erpm, -SOFT_LIMIT_ERPM, SOFT_LIMIT_ERPM);
    uint8_t buf[4];
    int idx = 0;
    buffer_append_int32(buf, erpm, &idx);
    uint32_t eid = ((uint32_t)CAN_PACKET_SET_RPM << 8) | controller_id;
    return can_transmit_eid(eid, buf, 4);
}

int ak45_set_position(uint8_t controller_id, float deg)
{
    if (motor_index(controller_id) < 0) return -1;

    uint8_t err = get_error_code(controller_id);
    if (err != ERR_NONE) {
        fprintf(stderr, "[ak45] ID=0x%02X 에러(%d) 상태에서 명령 차단\n", controller_id, err);
        return -1;
    }

    deg = clampf(deg, -SOFT_LIMIT_POS_DEG, SOFT_LIMIT_POS_DEG);
    int32_t val = (int32_t)(deg * 10000.0f);
    uint8_t buf[4];
    int idx = 0;
    buffer_append_int32(buf, val, &idx);
    uint32_t eid = ((uint32_t)CAN_PACKET_SET_POS << 8) | controller_id;
    return can_transmit_eid(eid, buf, 4);
}

int ak45_set_origin(uint8_t controller_id, uint8_t permanent)
{
    if (motor_index(controller_id) < 0) return -1;

    // permanent=1은 듀얼 엔코더 모델 전용
    uint8_t buf[1] = { (uint8_t)(permanent ? 1u : 0u) };
    uint32_t eid = ((uint32_t)CAN_PACKET_SET_ORIGIN << 8) | controller_id;
    return can_transmit_eid(eid, buf, 1);
}

int ak45_set_pos_spd(uint8_t controller_id, float deg, int16_t spd_erpm_div10, int16_t acc)
{
    if (motor_index(controller_id) < 0) return -1;

    // 모드 6: pos(int32) + spd(int16, ERPM÷10) + acc(int16, 10 ERPM/s²)
    uint8_t err = get_error_code(controller_id);
    if (err != ERR_NONE) {
        fprintf(stderr, "[ak45] ID=0x%02X 에러(%d) 상태에서 명령 차단\n", controller_id, err);
        return -1;
    }

    deg = clampf(deg, -SOFT_LIMIT_POS_DEG, SOFT_LIMIT_POS_DEG);
    int32_t pos_val = (int32_t)(deg * 10000.0f);

    uint8_t buf[8];
    buf[0] = (pos_val >> 24) & 0xFF;
    buf[1] = (pos_val >> 16) & 0xFF;
    buf[2] = (pos_val >>  8) & 0xFF;
    buf[3] =  pos_val        & 0xFF;
    buf[4] = (spd_erpm_div10 >> 8) & 0xFF;
    buf[5] =  spd_erpm_div10       & 0xFF;
    buf[6] = (acc >> 8) & 0xFF;
    buf[7] =  acc       & 0xFF;

    uint32_t eid = ((uint32_t)CAN_PACKET_SET_POS_SPD << 8) | controller_id;
    return can_transmit_eid(eid, buf, 8);
}

int ak45_emergency_stop_one(uint8_t controller_id)
{
    // Current Brake 0A 송신 (§5 규칙 2)
    return ak45_set_current_brake(controller_id, 0.0f);
}

int ak45_emergency_stop(void)
{
    int r1 = ak45_emergency_stop_one(CONTROLLER_ID_1);
    int r2 = ak45_emergency_stop_one(CONTROLLER_ID_2);
    return (r1 == 0 && r2 == 0) ? 0 : -1;
}

// ─── 상태 조회 ────────────────────────────────────────────────────────────────
MotorState ak45_get_state(uint8_t controller_id)
{
    MotorState s;
    memset(&s, 0, sizeof(s));
    int idx = motor_index(controller_id);
    if (idx < 0) return s;

    pthread_mutex_lock(&g_state_mutex);
    s = g_state[idx];
    pthread_mutex_unlock(&g_state_mutex);
    return s;
}

const char *ak45_error_str(uint8_t code)
{
    switch ((MotorError)code) {
        case ERR_NONE:            return "정상";
        case ERR_MOTOR_OVERHEAT:  return "모터 과열";
        case ERR_OVERCURRENT:     return "과전류";
        case ERR_OVERVOLTAGE:     return "과전압";
        case ERR_UNDERVOLTAGE:    return "저전압";
        case ERR_ENCODER_FAULT:   return "엔코더 고장";
        case ERR_MOSFET_OVERHEAT: return "MOSFET 과열";
        case ERR_MOTOR_STALL:     return "모터 스톨";
        default:                  return "알 수 없음";
    }
}
