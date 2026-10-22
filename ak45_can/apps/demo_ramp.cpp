#include "ak45_36_socketcan_control.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

static volatile int g_quit = 0;
static void sig_handler(int sig) { (void)sig; g_quit = 1; }

static const uint8_t MOTOR_IDS[NUM_MOTORS] = {
    CONTROLLER_ID_1, CONTROLLER_ID_2, CONTROLLER_ID_3,
    CONTROLLER_ID_4, CONTROLLER_ID_5, CONTROLLER_ID_6
};

#define SEND_INTERVAL_MS   100    // 모터 명령 재송신 주기 (AGENTS.md §5 규칙4 - 타임아웃 방지)
#define DEFAULT_STEP_DEG   1.0f
#define DEFAULT_STEP_MS    150    // 1스텝(step_deg) 이동 후 다음 스텝까지 대기

// 모터별 램프(계단식 이동) 상태. current_deg가 final_deg에 도달할 때까지
// step_interval_ms마다 step_deg만큼씩 값을 옮겨가며, 그 사이에도 SEND_INTERVAL_MS
// 주기로 같은 목표를 계속 재전송한다 (blocking sleep 없이 tick 기반으로 진행).
typedef struct {
    int    active;    // 1=아직 목표로 이동 중
    int    stopped;   // 1=워치독 실패로 영구 정지, 더 이상 재송신 안 함
    float  final_deg;
    float  step_deg;
    float  current_deg;
    long   step_interval_ms;
    struct timespec last_step_time;
} RampState;

static long ms_since(const struct timespec *t)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec - t->tv_sec) * 1000L + (now.tv_nsec - t->tv_nsec) / 1000000L;
}

int main(int argc, char *argv[])
{
    if (argc < 2) {
        fprintf(stderr,
            "사용법: %s <목표각도1> [목표각도2 ... 최대 %d개] [--step=도] [--interval=ms]\n"
            "  예: %s 45                  ID1을 현재 위치에서 45도까지 1도씩 천천히 이동\n"
            "      %s 45 30 --step=0.5 --interval=200   ID1=45도, ID2=30도 동시 이동, 0.5도씩 200ms 간격\n",
            argv[0], NUM_MOTORS, argv[0], argv[0]);
        return 1;
    }

    float targets[NUM_MOTORS];
    int   n_targets = 0;
    float step_deg = DEFAULT_STEP_DEG;
    long  step_interval_ms = DEFAULT_STEP_MS;

    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], "--step=", 7) == 0) {
            step_deg = fabsf((float)atof(argv[i] + 7));
        } else if (strncmp(argv[i], "--interval=", 11) == 0) {
            step_interval_ms = atol(argv[i] + 11);
        } else if (n_targets < NUM_MOTORS) {
            targets[n_targets++] = (float)atof(argv[i]);
        }
    }
    if (n_targets == 0) {
        fprintf(stderr, "목표각도를 1개 이상 입력하세요.\n");
        return 1;
    }
    if (step_deg <= 0.0f) step_deg = DEFAULT_STEP_DEG;
    if (step_interval_ms <= 0) step_interval_ms = DEFAULT_STEP_MS;

    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    if (ak45_init() < 0) {
        fprintf(stderr, "초기화 실패. can0 인터페이스 확인:\n"
                "  sudo ip link set can0 up type can bitrate 1000000\n");
        return 1;
    }

    printf("[demo] 시연 모드: %.2f도씩, %ldms 간격으로 목표까지 이동합니다 (Ctrl+C로 중단)\n",
           step_deg, step_interval_ms);
    printf("[demo] 현재 위치(피드백) 수신 대기 중...\n");

    RampState ramp[NUM_MOTORS];
    memset(ramp, 0, sizeof(ramp));

    struct timespec wait_start;
    clock_gettime(CLOCK_MONOTONIC, &wait_start);
    for (int i = 0; i < n_targets; i++) {
        uint8_t id = MOTOR_IDS[i];
        MotorState s;
        do {
            s = ak45_get_state(id);
            if (s.valid) break;
            usleep(20000);
        } while (ms_since(&wait_start) < 3000 && !g_quit);

        ramp[i].active           = 1;
        ramp[i].final_deg        = targets[i];
        ramp[i].step_deg         = step_deg;
        ramp[i].current_deg      = s.valid ? s.position_deg : 0.0f;  // 피드백 없으면 0도 기준
        ramp[i].step_interval_ms = step_interval_ms;
        clock_gettime(CLOCK_MONOTONIC, &ramp[i].last_step_time);

        printf("[demo] ID%d: %.2f도 -> %.2f도\n", i + 1, ramp[i].current_deg, ramp[i].final_deg);
    }

    while (!g_quit) {
        for (int i = 0; i < n_targets; i++) {
            uint8_t id = MOTOR_IDS[i];

            if (!ak45_is_watchdog_ok(id)) {
                fprintf(stderr, "\n[워치독] ID=0x%02X 피드백 %dms 초과 - 긴급 정지\n", id, WATCHDOG_TIMEOUT_MS);
                ak45_emergency_stop_one(id);
                ramp[i].active  = 0;
                ramp[i].stopped = 1;   // 워치독 실패 모터는 더 이상 위치 명령 재송신하지 않음
                continue;
            }
            if (ramp[i].stopped) continue;

            if (ramp[i].active) {
                // 논블로킹 스텝 타이머: 여기서 sleep하지 않고 경과 시간만 확인 -
                // 그 사이 SEND_INTERVAL_MS 주기의 재전송/피드백 출력은 계속 진행된다.
                if (ms_since(&ramp[i].last_step_time) >= ramp[i].step_interval_ms) {
                    float diff = ramp[i].final_deg - ramp[i].current_deg;
                    if (fabsf(diff) <= ramp[i].step_deg) {
                        ramp[i].current_deg = ramp[i].final_deg;
                        ramp[i].active = 0;
                        printf("\n[demo] ID%d 목표(%.2f도) 도달 - 위치 유지 신호 계속 송신 (Ctrl+C로 종료)\n",
                               i + 1, ramp[i].final_deg);
                    } else {
                        ramp[i].current_deg += (diff > 0 ? ramp[i].step_deg : -ramp[i].step_deg);
                    }
                    clock_gettime(CLOCK_MONOTONIC, &ramp[i].last_step_time);
                }
            }

            // 목표 도달 후에도 계속 재송신 -- 재송신을 멈추면 모터 쪽 명령 타임아웃으로
            // 홀딩 토크가 풀릴 수 있으므로(AGENTS.md §5 규칙4), Ctrl+C 전까지 계속 보낸다.
            ak45_set_position(id, ramp[i].current_deg);
        }

        char line[512];
        int  off = 0;
        for (int i = 0; i < n_targets; i++) {
            uint8_t id = MOTOR_IDS[i];
            MotorState s = ak45_get_state(id);
            if (i > 0) off += snprintf(line + off, sizeof(line) - off, " | ");
            if (s.valid) {
                off += snprintf(line + off, sizeof(line) - off,
                        "0x%02X %-8.2f (목표 %.2f)", id, s.position_deg, ramp[i].final_deg);
            } else {
                off += snprintf(line + off, sizeof(line) - off, "0x%02X (피드백 대기중)", id);
            }
        }
        printf("\r%-100s", line);
        fflush(stdout);

        usleep(SEND_INTERVAL_MS * 1000); // 100ms tick (블로킹 램프 대기가 아닌 CAN 재송신 주기)
    }

    printf("\n[demo] Ctrl+C 감지, 종료 중...\n");
    ak45_close();
    return 0;
}
