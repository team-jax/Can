#include "ak45_36_socketcan_control.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>

static volatile int g_quit = 0;

static const uint8_t MOTOR_IDS[NUM_MOTORS] = {
    CONTROLLER_ID_1, CONTROLLER_ID_2, CONTROLLER_ID_3,
    CONTROLLER_ID_4, CONTROLLER_ID_5, CONTROLLER_ID_6
};

static pthread_mutex_t g_target_mutex = PTHREAD_MUTEX_INITIALIZER;
static float g_target_deg[NUM_MOTORS] = { 0 };
static int   g_has_target[NUM_MOTORS] = { 0 };

static void sig_handler(int sig) { (void)sig; g_quit = 1; }

// 공백으로 구분된 각도값을 최대 max_count개까지 파싱
static int parse_floats(const char *str, float *out, int max_count)
{
    int count = 0;
    const char *p = str;
    while (count < max_count) {
        char *end;
        float v = strtof(p, &end);
        if (end == p) break;
        out[count++] = v;
        p = end;
    }
    return count;
}

// 실행 중 터미널 입력으로 목표각도를 갱신하는 스레드
// 입력 형식: "30"              → ID1(1번 모터)만 30도로 이동
//            "30 20 10" / ">30 20 10" → ID1=30도, ID2=20도, ID3=10도 동시 이동 (최대 NUM_MOTORS개)
static void *input_thread(void *arg)
{
    (void)arg;
    char line[256];

    while (!g_quit) {
        printf("\n> 목표각도 입력 (예: 30 / 30 20 10 (최대 %d개), q=종료): ", NUM_MOTORS);
        fflush(stdout);

        if (!fgets(line, sizeof(line), stdin)) break;

        // 개행 제거 후 종료 명령 확인
        line[strcspn(line, "\r\n")] = '\0';
        if (strcmp(line, "q") == 0 || strcmp(line, "quit") == 0) {
            g_quit = 1;
            break;
        }
        if (line[0] == '\0') continue;

        const char *p = line;
        if (*p == '>') p++;   // '>' 접두사는 기존 표기와의 호환을 위해 허용

        float vals[NUM_MOTORS];
        int   n = parse_floats(p, vals, NUM_MOTORS);
        if (n == 0) {
            printf("숫자를 입력하세요 (예: 30, -45), 여러 모터는 공백으로 구분 (최대 %d개), q로 종료\n",
                   NUM_MOTORS);
            continue;
        }

        pthread_mutex_lock(&g_target_mutex);
        for (int i = 0; i < n; i++) {
            g_target_deg[i] = vals[i];
            g_has_target[i] = 1;
        }
        pthread_mutex_unlock(&g_target_mutex);

        printf("목표 위치 갱신:");
        for (int i = 0; i < n; i++) {
            printf(" ID%d=%.2f도", i + 1, vals[i]);
        }
        printf(" (소프트 리밋 ±%.1f도로 클램핑됨)\n", SOFT_LIMIT_POS_DEG);
    }
    return NULL;
}

int main(int argc, char *argv[])
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    for (int i = 0; i < NUM_MOTORS && i + 1 < argc; i++) {
        g_target_deg[i] = (float)atof(argv[i + 1]);
        g_has_target[i] = 1;
    }

    if (ak45_init() < 0) {
        fprintf(stderr, "초기화 실패. can0 인터페이스 확인:\n"
                "  sudo ip link set can0 up type can bitrate 1000000\n");
        return 1;
    }

    pthread_t input_tid;
    pthread_create(&input_tid, NULL, input_thread, NULL);
    pthread_detach(input_tid);

    printf("피드백 모니터링 시작 (Ctrl+C 또는 q 입력으로 종료)\n");
    int any_target = 0;
    for (int i = 0; i < NUM_MOTORS; i++) any_target |= g_has_target[i];
    if (any_target) {
        printf("초기 목표 위치:");
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (g_has_target[i]) printf(" ID%d=%.2f도", i + 1, g_target_deg[i]);
        }
        printf(" (소프트 리밋 ±%.1f도로 클램핑됨)\n", SOFT_LIMIT_POS_DEG);
    }
    printf("실행 중 언제든 각도를 입력하면 목표 위치가 즉시 갱신됩니다.\n");
    printf("%-4s %-10s %-10s %-10s %-8s %s\n",
           "ID", "Pos(deg)", "Spd(ERPM)", "Cur(A)", "Temp(C)", "Error");

    // 첫 피드백 수신 대기
    sleep(1);

    while (!g_quit) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            uint8_t id = MOTOR_IDS[i];

            if (!ak45_is_watchdog_ok(id)) {
                fprintf(stderr, "[워치독] ID=0x%02X 피드백 %dms 초과 — 긴급 정지\n",
                        id, WATCHDOG_TIMEOUT_MS);
                ak45_emergency_stop_one(id);
                continue;
            }

            pthread_mutex_lock(&g_target_mutex);
            int   has_target = g_has_target[i];
            float target_deg = g_target_deg[i];
            pthread_mutex_unlock(&g_target_mutex);

            if (has_target) {
                // 모터 쪽 명령 타임아웃보다 짧은 주기로 재송신 (AGENTS.md §5 규칙4)
                ak45_set_position(id, target_deg);
            }
        }

        char line[512];
        int  off = 0;
        for (int i = 0; i < NUM_MOTORS; i++) {
            uint8_t id = MOTOR_IDS[i];
            MotorState s = ak45_get_state(id);
            if (i > 0) off += snprintf(line + off, sizeof(line) - off, " | ");
            if (s.valid) {
                off += snprintf(line + off, sizeof(line) - off,
                        "0x%02X %-8.2f %-9.0f %-7.2f %-5d %s",
                        id, s.position_deg, s.speed_erpm, s.current_a,
                        s.temperature_c, ak45_error_str(s.error_code));
            } else {
                off += snprintf(line + off, sizeof(line) - off, "0x%02X (피드백 대기중)", id);
            }
        }
        printf("\r%-100s", line);
        fflush(stdout);

        usleep(100000); // 100ms
    }

    printf("\n종료 중...\n");
    ak45_close();
    return 0;
}
