#include "ak45_36_socketcan_control.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>

static volatile int g_quit = 0;

static pthread_mutex_t g_target_mutex = PTHREAD_MUTEX_INITIALIZER;
static float g_target_deg = 0.0f;
static int   g_has_target = 0;

static void sig_handler(int sig) { (void)sig; g_quit = 1; }

// 실행 중 터미널 입력으로 목표각도를 갱신하는 스레드
static void *input_thread(void *arg)
{
    (void)arg;
    char line[128];

    while (!g_quit) {
        printf("\n> 목표각도 입력(도, q=종료): ");
        fflush(stdout);

        if (!fgets(line, sizeof(line), stdin)) break;

        // 개행 제거 후 종료 명령 확인
        line[strcspn(line, "\r\n")] = '\0';
        if (strcmp(line, "q") == 0 || strcmp(line, "quit") == 0) {
            g_quit = 1;
            break;
        }
        if (line[0] == '\0') continue;

        char *end;
        float deg = strtof(line, &end);
        if (end == line) {
            printf("숫자를 입력하세요 (예: 30, -45) 또는 q로 종료\n");
            continue;
        }

        pthread_mutex_lock(&g_target_mutex);
        g_target_deg = deg;
        g_has_target = 1;
        pthread_mutex_unlock(&g_target_mutex);

        printf("목표 위치 갱신: %.2f도 (소프트 리밋 ±%.1f도로 클램핑됨)\n",
               deg, SOFT_LIMIT_POS_DEG);
    }
    return NULL;
}

int main(int argc, char *argv[])
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    if (argc > 1) {
        g_target_deg = (float)atof(argv[1]);
        g_has_target = 1;
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
    if (g_has_target) {
        printf("초기 목표 위치: %.2f도 (소프트 리밋 ±%.1f도로 클램핑됨)\n",
               g_target_deg, SOFT_LIMIT_POS_DEG);
    }
    printf("실행 중 언제든 각도를 입력하면 목표 위치가 즉시 갱신됩니다.\n");
    printf("%-10s %-10s %-10s %-8s %s\n",
           "Pos(deg)", "Spd(ERPM)", "Cur(A)", "Temp(C)", "Error");

    // 첫 피드백 수신 대기
    sleep(1);

    while (!g_quit) {
        if (!ak45_is_watchdog_ok()) {
            fprintf(stderr, "[워치독] 피드백 %dms 초과 — 긴급 정지\n",
                    WATCHDOG_TIMEOUT_MS);
            ak45_emergency_stop();
        } else {
            pthread_mutex_lock(&g_target_mutex);
            int   has_target = g_has_target;
            float target_deg = g_target_deg;
            pthread_mutex_unlock(&g_target_mutex);

            if (has_target) {
                // 모터 쪽 명령 타임아웃보다 짧은 주기로 재송신 (AGENTS.md §5 규칙4)
                ak45_set_position(target_deg);
            }
        }

        MotorState s = ak45_get_state();
        if (s.valid) {
            printf("\r%-10.2f %-10.0f %-10.2f %-8d %s    ",
                   s.position_deg, s.speed_erpm, s.current_a,
                   s.temperature_c, ak45_error_str(s.error_code));
            fflush(stdout);
        }

        usleep(100000); // 100ms
    }

    printf("\n종료 중...\n");
    ak45_close();
    return 0;
}
