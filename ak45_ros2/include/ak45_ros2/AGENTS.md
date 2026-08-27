<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-08-20 | Updated: 2026-08-20 -->

# ak45_ros2 (headers)

## Purpose
복사해 온 제어 라이브러리의 공개 헤더 하나만 있다. **수정 금지 구역이다.**

## Key Files

| File | Description |
|------|-------------|
| `ak45_36_socketcan_control.h` | **원본 복사본 — 한 글자도 수정 금지.** API 선언, 프로토콜 상수, `MotorState` 구조체, `CanPacketId`/`MotorError` 열거형 |

## For AI Agents

### Phase 1에서 쓰는 API는 5개뿐
```c
int  ak45_init(void);                             // 0=성공, -1=실패
void ak45_close(void);                            // 내부에서 ak45_emergency_stop() 먼저 호출
MotorState ak45_get_state(uint8_t controller_id); // 값 복사 반환, 논블로킹
int  ak45_is_watchdog_ok(uint8_t controller_id);  // 1=정상, 0=타임아웃/미수신
const char *ak45_error_str(uint8_t code);         // 한국어 문자열 반환
```

나머지 함수(`ak45_set_position` 등)는 **모터를 움직이므로 Phase 1에서 호출 금지**다.

`ak45_get_state()`가 **값 복사를 반환**하는 성질이 노드 설계의 근거다 — 내부 mutex만 잡고 즉시 반환하므로 타이머 콜백에서 폴링해도 executor가 막히지 않고, 노드가 잠금을 만들 필요가 없다.

### 이 헤더에 있는 상수 (노드가 파라미터로 만들지 않는 이유)
| 상수 | 값 | 노드 쪽 처리 |
|---|---|---|
| `CAN_INTERFACE` | `"can0"` (15행) | **컴파일 타임 상수** → `can_interface` 파라미터를 만들지 않는다 |
| `NUM_MOTORS` | `6` (14행) | 노드는 자체 `kNumMotors = 6`을 쓴다 |
| `CONTROLLER_ID_1~6` | `0x01`~`0x06` (8~13행) | `motor_ids` 파라미터를 만들지 않는다(변경 금지) |
| `WATCHDOG_TIMEOUT_MS` | `200` (32행) | `watchdog_timeout_ms` 파라미터를 만들지 않는다 |
| `SOFT_LIMIT_CURRENT_A` | `5.0f` (24행) | Phase 1 무관(명령 없음) |
| `SOFT_LIMIT_ERPM` | `500` (27행, 시연용 임시값) | Phase 1 무관. **Phase 2 착수 전 확정 필요** |
| `SOFT_LIMIT_POS_DEG` | `360.0f` (29행) | Phase 1 무관 |

### 워치독 경계값
판정식이 `elapsed < WATCHDOG_TIMEOUT_MS`이므로 **정확히 200ms는 실패로 친다.** 또한 `ak45_is_watchdog_ok()`는 `valid == 0`일 때도 `0`을 반환하므로 진단 판정에서 `valid`를 먼저 봐야 한다.

### 동일성 확인
```bash
diff /home/jaejun/바탕화면/Can/ak45_36_socketcan_control.h \
     ~/ros2_ws/src/ak45_ros2/include/ak45_ros2/ak45_36_socketcan_control.h
```

## Dependencies

### External
- `linux/can.h`, `linux/can/raw.h` — SocketCAN 커널 헤더
- `pthread.h`, `time.h`

<!-- MANUAL: -->
