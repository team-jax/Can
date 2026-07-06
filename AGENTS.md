<!-- Generated: 2026-07-05 | Updated: 2026-07-05 -->

# can_ak45

## Purpose
CubeMars AK45-36 KV80 액추에이터 **2대(ID1, ID2)** 를 Linux + CANable(SocketCAN) 환경에서 Servo 모드 CAN으로 동시 제어하는 C++ 프로젝트. MIT 모드는 사용하지 않는다. 프로토콜 근거는 CubeMars AK Series Module Driver Manual V1.0.18(2026.01.19).

## Key Files

| File | Description |
|------|-------------|
| `ak45_36_socketcan_control.h` | API 선언, 프로토콜 상수(CONTROLLER_ID, 소프트 리밋), MotorState 구조체, CanPacketId/MotorError 열거형 |
| `ak45_36_socketcan_control.cpp` | SocketCAN 초기화, 수신 스레드, 피드백 파싱(0x29), 명령 함수(클램핑·에러 차단 포함), 워치독 |
| `main.cpp` | 피드백 모니터링 루프 예제. 실제 명령은 주석 처리된 예시 참조 |
| `Makefile` | `make` / `make clean`. 타겟 바이너리: `ak45_ctrl` |

프로토콜 상세(매뉴얼 대조 완료), 미확정 사항 체크리스트, 안전 규칙은 별도 CLAUDE.md가 아니라 이 문서 하단의 "AK45-36 KV80 CAN Servo 모드 제어 프로젝트 문서" 섹션(§1~§8)에 통합되어 있다.

## For AI Agents

### CAN 프레임 구조 (변경 금지)
- **명령 CAN ID** = `(CanPacketId << 8) | controller_id` — Extended Frame (29bit)
- **피드백 CAN ID** = `(0x29 << 8) | controller_id` — ID1=`0x2901`, ID2=`0x2902`
- 바이트 순서: big-endian (MSB first)
- 두 모터는 동일한 CAN 버스(`can0`)를 공유하며 `controller_id`(CONTROLLER_ID_1=0x01 / CONTROLLER_ID_2=0x02)로만 구분된다. 소켓/스레드는 1개, 상태는 모터별로 분리 관리.

### 다중 모터(ID1/ID2) 제어
- 모든 명령 함수(`ak45_set_position` 등)는 첫 인자로 `uint8_t controller_id`(`CONTROLLER_ID_1` 또는 `CONTROLLER_ID_2`)를 받는다.
- `MotorState`는 `g_state[NUM_MOTORS]` 배열로 모터별 관리. `motor_index(controller_id)`로 배열 인덱스를 얻는다. 등록되지 않은 ID는 -1을 반환하고 모든 API가 안전하게 실패(-1) 처리.
- `rx_thread`는 수신 프레임의 Function ID가 `0x29`이면 하위 바이트(controller_id)로 어느 모터의 피드백인지 판별해 해당 슬롯에 저장 — ID1/ID2 각각 별도 워치독(`ak45_is_watchdog_ok(id)`)과 에러 상태를 가진다.
- `ak45_emergency_stop()`은 ID1·ID2 모두 정지, `ak45_emergency_stop_one(id)`은 지정 모터만 정지(워치독 타임아웃 시 해당 모터만 멈추는 데 사용).
- `main.cpp` 터미널 입력 형식:
  - `30` → ID1(1번 모터)만 30도로 이동 (기존 단일 모터 방식과 호환)
  - `>30 20` → ID1=30도, ID2=20도 **동시** 이동
  - 실행 인자도 동일하게 확장: `./ak45_ctrl 30 20` → 초기 목표 ID1=30도, ID2=20도
- 새로운 모터(ID3 이상)를 추가하려면: `CONTROLLER_ID_3` 정의 → `NUM_MOTORS` 증가 → `motor_index()`에 분기 추가 → `main.cpp`의 `MOTOR_IDS[]`/타겟 배열 크기 확장.

### 소프트 리밋 상수 위치
헤더 파일 `ak45_36_socketcan_control.h` 상단:
- `SOFT_LIMIT_CURRENT_A 5.0f` — 전류 한계 미확정, CubeMarsTool 확인 후 갱신
- `SOFT_LIMIT_ERPM 10000` — NPP 미확정, 공식: `출력축 6 rad/s × NPP × 36` 으로 갱신
- `SOFT_LIMIT_POS_DEG 360.0f` — 운영 범위, 필요 시 조정 가능
- 두 모터 모두 동일한 소프트 리밋 상수를 공유한다(모터별 개별 리밋 아님).

### Working In This Directory
- 소프트 리밋을 완화하기 전 반드시 본 문서 §7 미확정 사항(NPP, 전류 한계) 확인
- 새 명령 함수 추가 시 `controller_id` 인자를 받고, 에러 코드 차단(`get_error_code(controller_id) != ERR_NONE` 체크) 및 클램핑을 반드시 포함
- 피드백 파싱은 `parse_feedback(controller_id, ...)` 단일 함수에서만 처리 — 스케일 혼동 방지(본 문서 §2.4)
- 모드 6(Position-Velocity)의 속도 필드는 ERPM÷10 스케일 — 모드 3(RPM)의 ERPM 그대로와 혼동 주의

### Testing Requirements
```bash
sudo ip link set can0 up type can bitrate 1000000
make
./ak45_ctrl                  # 피드백 모니터링 (ID1/ID2 둘 다)
./ak45_ctrl 30 20             # 초기 목표: ID1=30도, ID2=20도
candump can0                 # 송수신 프레임 덤프 확인 (0x2901=ID1 피드백, 0x2902=ID2 피드백)
```
실행 중 `>30 20` 입력 시 ID1=30도, ID2=20도로 동시 이동. 숫자만 입력하면 ID1만 이동(기존 방식).

### Common Patterns
- 명령 함수 패턴: `controller_id` 유효성(`motor_index() >= 0`) 확인 → 에러 체크 → 클램핑 → `buffer_append_int32` → `can_transmit_eid`
- 상태 접근: 항상 `g_state_mutex` 락 후 `g_state[motor_index(controller_id)]` 읽기/쓰기

## Dependencies

### Internal
- `ak45_36_socketcan_control.h` ↔ `ak45_36_socketcan_control.cpp` — 단일 모듈 쌍
- `main.cpp` → `ak45_36_socketcan_control.h` 만 포함

### External
- `linux/can.h`, `linux/can/raw.h` — SocketCAN 커널 헤더
- `libpthread` — 수신 스레드
- `libm` — 수학 함수

<!-- MANUAL: 미확정 사항 확인 후 소프트 리밋 갱신 내역을 아래에 기록 -->
