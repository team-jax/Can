<!-- Generated: 2026-07-05 | Updated: 2026-08-27 -->

# can_ak45

## Purpose
CubeMars AK45-36 KV80 액추에이터 **6대(CAN ID 0x01~0x06)** 를 Linux + CANable(SocketCAN) 환경에서 **Servo 모드 CAN**으로 제어하는 C++ 프로젝트. MIT 모드는 사용하지 않는다. 프로토콜 근거는 CubeMars AK Series Module Driver Manual V1.0.18(2026.01.19)이며 원문 대조 결과는 이 문서 하단 §1~§8에 있다.

⚠️ **문서 전반이 "6대"를 전제하지만 2026-08-27 현재 버스에는 모터가 1대(ID 0x02)뿐이다.** 코드는 6대 기준 그대로 두고 운용만 1대로 하는 상태다. 아래 「현재 버스 실상」 참조.

이 저장소는 **Phase 0(C++ 제어 라이브러리 + CLI)** 의 코드를 담는다. Phase 1(ROS2 읽기 전용 상태 발행 노드)의 사양서 `ros2.md`는 여기 있으나 **구현 패키지는 저장소 밖(`~/ros2_ws/src/ak45_ros2`)에 있고 어느 저장소에 둘지 미정**이다(`STATUS.md` P11). 프로젝트 전체 진행 상황·미결 항목은 `STATUS.md`를 본다.

## Key Files

| File | Description |
|------|-------------|
| `ak45_36_socketcan_control.h` | API 선언, 프로토콜 상수(`CONTROLLER_ID_1~6`, `NUM_MOTORS 6`, `CAN_INTERFACE "can0"`, `WATCHDOG_TIMEOUT_MS 200`, 소프트 리밋), `MotorState` 구조체, `CanPacketId`/`MotorError` 열거형 |
| `ak45_36_socketcan_control.cpp` | SocketCAN 초기화, 수신 스레드, 피드백 파싱(0x29), 명령 함수(클램핑·에러 차단 포함), 모터별 워치독, 프로세스 단일 실행 잠금(`flock`, `/tmp/ak45_ctrl.lock`) |
| `main.cpp` | 피드백 모니터링 루프 + 터미널 입력 스레드(`input_thread`)로 실시간 목표각도 갱신, `ak45_set_position` 실제 호출. 100ms 주기 명령 재송신 루프가 여기 있다(라이브러리에는 없다) |
| `demo_ramp.cpp` | 시연용 완속 이동 프로그램. `--step`(기본 1도)씩 `--interval`(기본 150ms) 간격으로 계단식 이동. `sleep`으로 블로킹하지 않고 `clock_gettime` 기반 논블로킹 타이머로 스텝 전환 시각만 확인하며, 그 사이에도 100ms 주기 CAN 재송신·워치독 체크·피드백 출력은 계속 진행 |
| `Makefile` | `make` / `make clean`. **`-std=c++11`, `-Wall -Wextra -O2`, `LDFLAGS = -lpthread -lm`.** 타겟 바이너리: `ak45_ctrl`, `ak45_ctrl_demo` |
| `README.md` | **진입점 문서** — 프로젝트 소개, 빌드·실행, CLI 사용법, Phase 상태, 문서 지도. 프로토콜은 요약만 싣고 전문은 이 문서 §1~§8을 참조한다(중복 금지 — 2026-08-20에 낡은 복사본을 걷어냈다) |
| `run.txt` | 터미널 2개용 **빠른 실행 요약**(2026-08-27 재작성). 이전의 45°/90° 실행 로그가 아니다. `RUN.md`가 이를 대체하며, run.txt의 위치·온도 값은 이미 낡았다(7~10행, 86행) |
| `RUN.md` | **실행 런북(정본).** 0~2단계 준비, 시나리오 A(ROS2 읽기 전용)/B(CLI 이동), 실측 결과·발견 1~3, 재실측(2026-08-27 18:58), 트러블슈팅. 실행 방법을 찾을 때 **여기부터 본다.** Git 미추적 |
| `ros2.md` | **Phase 1 ROS2 상태 발행 노드 구현 명세서 v3.0.** 노드 이름·토픽·QoS·로그 문구·검증 항목 V0~V15가 전부 확정값으로 적혀 있다. **Git 미추적**(`STATUS.md` 할 일) |
| `STATUS.md` | 프로젝트 전체 상태 — 했던 일 / 하는 일 / 할 일 / 목표 / 사용 기술 / 문제점 P1~P12. **Git 미추적** |
| `AGENTS.md` | 이 문서. 상단은 자동 생성 구역, `<!-- MANUAL -->` 아래 §1~§8은 수작성 프로토콜 레퍼런스 |

**빌드 산출물이 저장소에 커밋되어 있다**: `ak45_36_socketcan_control.o`, `main.o`, `demo_ramp.o`, `ak45_ctrl`, `ak45_ctrl_demo`, `ak45_demo`. `.gitignore`가 **없다.**

## Subdirectories

| Directory | Purpose |
|-----------|---------|
| `ak45_ros2/` | **ROS2 Humble 패키지(Phase 1+2).** 노드 하나 `ak45_node` — 상태 발행 + 명령 수신. `~/ros2_ws/src/ak45_ros2`가 여기를 가리키는 **심볼릭 링크**다. 자체 AGENTS.md 6개를 가진다 (`ak45_ros2/AGENTS.md` 참조) |
| `.claude/` | Claude Code 프로젝트 설정(`settings.local.json` — Bash 권한 허용 목록). 소스가 아니므로 AGENTS.md를 두지 않는다 |
| `.omc/` | oh-my-claudecode 세션·상태 캐시(`project-memory.json`, `sessions/`, `state/`, `specs/`). 도구 생성물이며 **11개 파일이 Git에 커밋되어 있다**(`specs/`와 `state/deep-dive-state.json`은 미추적). AGENTS.md 생성 대상이 아니다 |

~~소스 디렉토리는 루트 하나뿐인 플랫 구조다.~~ → **2026-08-27부터 `ak45_ros2/`가 저장소 안으로 들어와 2계층이 됐다.**
루트는 Phase 0(C++ 라이브러리 + CLI), `ak45_ros2/`는 Phase 1+2(ROS2 패키지)를 담는다.

⚠️ **`ak45_36_socketcan_control.{h,cpp}`가 저장소 안에 두 벌 있다** — 루트 원본과 `ak45_ros2/` 복사본.
내용은 동일해야 하며(2026-08-27 `diff` 확인) **루트에서 고치고 복사본을 맞추는** 방향이다.
루트는 `-std=c++11`, 패키지는 **C++17**로 같은 파일을 컴파일하므로 양쪽에서 컴파일되는 코드만 쓴다.

## For AI Agents

### ⚠️ 현재 버스 실상 — 모터가 1대다 (2026-08-27 실측)

코드·상수·이 문서는 전부 **6대(ID 0x01~0x06)** 를 전제하지만, **실제 버스에 응답하는 모터는 ID 0x02 하나뿐이다.**

| 항목 | 실측값 |
|---|---|
| 응답 모터 | **ID 0x02 단 1대.** 0x01·0x03~0x06은 피드백 0건 |
| 피드백 주기 | 50Hz (30초/1500프레임) |
| 위치 | **+29.9도** (`00002902` 앞 2바이트 `01 2B`) |
| 온도 | **53°C** — 상승 멈춤. 전류 0A인데 26→53°C까지 오른 원인은 **미상** |
| 전류 / 에러 | 0.00A / 0 |

이것이 코드에 미치는 영향:

- `/joint_states`에 관절이 **1개만** 담기고 `/diagnostics`는 5개가 WARN으로 뜬다 — **정상 동작이다.** 결함으로 오진하지 말 것.
- `./ak45_ctrl`은 인자가 없어도 **조용하지 않다.** 워치독 분기가 피드백 없는 모터 5대에 100ms마다 Current Brake 0A를 보낸다(실측 5초/203프레임, stderr 초당 50줄). 0A라 모터는 안 움직인다. 순수 읽기 전용이 필요하면 ROS2 노드를 쓴다(송신 실측 0프레임).
- `ak45_ctrl` 입력은 **ID 순서 위치 매칭**이다. ID2를 움직이려면 값을 2개 줘야 한다 — `0 10`. `10`만 치면 없는 ID1로 간다.
- 원위치 복귀값은 **`0 29.9`** 다. 문서 곳곳의 `0 -9.8`은 앞 세션 값이라 **더 이상 원위치가 아니다.**

세션 초반 candump에 `00002900`(controller_id **0x00**)이 잠깐 보였다. `motor_index()`가 0x01~0x06만 등록하므로 **ID 0x00의 피드백은 조용히 폐기된다.** 모터 한 대가 공장 기본값 0에 머물러 있을 가능성이 있다.

### CAN 프레임 구조 (변경 금지)
- **명령 CAN ID** = `(CanPacketId << 8) | controller_id` — Extended Frame (29bit)
- **피드백 CAN ID** = `(0x29 << 8) | controller_id` — ID1=`0x2901` ... ID6=`0x2906`
- 바이트 순서: big-endian (MSB first)
- 모든 모터는 동일한 CAN 버스(`can0`)를 공유하며 `controller_id`(CONTROLLER_ID_1=0x01 ~ CONTROLLER_ID_6=0x06)로만 구분된다. 소켓/스레드는 1개, 상태는 모터별로 분리 관리.

### 다중 모터(ID1~ID6) 제어
- 모든 명령 함수(`ak45_set_position` 등)는 첫 인자로 `uint8_t controller_id`(`CONTROLLER_ID_1`~`CONTROLLER_ID_6`)를 받는다.
- `MotorState`는 `g_state[NUM_MOTORS]` 배열로 모터별 관리. `motor_index(controller_id)`는 `g_controller_ids[NUM_MOTORS]` 테이블을 순회해 배열 인덱스를 얻는다. 등록되지 않은 ID는 -1을 반환하고 모든 API가 안전하게 실패(-1) 처리.
- `rx_thread`는 수신 프레임의 Function ID가 `0x29`이면 하위 바이트(controller_id)로 어느 모터의 피드백인지 판별해 해당 슬롯에 저장 — ID1~ID6 각각 별도 워치독(`ak45_is_watchdog_ok(id)`)과 에러 상태를 가진다.
- `ak45_emergency_stop()`은 등록된 모든 모터(ID1~ID6)를 정지, `ak45_emergency_stop_one(id)`은 지정 모터만 정지(워치독 타임아웃 시 해당 모터만 멈추는 데 사용).
- `main.cpp` 터미널 입력 형식(공백으로 구분, 최대 `NUM_MOTORS`개, `parse_floats()`로 파싱):
  - `30` → ID1(1번 모터)만 30도로 이동 (기존 단일 모터 방식과 호환)
  - `30 20 10` / `>30 20 10` → ID1=30도, ID2=20도, ID3=10도 **동시** 이동 (입력한 개수만큼만 갱신)
  - 실행 인자도 동일하게 확장: `./ak45_ctrl 30 20 10 5 0 -10` → 초기 목표 ID1~ID6 순서대로 반영
- 모터를 7대 이상으로 늘리려면: `CONTROLLER_ID_7` 등 정의 → `NUM_MOTORS` 증가 → `g_controller_ids[]`(cpp)와 `MOTOR_IDS[]`(main.cpp) 배열에 추가만 하면 나머지 로직은 `NUM_MOTORS` 기반으로 자동 확장된다.

### 소프트 리밋 상수 위치
헤더 파일 `ak45_36_socketcan_control.h` 상단:
- `SOFT_LIMIT_CURRENT_A 5.0f` (24행) — 전류 한계 미확정, CubeMarsTool 확인 후 갱신
- `SOFT_LIMIT_ERPM 500` (27행) — **현재값은 시연용으로 낮춘 임시값**(초본은 10000). NPP 미확정이라 정식값을 계산할 수 없다. 공식: `출력축 6 rad/s × NPP × 36`. NPP 확정 후 갱신하며, **명령을 보내는 Phase 2 착수 전에 반드시 확정할 것**
- `SOFT_LIMIT_POS_DEG 360.0f` (29행) — 운영 범위, 필요 시 조정 가능
- 6대 모두 동일한 소프트 리밋 상수를 공유한다(모터별 개별 리밋 아님).
- 상수를 바꾸면 **헤더 값 → 같은 줄 주석 → 이 문서 §4·§7 → `ros2.md` 부록 B.1·부록 C → `STATUS.md`** 를 같은 커밋에서 고친다(`ros2.md` 부록 E).

### Working In This Directory
- 소프트 리밋을 완화하기 전 반드시 본 문서 §7 미확정 사항(NPP, 전류 한계) 확인
- 새 명령 함수 추가 시 `controller_id` 인자를 받고, 에러 코드 차단(`get_error_code(controller_id) != ERR_NONE` 체크) 및 클램핑을 반드시 포함
- 피드백 파싱은 `parse_feedback(controller_id, ...)` 단일 함수에서만 처리 — 스케일 혼동 방지(본 문서 §2.4)
- 모드 6(Position-Velocity)의 속도 필드는 ERPM÷10 스케일 — 모드 3(RPM)의 ERPM 그대로와 혼동 주의
- `ak45_ctrl`은 `/tmp/ak45_ctrl.lock`을 `flock`으로 잠그므로 동시에 두 번째 인스턴스를 실행하면 `ak45_init()`이 즉시 실패한다(같은 CAN 버스에 대한 명령 충돌 방지). 테스트 중 "초기화 실패" 원인이 이것일 수 있음 — **ROS2 노드도 같은 락을 쓰므로 노드와 CLI를 동시에 띄울 수 없다**
- **`.h`/`.cpp` 두 파일은 ROS2 패키지가 파일 복사로 편입했다**(`~/ros2_ws/src/ak45_ros2`, 원본 커밋 `893d663`). Phase 1 사양(`ros2.md` 작업 규칙 3)은 복사본을 한 글자도 수정하지 못하게 하므로, 고칠 일이 생기면 **여기서 고치고 복사본을 다시 맞춘 뒤 패키지 README의 커밋 해시를 갱신**한다.
- **컴파일러 표준이 두 개다**: Makefile은 `-std=c++11`, ROS2 패키지는 C++17로 **같은 `.cpp`를 컴파일**한다. 양쪽에서 모두 컴파일되는 코드만 쓸 것.
- **빌드 산출물이 Git에 추적된다.** `make`를 돌리면 `.o`와 바이너리가 변경되어 working tree가 더러워진다. 커밋 전 `git status`로 의도한 변경만 담겼는지 확인할 것.
- `ak45_demo`는 현재 Makefile이 만들지 않는 **구 바이너리**이고 `make clean` 대상도 아니다. 빌드해도 갱신되지 않으니 동작 근거로 쓰지 말 것.
- **커밋된 바이너리는 이 PC에서 실행되지 않을 수 있다 — 빌드는 반드시 `make clean`부터.** Git에 담긴 바이너리는 glibc **2.38** 환경에서 링크된 것이라 이 PC(**2.35**)에서 `version 'GLIBC_2.38' not found`로 죽는다. `make`만 돌리면 `.o` 타임스탬프 때문에 재링크를 건너뛰어 안 고쳐진다. (현재 바이너리는 재빌드되어 GLIBC_2.34 요구 = 정상 실행된다.)

### Testing Requirements
```bash
sudo ip link set can0 up type can bitrate 1000000
make
./ak45_ctrl                  # 피드백 모니터링 (ID1~ID6 전부)
./ak45_ctrl 30 20 10          # 초기 목표: ID1=30도, ID2=20도, ID3=10도 (입력 개수만큼만 반영)
candump can0                 # 송수신 프레임 덤프 확인 (0x2901=ID1 피드백 ... 0x2906=ID6 피드백)

./ak45_ctrl_demo 45                              # 시연용: ID1을 현재 위치→45도까지 1도씩 천천히 이동
./ak45_ctrl_demo 45 30 --step=0.5 --interval=200 # ID1=45도, ID2=30도 동시, 0.5도씩 200ms 간격
```
실행 중 `>30 20 10` 입력 시 ID1=30도, ID2=20도, ID3=10도로 동시 이동. 숫자만 입력하면 ID1만 이동(기존 방식).
`ak45_ctrl`과 `ak45_ctrl_demo`는 같은 `flock` 잠금(`/tmp/ak45_ctrl.lock`)을 공유하므로 동시에 두 개를 실행할 수 없다(같은 CAN 버스 명령 충돌 방지).

하드웨어가 없을 때: `sudo ip link add dev can0 type vcan && sudo ip link set up can0` 으로 가상 `can0`을 만들면 초기화·발행 경로까지는 검증된다(실제 피드백 값·감속비 판별은 실모터 필요). `candump`/`cansend`가 없는 환경도 있으니 `can-utils` 설치 여부를 먼저 확인할 것.

### Common Patterns
- 명령 함수 패턴: `controller_id` 유효성(`motor_index() >= 0`) 확인 → 에러 체크 → 클램핑 → `buffer_append_int32` → `can_transmit_eid`
- 상태 접근: 항상 `g_state_mutex` 락 후 `g_state[motor_index(controller_id)]` 읽기/쓰기
- `ak45_get_state()`는 락을 잡고 **값 복사를 반환**한다 — 호출자는 별도 잠금이 필요 없다(ROS2 노드가 이 성질에 의존한다)

## Dependencies

### Internal
- `ak45_36_socketcan_control.h` ↔ `ak45_36_socketcan_control.cpp` — 단일 모듈 쌍
- `main.cpp` → `ak45_36_socketcan_control.h` 만 포함
- `demo_ramp.cpp` → `ak45_36_socketcan_control.h` 만 포함
- 저장소 밖: `~/ros2_ws/src/ak45_ros2` 가 `.h`/`.cpp` 를 복사해 사용

### External
- `linux/can.h`, `linux/can/raw.h` — SocketCAN 커널 헤더
- `libpthread` — 수신 스레드
- `libm` — Makefile `LDFLAGS`에 `-lm`이 있으나 Ubuntu 22.04(glibc 2.35)는 **libm이 libc에 병합되어 있어 no-op이다.** 파일별로 보면: **라이브러리 `.cpp`는 `<math.h>`를 include만 하고 수학 함수를 호출하지 않는다**(`clampf`는 비교 연산뿐) — 그래서 ROS2 패키지는 `-lm` 없이 링크한다(`ros2.md` §1.3). 반면 **`demo_ramp.cpp`는 `fabsf()`를 호출**하므로, libm이 분리된 다른 환경으로 옮길 때 Makefile에서 `-lm`을 빼면 `ak45_ctrl_demo` 링크가 깨진다.

<!-- MANUAL: 미확정 사항 확인 후 소프트 리밋 갱신 내역을 아래에 기록 -->

---

# AK45-36 KV80 CAN Servo 모드 제어 프로젝트 문서

> 프로토콜 근거: **CubeMars AK Series Module Driver Manual V1.0.18** (2026.01.19, AK 2.0용)
> 이 문서의 프로토콜 값은 매뉴얼 원문과 대조 완료. ✅ = 매뉴얼로 검증됨, ⚠️ = 미확정(실측/CubeMarsTool 확인 필요)

---

## 1. 프로젝트 개요

CubeMars AK45-36 KV80 액추에이터를 Linux + CANable(SocketCAN) 환경에서 **Servo 모드 CAN**으로 제어한다. MIT 모드는 사용하지 않는다.

### 하드웨어 구성
| 항목 | 내용 |
|---|---|
| 모터 | AK45-36 KV80 (BLDC + 36:1 유성기어 + 드라이버 일체형) |
| CAN 인터페이스 | CANable (candlelight 펌웨어, SocketCAN `can0` 인식 확인됨) |
| OS | Linux |
| CAN 속도 | **1Mbps** ✅ (매뉴얼: "1Mbps, No change recommended") |
| 프레임 | Extended Frame (29bit ID) ✅ (Servo 모드는 확장 프레임만 응답 — 공식 FAQ) |

### 파일 구성
| 파일 | 역할 |
|---|---|
| `ak45_36_socketcan_control.h` | API 선언, 프로토콜 상수, MotorState 구조체 |
| `ak45_36_socketcan_control.cpp` | C++ + SocketCAN 구현체 (유일한 제어 구현) |
| `main.cpp` | 피드백 모니터링 + 터미널 목표각도 입력 CLI |
| `Makefile` | 빌드 (`make` / `make clean`) |

---

## 2. Servo 모드 CAN 프로토콜 (매뉴얼 5.1~5.2절 검증 완료)

### 2.1 명령 프레임 (호스트 → 모터) ✅

```
CAN ID (29bit) = (제어모드 << 8) | Controller_ID
프레임 타입: Extended, 데이터: big-endian
```

| 모드 | 이름 | 데이터 | 스케일 / 범위 (매뉴얼 원문) |
|---|---|---|---|
| 0 | Duty Cycle | int32, 4B | 값/100000 = 듀티. 기본 허용 0.005–0.95 |
| 1 | Current Loop (토크) | int32, 4B | 값/1000 = A. **−60000~60000 = −60~60A** |
| 2 | Current Brake | int32, 4B | 값/1000 = A. 0~60000 = 0~60A |
| 3 | Velocity | int32, 4B | **그대로 ERPM. −100000~100000** |
| 4 | Position | int32, 4B | 값/10000 = deg. **−360000000~360000000 = −36000°~36000°** |
| 5 | Set Origin | uint8, 1B | 0=임시 원점(전원 차단 시 소멸), 1=영구 영점(듀얼 엔코더 모델 전용) |
| 6 | Position-Velocity | int32+int16+int16, 8B | pos/10000=deg · spd int16(−32768~32767 → **−327680~327680 ERPM**, 즉 송신값=목표ERPM/10) · acc int16(1단위 = 10 ERPM/s²) |

> 사용자 스펙과의 차이: "Velocity: ERPM 그대로 (스케일 검증 필요)" → 모드 3 단독은 **ERPM 그대로 맞음** (매뉴얼 확정). 단 **모드 6(Position-Velocity)의 speed/acc 필드는 ÷10 스케일**이므로 혼동 주의. 매뉴얼 예제 코드도 `spd/10.0`으로 송신.

### 2.2 피드백 프레임 (모터 → 호스트, Function ID 0x29) ✅

주기 업로드 방식. 업로드 주파수 1~500Hz는 CubeMarsTool → Application Functions → "Send status over CAN" + Rate(Hz)에서 설정. DLC 8, Extended, ID = Function ID + Motor ID.

| 바이트 | 내용 | 타입 / 스케일 (매뉴얼 원문) |
|---|---|---|
| [0:1] | Position | int16 × 0.1 → **−3200° ~ 3200°** |
| [2:3] | Speed | int16 × 10 → **−320000 ~ 320000 ERPM** |
| [4:5] | Current | int16 × 0.01 → −60 ~ 60A |
| [6] | 온도 | int8, **−20~127℃** (오프셋 없음 — MIT 모드의 −40 오프셋과 다름) |
| [7] | Error Code | uint8 |

기타 Function ID: `0x09` = 점프 스타트 상태, `0x2C` = Servo 모드 진입 프레임 (응답 0xFA 0xFB 0xFC 0xFD 고정).

### 2.3 에러 코드 ✅
| 코드 | 의미 | 코드 | 의미 |
|---|---|---|---|
| 0 | 정상 | 4 | 저전압 |
| 1 | 모터 과열 | 5 | 엔코더 고장 |
| 2 | 과전류 | 6 | MOSFET 과열 |
| 3 | 과전압 | 7 | 모터 스톨 |

### 2.4 매뉴얼 원문 참조 코드 (송수신) ✅
```c
// 전송 (예: 속도 모드)
void comm_can_set_rpm(uint8_t controller_id, float rpm) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)rpm, &send_index);
    comm_can_transmit_eid(controller_id |
        ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

// 수신 (0x29 피드백 파싱)
int16_t pos_int = (Data[0] << 8) | Data[1];
int16_t spd_int = (Data[2] << 8) | Data[3];
int16_t cur_int = (Data[4] << 8) | Data[5];
motor_pos  = pos_int * 0.1f;    // deg
motor_spd  = spd_int * 10.0f;   // ERPM
motor_cur  = cur_int * 0.01f;   // A
motor_temp = (int8_t)Data[6];   // ℃
motor_err  = Data[7];
```

---

## 3. 단위 변환 (ERPM ↔ 출력축)

```
출력축 RPM = ERPM / (극쌍수 NPP × 기어비 36)
출력축 rad/s = 출력축 RPM × 2π / 60
```

⚠️ **AK45-36의 극쌍수(NPP)는 미확정.** 이전 CubeMarsTool 스크린샷의 NPP=21은 Motor type이 AK80_3으로 잘못 선택된 상태의 프리셋 값이므로 AK45-36 값으로 사용 금지. Motor type을 AK45-36으로 바꾼 뒤 Read Parameters로 실제 값 확인 후 이 문서와 코드 상수를 동시에 갱신할 것.

---

## 4. 물리적 제한값

- Servo 모드의 하드웨어 리밋(전압/전류/온도/듀티)은 CubeMarsTool → **System Settings**에서 모터에 저장된 값이 기준. Read Parameters로 읽어서 확인.
- 참고: MIT 모드 공식 파라미터(매뉴얼 5.3 표, AK45-36): 위치 ±12.5 rad, **속도 ±6.0 rad/s, 토크 ±34.0 N·m**. Servo 모드 CAN 명령 범위(±60A, ±100000 ERPM)는 프로토콜상 한계일 뿐 모터 물리 한계가 아니므로, **코드의 소프트 리밋은 MIT 표 값을 보수적 상한으로 사용 권장** (출력축 6 rad/s ≈ 57 RPM).
- ⚠️ AK45-36 정격/피크 전류는 미확정 — 제품 스펙시트 또는 CubeMarsTool Current Limits로 확인 후 기록.

---

## 5. 안전 규칙 (코드 작성 시 필수)

1. **소프트 리밋**: 명령 송신 전 클램핑. 속도는 출력축 ±6 rad/s 환산값 이하, 전류는 확정 전까지 보수적으로(예: ±5A) 제한.
2. **피드백 워치독**: 0x29 피드백이 N ms(예: 200ms) 이상 끊기면 Current Brake 0A 또는 Duty 0 송신 후 정지.
3. **에러 코드 처리**: error_code ≠ 0이면 즉시 명령 중단. 특히 1/2/6(과열·과전류)은 재시도 금지.
4. **명령 타임아웃**: 모터 쪽 Timeout(ms) 설정(Application Functions)과 호스트 송신 주기를 맞출 것 — 모터 타임아웃보다 짧은 주기로 명령 재송신해야 정지 안 함.
5. 온도 모니터링: 피드백 [6]이 지속 상승하면 듀티/전류 축소.

---

## 6. CANable / SocketCAN 설정

```bash
sudo ip link set can0 up type can bitrate 1000000   # 1Mbps 고정
candump can0                                         # 피드백 확인
# 예: Motor ID 0x68, 위치 모드(4), 100° = 1000000(0x000F4240)
cansend can0 00000468#000F4240
```

---

## 7. 미확정 사항 체크리스트

- [ ] 실제 모터 6대의 `Controller ID` (코드 placeholder ID1=0x01 ~ ID6=0x06, 서로 달라야 함) — 모터별로 CubeMarsTool → Application Functions에서 확인/설정. **변경 절차: Read Parameters → ID 입력 → Write Parameters → System Reset → 재접속 확인** (Read 선행은 매뉴얼 4.1.1.5 빨간 경고 사항)
- [x] ~~Velocity 명령 스케일~~ → 매뉴얼로 확정: 모드 3은 ERPM 그대로, 모드 6은 ÷10
- [x] ~~피드백 Speed 스케일~~ → 매뉴얼로 확정: int16 × 10 ERPM
- [ ] AK45-36 극쌍수(NPP) — CubeMarsTool에서 Motor type을 AK45-36으로 선택 후 Read Parameters로 확인
- [ ] AK45-36 정격/피크 전류 (Current Limits) — 확인 전까지 소프트 리밋 ±5A
- [ ] Servo 모드 하드웨어 리밋 실측값 (System Settings 저장값)
- [x] CANable 펌웨어 candlelight 확인 (SocketCAN 인식 확인됨)

---

## 8. 사용자 스펙 대비 정정 사항 요약

| 항목 | 기존 스펙 | 매뉴얼 확정값 |
|---|---|---|
| Velocity 명령 스케일 | "검증 필요" | 모드 3: ERPM 그대로 / 모드 6: 송신값 = ERPM÷10 ✅ |
| 피드백 Speed | "스케일 검증 필요" | int16 × 10 = ERPM ✅ |
| 피드백 온도 | int8 ℃ | int8, −20~127℃, 오프셋 없음 (MIT의 −40과 다름) ✅ |
| Position 명령 범위 | 명시 없음 | ±36000° (±100회전) ✅ |
| P/V/T 물리 제한 | "CubeMarsTool 확인 예정" | MIT 표 기준 속도 ±6 rad/s·토크 ±34 N·m는 확보. 전류 한계는 여전히 미확정 |
