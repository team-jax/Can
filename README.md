# Can — CubeMars AK45-36 로봇팔 제어

CubeMars **AK45-36 KV80** 액추에이터 **6대(CAN ID 0x01~0x06)** 를 Linux + CANable(SocketCAN) 환경에서
**Servo 모드 CAN**으로 제어한다. MIT 모드는 사용하지 않는다.
최종 목표는 ROS2(Humble) + MoveIt2로 궤적을 계획·실행하는 휴머노이드 팔이다.

프로토콜 근거: **CubeMars AK Series Module Driver Manual V1.0.18** (2026.01.19, AK 2.0용) — 원문 대조 완료.

## 진행 상태

| Phase | 내용 | 상태 |
|---|---|---|
| **Phase 0** | C++ 제어 라이브러리 + CLI로 6대 직접 제어 | **완료** |
| **Phase 1** | ROS2 읽기 전용 상태 발행 노드 (`/joint_states`, `/diagnostics`) | **완료 (실기 검증 통과)** |
| **Phase 2** | ROS2 명령 인터페이스 (`/ak45/command` → 모터 이동) | **완료 (2026-08-27, P1~P10 실기 검증 통과)** |
| Phase 3 | URDF + `robot_state_publisher` + MoveIt2 | 미착수 |

세부 진행 상황·미결 항목은 [`STATUS.md`](STATUS.md)를, **실행 방법은 [`RUN.md`](RUN.md)** 를,
실측 기록·트러블슈팅은 [`FINDINGS.md`](FINDINGS.md)를 본다.
ROS2 노드 사양은 [`ros2.md`](ros2.md) (Phase 2는 §13), 패키지는 [`ak45_ros2/`](ak45_ros2/)에 있다.

## ROS2로 모터 움직이기 (Phase 2)

```bash
# [터미널 1]
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch ak45_ros2 ak45_node.launch.py

# [터미널 2]  ak45_2 를 10도로
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run ak45_ros2 ak45_deg 10
```

`ak45_deg` 는 **도 단위로 받아 rad 로 환산해 발행하는 헬퍼**다. 토픽 자체는 ROS 표준대로
rad(`sensor_msgs/msg/JointState`)이므로, 정책 노드가 붙을 때는 표준 토픽을 그대로 쓰면 된다.

재송신은 **노드가** 100ms 주기로 대신 한다. 한 번만 보내면 된다.
**기본 상한은 ±15°이고 넘으면 클램프가 아니라 거부된다** (기준축 판별 전 안전장치).

## 하드웨어

| 항목 | 내용 |
|---|---|
| 모터 | AK45-36 KV80 ×6 (BLDC + 36:1 유성기어 + 드라이버 일체형) |
| CAN 어댑터 | CANable (candlelight 펌웨어, SocketCAN `can0` 인식 확인됨) |
| CAN 속도 | **1Mbps 고정** (매뉴얼: "1Mbps, No change recommended") |
| 프레임 | CAN 2.0B **Extended (29bit ID)**, big-endian — Servo 모드는 확장 프레임만 응답 |
| 제어 모드 | **Servo 모드** (MIT 모드 미사용) |
| OS | Linux (Ubuntu 22.04에서 검증) |

## 저장소 구성

| 파일 | 역할 |
|---|---|
| `ak45_36_socketcan_control.h` / `.cpp` | **제어 라이브러리(메인).** SocketCAN 초기화, 수신 스레드, 피드백 파싱(0x29), 명령 함수(클램핑·에러 차단), 모터별 워치독, `flock` 단일 프로세스 잠금 |
| `main.cpp` | → `ak45_ctrl`. 피드백 모니터링 + 터미널 목표각도 입력 CLI |
| `demo_ramp.cpp` | → `ak45_ctrl_demo`. 시연용 계단식 완속 이동 |
| `Makefile` | `make` / `make clean` |
| `AGENTS.md` | **프로토콜 레퍼런스(§1~§8)** — 매뉴얼 원문 대조 결과, 안전 규칙, 미확정 체크리스트. 이 저장소의 프로토콜 정본 |
| `ros2.md` | **Phase 1 ROS2 노드 구현 명세서 v3.0** — 토픽·QoS·로그 문구·검증 항목 V0~V15 |
| `STATUS.md` | 프로젝트 전체 상태 — 했던 일 / 하는 일 / 할 일 / 문제점 P1~P12 |
| `run.txt` | 실기 실행 기록 (45°/90° 이동 로그) |

> `.o`·바이너리(`ak45_ctrl`, `ak45_ctrl_demo`, `ak45_demo`)가 저장소에 커밋되어 있고 `.gitignore`가 없다.
> `make`를 돌리면 working tree가 더러워지므로 커밋 전 `git status`를 확인할 것.
> `ak45_demo`는 현재 Makefile이 만들지 않는 **구 바이너리**다(`make clean` 대상도 아님).

## 빌드 · 실행 (Phase 0)

```bash
# 1) CAN 인터페이스 활성화 (재부팅마다 1회)
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0          # state UP, bitrate 1000000 확인

# 2) 피드백이 실제로 오는지 먼저 확인 (★ 가장 중요)
candump can0                        # 0x00002901~0x00002906 프레임이 흘러야 정상

# 3) 빌드
make                                # → ak45_ctrl, ak45_ctrl_demo
```

**2번이 안 되면 프로그램을 아무리 잘 짜도 피드백이 오지 않는다.**
CubeMarsTool → Application Functions → **"Send status over CAN" 체크 + Rate(Hz) 50 이상** → Write Parameters.
Rate = 0이 가장 흔한 오진 원인이다.

### `ak45_ctrl` — 모니터링 + 목표각도 입력

```bash
./ak45_ctrl                  # 피드백만 모니터링 (ID1~ID6 전부)
./ak45_ctrl 30               # ID1을 30도로
./ak45_ctrl 30 20 10         # ID1=30, ID2=20, ID3=10 (입력한 개수만큼만 반영)
```

실행 중 언제든 목표를 갱신할 수 있다 (공백 구분, 최대 6개, `q`로 종료):

```
> 30              → ID1만 30도
> 30 20 10        → ID1=30, ID2=20, ID3=10 동시 이동
```

### `ak45_ctrl_demo` — 시연용 완속 이동

목표까지 한 번에 점프하지 않고 `--step`씩 `--interval` 간격으로 계단식 이동한다.
대기 중에도 100ms 주기 CAN 재송신·워치독 체크·피드백 출력은 계속된다(논블로킹 타이머).

```bash
./ak45_ctrl_demo 45                              # ID1을 현재 위치→45도까지 1도씩
./ak45_ctrl_demo 45 30 --step=0.5 --interval=200 # ID1=45, ID2=30 동시, 0.5도씩 200ms 간격
```

기본값: `--step=1`(도), `--interval=150`(ms).

> **`ak45_ctrl`과 `ak45_ctrl_demo`는 동시에 실행할 수 없다.** 둘 다 `/tmp/ak45_ctrl.lock`을 `flock`으로
> 잠근다(같은 CAN 버스 명령 충돌 방지). ROS2 노드도 같은 락을 쓰므로 **노드와 CLI도 동시 실행 불가**다.
> "초기화 실패"의 흔한 원인이며, 락 파일을 지울 필요는 없다 — 프로세스가 죽으면 OS가 회수한다.

## ROS2 Phase 1 — 상태 발행 노드

모터를 **움직이지 않고** 피드백만 `/joint_states`(50Hz)·`/diagnostics`(2Hz)로 발행하는 읽기 전용 노드.
표준 메시지만 쓰고 커스텀 메시지를 만들지 않는다.

- 사양서: [`ros2.md`](ros2.md) (v3.0) — 이름·값·QoS·로그 문구·검증 항목 V0~V15가 전부 확정값
- 구현: `~/ros2_ws/src/ak45_ros2` — **아직 어느 저장소에도 속하지 않는다** (`STATUS.md` P11)

```bash
cd ~/ros2_ws
colcon build --packages-select ak45_ros2
source install/setup.bash
ros2 launch ak45_ros2 state_publisher.launch.py
```

검증 현황: 빌드(GCC 11.4.0 경고 0개)·사양 문구 감사·실패 경로(파라미터 위반 3종, `ak45_init()` 실패) **통과**.
V0~V15는 `can0` 인터페이스 필요, **V3(위치 피드백 기준축 판별)은 실모터 필요**.

`velocity`/`effort`는 **항상 빈 배열**이다 — NPP·KT 미확정으로 변환식을 만들 수 없다(아래 참조).

## 프로토콜 요약

전문은 **[`AGENTS.md`](AGENTS.md) §1~§8**에 있다(매뉴얼 원문 대조 완료). 여기서는 콘솔에서 바로 쓰는 것만.

**명령** — CAN ID = `(제어모드 << 8) | Controller_ID`, Extended, big-endian

| 모드 | 이름 | 스케일 |
|---|---|---|
| 0 | Duty Cycle | 값/100000 = 듀티 |
| 1 | Current Loop | 값/1000 = A (±60A) |
| 2 | Current Brake | 값/1000 = A (0~60A) |
| 3 | Velocity | **그대로 ERPM** (±100000) |
| 4 | Position | 값/10000 = deg (±36000°) |
| 5 | Set Origin | 0=임시, 1=영구(듀얼 엔코더 전용) |
| 6 | Position-Velocity | pos/10000=deg · **spd·acc는 ÷10 스케일** |

> ⚠️ 모드 3의 속도는 ERPM 그대로인데 **모드 6의 속도 필드는 ÷10**이다. 가장 헷갈리는 지점.

**피드백** — CAN ID = `(0x29 << 8) | Controller_ID`, DLC 8 (ID1=`0x2901` … ID6=`0x2906`)

| 바이트 | 내용 | 타입 / 스케일 |
|---|---|---|
| `[0:1]` | Position | int16 × 0.1 → ±3200° |
| `[2:3]` | Speed | int16 × 10 → ±320000 **ERPM** |
| `[4:5]` | Current | int16 × 0.01 → ±60A |
| `[6]` | 온도 | int8, −20~127℃ (**오프셋 없음** — MIT의 −40과 다름) |
| `[7]` | Error Code | uint8 (0~7) |

**에러 코드**: 0 정상 · 1 모터 과열 · 2 과전류 · 3 과전압 · 4 저전압 · 5 엔코더 고장 · 6 MOSFET 과열 · 7 모터 스톨

```bash
# 수동 프레임 송신 예: Motor ID 0x68, 위치 모드(4), 100° = 1000000(0x000F4240)
cansend can0 00000468#000F4240
```

## 안전 규칙

1. **소프트 리밋**: 명령 송신 전 클램핑. 상수는 `ak45_36_socketcan_control.h` 상단
   (`SOFT_LIMIT_CURRENT_A 5.0f`, `SOFT_LIMIT_ERPM 500`, `SOFT_LIMIT_POS_DEG 360.0f`).
   6대가 같은 리밋을 공유한다.
2. **피드백 워치독**: 0x29 피드백이 200ms(`WATCHDOG_TIMEOUT_MS`) 이상 끊기면 해당 모터 정지.
   판정식이 `elapsed < 200`이므로 **정확히 200ms는 실패로 친다.**
3. **에러 코드 처리**: `error_code ≠ 0`이면 즉시 명령 중단. 특히 1/2/6(과열·과전류)은 재시도 금지.
4. **명령 재송신**: 라이브러리에는 재송신 루프가 **없다.** 명령 함수 1회 = CAN 프레임 1개다.
   실제 100ms 루프는 애플리케이션(`main.cpp`, `demo_ramp.cpp`)에 있다.
   **재송신을 멈추면 모터 타임아웃으로 홀딩 토크가 풀린다.**
5. **온도 모니터링**: 피드백 `[6]`이 지속 상승하면 듀티/전류 축소.

## 미확정 사항

Phase 2(모터 이동 명령) 착수 전에 반드시 확정해야 하는 것들이다. 전체 목록은 `STATUS.md` P1~P12.

- ⚠️ **AK45-36 극쌍수(NPP)** — 매뉴얼 전문에 없음(부재 확인). ERPM→rad/s 변환이 막혀 있다.
  **기존 CubeMarsTool 스크린샷의 `NPP=21`을 쓰면 안 된다** — Motor type이 `AK80_3`으로 잘못 선택된
  상태의 프리셋 값이다. Motor type을 **AK45-36으로 바꾼 뒤** Read Parameters로 실제 값을 확인할 것.
- ⚠️ **토크 상수(KT)** — 매뉴얼에 식만 있고 AK45-36 값이 없음. 전류→토크 변환 불가.
- ⚠️ **`SOFT_LIMIT_ERPM`** — 현재 `500`은 **시연용 임시값**(초본 `10000`). 정식값 공식은
  `출력축 6 rad/s × NPP × 36`이므로 NPP 확정이 선행 조건이다.
- ⚠️ **정격/피크 전류** — 매뉴얼은 드라이버 보드별 정격만 제시. **실물 보드 실크 표기를 눈으로 확인**해야
  확정된다. 그때까지 `SOFT_LIMIT_CURRENT_A 5.0f` 유지.
- ⚠️ **위치 피드백 기준축** — 출력축인지 모터축인지 매뉴얼에 명시가 없다. 모터축이면 각도에 `/36`이
  필요하다. 판별 절차는 `ros2.md` §11.4 (V3).
- ⚠️ **URDF** — 7DOF/8DOF 조율 중. TF 트리·관절 부호·영점 오프셋이 전부 여기 걸려 있다.

## 문서 지도

| 문서 | 담는 것 |
|---|---|
| `README.md` (이 문서) | 프로젝트 소개, 빌드·실행, 프로토콜 요약 |
| `AGENTS.md` | 코드 작업 규칙 + **프로토콜 정본 §1~§8** (매뉴얼 대조 결과, 안전 규칙, 미확정 체크리스트) |
| `ros2.md` | Phase 1 ROS2 노드 구현 명세 (v3.0) |
| `STATUS.md` | 진행 상황, 문제점 P1~P12와 해결 방안 |

프로토콜 값을 고칠 때는 `AGENTS.md`가 정본이다. 이 README의 요약 표와 어긋나면 `AGENTS.md`를 따른다.

## 라이선스

**미정.** LICENSE 파일이 없고 팀이 정한 바 없다 (`STATUS.md` P12).
ROS2 패키지 `package.xml`의 `<license>`도 그래서 `TODO`로 두었다.
