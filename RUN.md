# RUN — 실행 런북

> 갱신: 2026-08-27 · 대상 PC: Ubuntu 22.04 / glibc 2.35 / ROS2 Humble / x86_64
>
> **이 문서는 실행 방법만 담는다.**
> 실측 기록·발견 사항·트러블슈팅은 [`FINDINGS.md`](FINDINGS.md)로 옮겼다. **안 될 때 그쪽을 본다.**
> 터미널에서 바로 보는 한 장짜리 요약은 `run.txt`.
> 프로토콜 상세는 `AGENTS.md`, ROS2 사양은 `ros2.md`(Phase 2는 §13), 프로젝트 상태는 `STATUS.md`.

## 어느 것을 쓸 것인가

| 하고 싶은 것 | 가는 곳 |
|---|---|
| 상태만 보고 싶다 (모터 안 움직임) | [시나리오 A](#시나리오-a--ros2-노드-확인-모터-안-움직임-안전) |
| **ROS2로 각도를 줘서 움직이고 싶다** | [**시나리오 A2**](#시나리오-a2--ros2로-모터-움직이기-phase-2) |
| ROS2 없이 CLI로 움직이고 싶다 / 완속 램프가 필요하다 | [시나리오 B](#시나리오-b--모터-움직이기-phase-0-cli) |
| 안 된다 | [`FINDINGS.md`](FINDINGS.md) |

## 먼저 알아야 할 것 2가지

**① ROS2 노드로 모터를 움직일 수 있다 (2026-08-27 Phase 2 완료).**
노드 이름이 `ak45_state_publisher` → **`ak45_node`** 로 바뀌었고 `/ak45/command` 구독이 추가됐다.
→ **"노드 띄우고 각도 주면 움직인다"가 이제 된다.** 아래 **시나리오 A2**를 본다.
단 **첫 명령을 받기 전에는 명령 프레임을 하나도 보내지 않는다** — 그때까지는 읽기 전용과 동일하다.

**② ROS2 노드와 `ak45_ctrl`을 동시에 띄울 수 없다.**
라이브러리가 `/tmp/ak45_ctrl.lock`에 `flock`을 걸어 AK45 프로세스를 시스템에 1개로 제한한다.
→ 터미널 1에서 노드, 터미널 2에서 `ak45_ctrl`로 각도 입력 = **불가능.** 나중에 띄운 쪽이 즉시 죽는다.
→ 그래서 아래 **시나리오 A와 B는 번갈아** 실행한다. 동시에 못 한다.

---

## 0단계 — 최초 1회만

```bash
sudo apt install can-utils        # candump / cansend. 3단계에 필수
```

**빌드는 반드시 `clean`부터.** Git에 커밋된 바이너리는 glibc 2.38 환경에서 링크된 것이라
이 PC(glibc 2.35)에서 `version 'GLIBC_2.38' not found`로 실행되지 않는다.
`make`만 돌리면 `.o` 타임스탬프 때문에 재링크를 건너뛰어 안 고쳐진다.

```bash
cd ~/바탕화면/Can
make clean && make                # ak45_ctrl, ak45_ctrl_demo 생성
```

> `ak45_demo`는 Makefile 타겟도 `make clean` 대상도 아닌 유령 바이너리다. 깨진 상태 그대로이므로 쓰지 않는다.

ROS2 패키지 빌드:

```bash
cd ~/ros2_ws
colcon build --packages-select ak45_ros2
```

---

## 1단계 — CAN 인터페이스 올리기 (재부팅마다 1회, sudo 필요)

```bash
lsusb                                              # CANable 인식 확인 (보통 1d50:606f)
sudo ip link set can0 up type can bitrate 1000000 restart-ms 100   # 1Mbps 고정
ip -details link show can0                         # state UP, bitrate 1000000 확인
```

`restart-ms 100` 을 **빼지 말 것.** 기본값 0 은 bus-off 가 나면 자동 복구를 안 한다 —
인터페이스가 죽은 채로 남고 `ip link down/up` 을 사람이 직접 해야 한다.
이 버스는 이미 상시 `ERROR-WARNING` 이라(→ `STATUS.md` P16) bus-off 가 현실적인 위험이다.

`can state` 가 `ERROR-WARNING`/`ERROR-PASSIVE` 로 보이면 **배선 문제다.** 확인:

```bash
candump -e can0,0~0,#FFFFFFFF        # 에러 프레임만. bit-stuffing 이 보이면 물리층 불량
ip -details -statistics link show can0   # error-warn / error-pass / bus-off 카운터
```

인터페이스가 `can1`으로 올라오면 **그대로는 못 쓴다.** 라이브러리가 `CAN_INTERFACE "can0"`
컴파일 타임 상수로 `can0`만 찾는다. 이름을 바꾼다:

```bash
sudo ip link set can1 down && sudo ip link set can1 name can0
```

### 하드웨어 없이 연습만 할 때 (가상 CAN)

모터는 안 움직이지만 노드 기동·토픽·QoS 검증은 된다. **이름을 `vcan0`이 아니라 `can0`으로 만들어야 한다.**

```bash
sudo modprobe vcan
sudo ip link add dev can0 type vcan
sudo ip link set up can0
```

---

## 2단계 — 피드백이 오는지 확인 (★ 건너뛰면 안 되는 관문)

```bash
candump can0
```

`0x00002901` ~ `0x00002906` Extended ID 프레임이 흘러야 정상이다 (피드백 ID = `(0x29 << 8) | 모터ID`).

**아무것도 안 흐르면 모터 설정 문제다.** 서보 모드 피드백은 주기 업로드 방식이고 기본으로 꺼져 있다:

> CubeMarsTool → **Application Functions** → **"Send status over CAN" 체크** → **Rate(Hz) 50 이상** → **Write Parameters**

이 상태에서 그냥 진행하면:
- ROS 노드: `/joint_states`가 영원히 빈 배열, `/diagnostics` 6개 전부 `level: 1`
- `ak45_ctrl_demo`: **현재 위치를 0도로 가정하고 램프를 시작한다** → 실제 위치가 90도면 급격히 움직인다 (아래 안전 주의 참조)

확인이 끝나면 Ctrl+C로 `candump`를 끈다.

---

## 시나리오 A — ROS2 노드 확인 (모터 안 움직임, 안전)

### 터미널 1 — 노드 실행

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ak45_ros2 ak45_node.launch.py
```

정상이면:

```
[INFO] ak45_node 시작. 인터페이스=can0, 상태=50.0Hz, 진단=2.0Hz, 명령 재송신=10.0Hz, 목표각 상한=±15.0도, 온도 상한=60.0도, 명령 타임아웃=없음(마지막 목표 홀딩)
```

### 터미널 2 — 상태 확인

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 topic list                            # /ak45/command /joint_states /diagnostics
                                           # /parameter_events /rosout
ros2 topic echo /joint_states              # position은 rad 단위
ros2 topic hz /joint_states                # 약 50Hz
ros2 topic echo /diagnostics               # status 7개 (모터 6 + can_bus 1)
ros2 topic hz /diagnostics                 # 약 2Hz
ros2 topic info /joint_states --verbose    # RELIABLE / VOLATILE / KEEP_LAST(10)
ros2 param get /ak45_node joint_names
```

읽는 법:

| 관측 | 의미 |
|---|---|
| `position`이 rad 단위 | 정상. deg → rad 변환만 한다 |
| `velocity: []`, `effort: []` | **정상.** NPP·KT 미확정으로 변환 불가 → 원본값은 `/diagnostics`에 있음 |
| `frame_id: ''` | 정상 |
| `/joint_states`의 name 개수가 6 미만 | 피드백이 오는 모터만 담긴다. 안 오는 모터는 빠진다 |
| `/diagnostics` status가 항상 6개 | 정상. 안 붙은 모터도 WARN으로 보고한다 |
| `level: 0` / `정상` | 해당 모터 OK |
| `level: 1` / `피드백 수신 없음` | 2단계(Send status over CAN)를 안 했다 |
| `level: 2` / `피드백 끊김: 200ms 초과` | 붙었다가 끊겼다. 배선·전원 확인 |

파라미터를 바꿔 띄우려면:

```bash
ros2 run ak45_ros2 ak45_node --ros-args \
  -p publish_rate_hz:=100.0 -p diagnostics_rate_hz:=2.0
```

허용 범위: `publish_rate_hz` 0 초과 500 이하 / `diagnostics_rate_hz` 0 초과 50 이하 /
`joint_names` **정확히 6개**. 위반하면 FATAL + 종료 코드 1.

### 종료

터미널 1에서 Ctrl+C. 이때 `candump`에 `0x00000201`~`0x00000206` 6개가 나가는 것은 **정상**이다 —
`ak45_close()`가 Current Brake **0A**를 보내는 것이라 모터는 움직이지 않는다.

---

## 시나리오 A2 — ROS2로 모터 움직이기 (Phase 2)

⚠️ **시나리오 B의 `ak45_ctrl`을 먼저 끈다.** flock 때문에 동시 실행이 안 된다.

### 터미널 1 — 노드

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ak45_ros2 ak45_node.launch.py
```

띄운 직후에는 **명령 프레임이 0건**이다. 첫 명령을 받기 전까지
아무것도 안 보내므로, 노드를 켜는 것만으로 모터가 튀지 않는다.

### 터미널 2 — 명령

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# ak45_2 를 10도로
ros2 run ak45_ros2 ak45_deg 10
```

**한 번만 보내면 된다.** 100ms 재송신은 **노드가 대신 한다** — 발행자가 계속 쏠 필요가 없다.
값을 바꿔 다시 보내면 목표가 즉시 갱신된다.

**이름 매칭이다. 인덱스가 아니다.** `ak45_ctrl`의 `0 10`(위치 순서)과 헷갈리지 말 것 —
여기서는 관절 이름으로 **직접 지정**한다. 모터가 1대뿐이어도 안전하다.

### 각도(도)로 보내기 — `ak45_deg` (권장)

`/ak45/command` 의 단위는 **rad** 다(ROS 표준 REP-103). 매번 환산하기 번거로우므로
**도 단위로 받아서 대신 환산해 주는 헬퍼**가 있다.

```bash
ros2 run ak45_ros2 ak45_deg 10            # ak45_2 를 10도로 (관절 생략 시 기본값)
ros2 run ak45_ros2 ak45_deg ak45_2 10     # 관절 명시
ros2 run ak45_ros2 ak45_deg ak45_2 10 ak45_3 -5   # 여러 관절 동시
ros2 run ak45_ros2 ak45_deg --list        # 현재 각도를 도 단위로 보기
```

출력:

```
ak45_2: 10.00도 -> 0.17453 rad
발행함 -> /ak45/command
노드가 100ms 주기로 재송신합니다. 반복 발행할 필요 없습니다.
```

`--list` 는 이렇게 나온다:

```
관절               현재각(도)          rad
------------------------------------
ak45_2             9.90      0.17279
```

> 이 헬퍼는 **환산과 발행만** 한다. 상한 검사는 노드가 하므로 15도를 넘겨도
> 스크립트는 막지 않는다 — **노드 터미널의 WARN 로그를 봐야 거부 여부를 안다.**

### rad 로 직접 보내기

정책 노드처럼 프로그램이 발행할 때는 표준 토픽을 그대로 쓴다.

```bash
ros2 topic pub -1 /ak45/command sensor_msgs/msg/JointState \
  "{name: ['ak45_2'], position: [0.1745]}"
```

| 도 | rad | 기본 설정에서 |
|---:|---|---|
| 1° | `0.01745` | ✅ 된다 |
| 5° | `0.08727` | ✅ 된다 |
| 10° | `0.1745` | ✅ 된다 |
| **15°** | **`0.2618`** | ✅ **여기까지가 최대** |
| 30° | `0.5236` | ❌ 거부 + WARN |
| 45° | `0.7854` | ❌ 거부 + WARN |
| 90° | `1.5708` | ❌ 거부 + WARN |
| 180° | `3.1416` | ❌ 거부 + WARN |

음수도 같은 값에 부호만 붙이면 된다 (`-0.1745` = −10°).

**거부되면 이 로그가 뜬다:**

```
[WARN] 목표각 30.0도가 상한 15.0도를 넘습니다. 무시합니다. (기준축 판별 전)
```

**이건 고장이 아니라 안전장치 S1이다.** 라디안 환산이 틀린 게 아니다 — 변환은 정상이고
(`0.5236 rad` = `30.0도`가 로그에 그대로 찍힌다) 그 다음에 상한이 막은 것이다.
**클램프가 아니라 거부**이므로 모터는 15°로도 안 가고 아예 안 움직인다.

#### 15°보다 크게 움직이려면

상한이 15°인 이유는 **위치 피드백이 출력축 기준인지 모터축 기준인지 아직 모르기 때문이다.**

| 180°를 보냈을 때 | 출력축 기준이면 | 모터축 기준이면 |
|---|---|---|
| 실제 출력축 회전 | **반 바퀴** | 약 5° |

출력축 기준인데 큰 값을 보내면 기구부를 부순다. 그래서 판별 전까지 묶어 둔 것이다.

올리는 방법:

```bash
# ak45_ros2/config/ak45_node.yaml 에서
#   max_command_deg: 15.0   ->   원하는 값 (최대 360.0)
colcon build --packages-select ak45_ros2   # config 를 install 로 복사
# 노드 재시작
```

⚠️ **올리기 전에 출력축에 무엇이 붙어 있는지 먼저 본다.**
맨 축이면 마음껏 올려도 된다. 링크·기구부가 붙어 있으면 물리적으로 허용되는 각도까지만 올린다.

### 상태 보기

```bash
ros2 topic echo /joint_states       # position 은 rad
ros2 topic echo /diagnostics        # status 7개. 모터는 KeyValue 9개
```

진단에서 Phase 2로 볼 것:

| key | 의미 |
|---|---|
| `target_deg` | 현재 목표각(deg). 명령을 받은 적 없으면 `none` |
| `command_active` | 재송신 중이면 `1`, S1~S4로 제외됐으면 `0` |
| `position_error_deg` | `target_deg - position_deg` |

`ak45/can_bus` 항목 (2026-09-03 추가) — CAN 물리층. 재송신이 가려주는 문제를 드러낸다:

| key | 의미 |
|---|---|
| `state` | `정상` / `경고(error-warning)` / `수동(error-passive)` / `버스오프(bus-off)` |
| `bit_stuff_errors` | **배선·종단저항 품질 지표.** 계속 오르면 물리층 불량 |
| `form_errors` | 프레임 포맷 위반. 위와 같은 원인 |
| `ack_errors` | **0이어야 정상.** 0이 아니면 그 ID를 받는 노드가 버스에 없다 |
| `busoff_count` | bus-off 진입 횟수. 0이 아니면 `restart-ms` 를 확인 |
| `restart_count` | 자동 복구 횟수 |

⚠️ 이 값이 오르는 것 자체는 **코드로 못 고친다.** 종단저항 120Ω·배선 문제다.

### 안전장치 — 걸리면 정상이다

| # | 조건 | 증상 |
|---|---|---|
| **S1** | 목표각 > **±360°** (`max_command_deg`) | 모터가 안 움직이고 WARN 1줄. **클램프가 아니라 거부** |
| **S2** | 온도 >= **60°C** | 그 모터 명령 중단. **온도가 내려가도 자동 복귀 안 함 — 노드 재시작 필요** |
| **S3** | 피드백 없는 모터 | 명령 프레임이 안 나간다. 지금 ID 0x02 외 5대가 여기 해당. **2026-09-03부터 WARN 을 찍는다** (전에는 무로그였다) |
| **S4** | 모터 `error_code != 0` | 라이브러리가 프레임을 막고 노드가 ERROR 로그 |

**`max_command_deg` 는 현재 360.0** 이다. 2026-08-27 기준축 판별(출력축 기준, `ros2.md` §13.10)과
90° 실기 검증이 끝나 라이브러리 상한(`SOFT_LIMIT_POS_DEG 360.0f`)까지 열어 뒀다.
⚠️ **출력축에 링크·기구부가 붙으면 물리적 허용 각도까지 다시 낮춰라.**
바꾸려면 `config/ak45_node.yaml` 을 고치고 노드를 재시작한다.

### 종료

`Ctrl+C` → 브레이크 0A 프레임 6개(`0x201`~`0x206`) 후 종료.
**홀딩 토크가 풀리므로 출력축에 무게가 있으면 떨어진다.**

## 시나리오 B — 모터 움직이기 (Phase 0 CLI)

⚠️ **시나리오 A의 노드를 먼저 끈다.** flock 때문에 동시 실행이 안 된다.

### 안전 순서 — 반드시 이 순서로

**① 먼저 현재 각도를 읽는다 (모터 안 움직임)**

```bash
cd ~/바탕화면/Can
./ak45_ctrl                # 인자 없음 = 위치 명령 0건 (모터 안 움직임)
```

인자를 주지 않으면 `ak45_set_position()`을 아예 호출하지 않는다. 모터는 움직이지 않는다.
화면에 ID별로 `Pos(deg) Spd(ERPM) Cur(A) Temp(C) Error`가 실시간 갱신된다.
**여기서 각 모터의 현재 각도를 적어둔다.** 종료는 `q` 또는 Ctrl+C.

⚠️ **단, "읽기 전용"은 아니다.** 피드백 없는 모터마다 100ms 주기로 Current Brake **0A**가 나가고
stderr에 `[워치독] ... 긴급 정지`가 초당 50줄 찍힌다. 0A라 모터는 안 움직이지만 화면이 안 보이므로
`./ak45_ctrl 2>/dev/null` 로 띄운다.
→ 순수 읽기 전용이 필요하면 **ROS2 노드**(시나리오 A)를 쓴다. (근거·실측: `FINDINGS.md`)

**② 터미널 2에서 CAN 프레임을 보며 소각도부터**

터미널 2:
```bash
candump can0
```

터미널 1:
```bash
cd ~/바탕화면/Can
./ak45_ctrl                # 다시 인자 없이 띄운다
```

프롬프트가 뜬다:

```
> 목표각도 입력 (예: 30 / 30 20 10 (최대 6개), q=종료):
```

여기서 **`10`** 을 치고 Enter → **ID1이 10도로 이동한다.**

> ⚠️ **입력은 ID 순서 위치 매칭이다.** 첫 값이 ID1, 둘째 값이 ID2다.
> **현재 버스에는 ID 0x02 하나뿐이라 `10`만 치면 없는 ID1로 가서 아무 일도 안 일어난다.**
> ID2를 움직이려면 값을 2개 줘야 한다 — **`0 10`**.
> (ROS2 시나리오 A2는 이름 매칭이라 이 함정이 없다: `name: ['ak45_2']`)

| 입력 | 결과 |
|---|---|
| `10` | ID1만 10도로 이동 |
| `10 15` | ID1=10도, ID2=15도 동시 이동 |
| `10 15 20 0 0 0` | ID1~ID6 전부 지정 (최대 6개) |
| `-10` | 음수 각도 가능 |
| `>10 15` | `>` 접두사는 옛 표기 호환용. 붙여도 되고 안 붙여도 된다 |
| `q` 또는 `quit` | 안전 정지(Current Brake 0A) 후 종료 |

값은 소프트 리밋 **±360도**로 클램핑된다. 목표는 100ms 주기로 계속 재송신된다 —
재송신을 멈추면 모터 쪽 타임아웃으로 홀딩 토크가 풀리기 때문이다.

**처음에는 10~15도부터.** 잘 움직이는 것을 확인한 뒤 값을 올린다.

### 시연용 완속 이동 (`ak45_ctrl_demo`)

현재 위치에서 목표까지 1도씩 계단식으로 천천히 이동한다.

```bash
./ak45_ctrl_demo 10                              # ID1을 10도까지 1도/150ms
./ak45_ctrl_demo 10 --step=0.5 --interval=200    # 0.5도씩 200ms 간격 (더 느리게)
./ak45_ctrl_demo 10 15                           # ID1=10도, ID2=15도 동시
```

기본 속도는 1도/150ms이므로 45도 ≈ 6.8초, 90도 ≈ 13.5초 걸린다.
목표 도달 후에도 Ctrl+C까지 위치 유지 신호를 계속 보낸다.

⚠️ **`demo`의 인자 해석 주의 — `0 0 0 45 90 0`을 "3개는 가만히"로 읽으면 틀린다.**
인자를 6개 주면 ID1~ID6 **전부가 명령 대상**이 된다. `0`을 준 모터는 "0도로 가라"는 명령을 받고
현재 위치가 0이 아니면 **실제로 움직인다.** 인자는 앞에서부터 ID1,2,3... 순서로 매핑되므로
ID4·5만 골라 움직이는 것은 불가능하다. 가만히 두려면 그 자리에 **현재 각도**를 넣거나,
아예 인자를 주지 않는다(`./ak45_ctrl_demo 10` = ID1만).

⚠️ **피드백이 없으면 램프가 무력화된다.** demo는 시작 위치를 피드백에서 읽는데,
못 받으면 **0도로 가정**한다. 실제 모터가 90도에 있으면 첫 명령이 "1도로 가라"가 되어
모터가 90→1도를 자기 최대 속도로 간다. 계단식 이동이 전혀 보호하지 못한다.
→ **2단계(`candump`로 피드백 확인)를 반드시 먼저 한다.**
또한 피드백 대기 3초는 모터별이 아니라 **전체 공유 예산**이다. ID1이 3초를 다 쓰면
ID2~6은 대기 없이 즉시 0도 가정으로 넘어간다.

---

## 안 될 때

### 「명령을 보냈는데 모터가 안 움직인다」

이 순서로 본다. **1번을 건너뛰면 안 된다** — 가장 흔한 오진이다.

1. **이미 그 각도에 있는 것 아닌가.** `ak45_deg --list` 로 현재각을 먼저 본다.
   같은 값을 다시 보내면 당연히 아무 일도 안 일어난다.
2. **노드 터미널에 `목표각 수신` INFO 가 찍혔나.**
   - 찍혔다 → 명령은 도달했다. 3번으로.
   - 안 찍혔다 → **도달하지 않았다.** 노드가 떠 있는지, `ros2 topic info -v /ak45/command` 의
     `Subscription count` 가 1인지 확인. (2026-09-03 이전 `ak45_deg` 는 `pub -1` 을 써서
     드물게 조용히 유실됐다 → `STATUS.md` P17)
3. **WARN/ERROR 가 찍혔나.** S1(상한 초과) / S2(온도) / S3(피드백 없음) / S4(모터 에러) 중 하나다.
4. **프레임이 실제로 나가나.** `candump can0 | grep 00000402` — 100ms 간격으로 나와야 한다.
5. **버스가 살아 있나.** `ros2 topic echo /diagnostics` 의 `ak45/can_bus` 를 본다.

증상별 확인 순서, 실측 기록, 발견 사항(종료 시 브레이크 프레임 누락 / 온도 상승 / 모터 1대)은
**[`FINDINGS.md`](FINDINGS.md)** 에 있다.

아직 안 끝난 것(기준축 판별 V3 등)은 `STATUS.md`와 `run.txt` 말미를 본다.
