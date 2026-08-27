# ak45_ros2

CubeMars AK45-36 6개를 다루는 ROS2 Humble 노드 **하나**(`ak45_node`).

- **Phase 1 — 읽기**: CAN 피드백을 `/joint_states`(50Hz) · `/diagnostics`(2Hz)로 발행
- **Phase 2 — 쓰기**: `/ak45/command`를 구독해 100ms 주기로 `ak45_set_position()` 재송신

사양서는 저장소 루트 `ros2.md` (v4.0). **Phase 2는 §13이 정본이다.**

> 노드를 **둘로 나누지 마라.** 라이브러리가 `flock(/tmp/ak45_ctrl.lock)`으로 프로세스당 1개만
> `ak45_init()`에 성공시킨다. 나중에 뜬 쪽은 반드시 죽는다.

## 1. 라이브러리 원본 커밋 해시

`include/ak45_ros2/ak45_36_socketcan_control.h` 와 `src/ak45_36_socketcan_control.cpp` 는
`team-jax/Can` 저장소에서 복사해 온 파일이며 **한 글자도 수정하지 않는다.**

| 항목 | 값 |
|---|---|
| 원본 저장소 | `team-jax/Can` — **이제 이 패키지도 같은 저장소 안에 있다** (`Can/ak45_ros2/`) |
| 원본 파일 경로 | 저장소 루트의 `ak45_36_socketcan_control.{h,cpp}` |
| 복사 시점 커밋 해시 | `893d663fdb26f1bf5c203067545a9f9888e4e301` |
| 복사 일자 | 2026-08-20 |
| 내용 동일성 | **2026-08-27 `diff` 확인 — 원본과 완전히 동일** |

원본을 고칠 일이 생기면 **저장소 루트에서 고치고** 여기 복사본을 다시 맞춘 뒤 이 해시를 갱신한다.

> ⚠️ **같은 저장소 안에 같은 파일이 두 벌 있다.** 루트 원본은 `Makefile`이 `-std=c++11`로,
> 여기 복사본은 `colcon`이 **C++17**로 컴파일한다. 양쪽에서 모두 컴파일되는 코드만 써야 한다.
> `diff`로 주기적으로 확인할 것:
> ```bash
> diff ../ak45_36_socketcan_control.h  include/ak45_ros2/ak45_36_socketcan_control.h
> diff ../ak45_36_socketcan_control.cpp src/ak45_36_socketcan_control.cpp
> ```

## 2. CAN 인터페이스는 `can0` 고정

`CAN_INTERFACE` 가 `#define "can0"` 인 **컴파일 타임 상수**다
(`include/ak45_ros2/ak45_36_socketcan_control.h:15`).

→ **`can_interface` ROS 파라미터는 존재하지 않는다.** 인터페이스를 바꾸려면
헤더의 `CAN_INTERFACE` 를 직접 고치고 재빌드해야 한다.

## 3. 실행 전 준비

```bash
# 1) CAN 어댑터 인식
ip -details link show can0        # 없으면 lsusb 로 어댑터부터 확인

# 2) CAN 인터페이스 활성화 (재부팅마다 1회, sudo 필요)
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0        # state UP, bitrate 1000000 확인

# 3) 모터 피드백이 실제로 오는지 확인 (★ 가장 중요)
candump can0
#   → 0x00002901 ~ 0x00002906 Extended ID 프레임이 흘러야 정상
#   → 안 오면 CubeMarsTool → Application Functions →
#     "Send status over CAN" 체크 + Rate(Hz) 50 이상 → Write Parameters
```

**3번이 안 되면 노드를 아무리 잘 짜도 `/joint_states` 는 빈 배열이다.**

빌드·실행:

```bash
cd ~/ros2_ws
colcon build --packages-select ak45_ros2
source install/setup.bash
ros2 launch ak45_ros2 ak45_node.launch.py
```

## 4. 터미널 2개로 쓰는 법

**[터미널 1] 노드**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ak45_ros2 ak45_node.launch.py
```

**[터미널 2] 명령 발행**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# ak45_2 를 0도로
ros2 topic pub -1 /ak45/command sensor_msgs/msg/JointState \
  "{name: ['ak45_2'], position: [0.0]}"

# 값을 바꾸면 즉시 따라간다 (10도)
ros2 topic pub -1 /ak45/command sensor_msgs/msg/JointState \
  "{name: ['ak45_2'], position: [0.1745]}"

# 상태 보기
ros2 topic echo /joint_states
ros2 topic echo /diagnostics
```

`-1` 은 "한 번만 발행"이다. 재송신은 **노드가** 100ms 주기로 대신 해주므로
발행자가 계속 쏠 필요가 없다. 나중에 강화학습 정책 노드가 터미널 2 자리를 대신해도 동일하다.

### 도 → 라디안 환산

`position` 은 **rad** 이다. 자주 쓰는 값:

| 도 | rad | 도 | rad |
|---:|---|---:|---|
| 1° | `0.01745` | 30° | `0.5236` |
| 5° | `0.08727` | 45° | `0.7854` |
| **10°** | **`0.1745`** | 90° | `1.5708` |
| **15°** | **`0.2618`** | 180° | `3.1416` |

> ⚠️ **기본 상한은 ±15°(`max_command_deg`)다.** 30° 이상을 보내면 **거부**되고 WARN이 뜬다.
> 클램프가 아니라 거부이므로 모터는 움직이지 않는다. 상한을 올리려면 아래 V3 판별을 먼저 한다.

## 5. 안전장치 (전부 켜져 있다)

| # | 조건 | 동작 |
|---|---|---|
| S1 | 목표각 절대값 > `max_command_deg` (기본 15°) | **그 관절만 거부.** 클램프 아님. WARN |
| S2 | 온도 >= `max_temperature_c` (기본 60℃) | 그 모터 **영구 제외.** 온도가 내려가도 **자동 복귀 안 함 — 노드 재시작 필요.** ERROR |
| S3 | 피드백 없음(`valid==0`) 또는 워치독 실패 | 그 모터 제외. 복귀 가능 |
| S4 | `ak45_set_position()` 이 `-1` 반환 | ERROR 로그. 모터 `error_code != 0` 이면 라이브러리가 프레임을 막는다 |

**첫 명령을 받기 전에는 명령 프레임을 하나도 보내지 않는다.** 노드를 켜는 순간
모터가 0도로 튀는 것을 막기 위해서다.

## 6. 종료하면 홀딩 토크가 풀린다

Ctrl+C 시 `ak45_close()` → `ak45_emergency_stop()` 으로 **Current Brake 0A 프레임 6개**
(`0x00000201` ~ `0x00000206`)가 나간다. 0A라 모터가 **힘을 놓는다.**

→ **출력축에 무게가 걸려 있으면 중력으로 떨어진다. 미리 받쳐둘 것.**

## 7. `joint_names` 에 중복 이름을 넣지 말 것

노드는 `joint_names` 의 **개수(정확히 6개)만 검증하고 중복은 검증하지 않는다.**
중복된 이름을 넣으면 `robot_state_publisher` 가 같은 이름의 마지막 값으로 덮어써서
**TF가 조용히 어긋난다.** 에러도 경고도 뜨지 않으므로 원인을 찾기 매우 어렵다.

## 8. 파라미터

| 이름 | 타입 | 기본값 | 허용 범위 | 위반 시 |
|---|---|---|---|---|
| `publish_rate_hz` | `double` | `50.0` | `> 0.0` && `<= 500.0` | FATAL 후 종료 코드 1 |
| `diagnostics_rate_hz` | `double` | `2.0` | `> 0.0` && `<= 50.0` | FATAL 후 종료 코드 1 |
| `joint_names` | `string[]` | `["ak45_1" ... "ak45_6"]` | 길이가 정확히 6 | FATAL 후 종료 코드 1 |
| `command_resend_rate_hz` | `double` | `10.0` | `> 0.0` && `<= 50.0` | FATAL 후 종료 코드 1 |
| `max_command_deg` | `double` | `15.0` | `> 0.0` && `<= 360.0` | FATAL 후 종료 코드 1 |
| `command_timeout_sec` | `double` | `0.0` | `>= 0.0` (0=타임아웃 없음) | FATAL 후 종료 코드 1 |
| `max_temperature_c` | `double` | `60.0` | 검증 없음 | — |

**런타임 변경 불가.** 값은 `config/ak45_node.yaml` 에서 바꾸고 재시작한다.

`command_timeout_sec` 기본이 `0.0`(타임아웃 없음)인 이유: 정책 노드가 끊겨도 팔이 갑자기
힘을 놓으면 중력으로 떨어진다. **홀딩 유지가 기본이고 끊김 감지는 사람이 켠다.**

## 9. 알려진 미확정 항목

- `velocity` / `effort` 는 **항상 빈 배열**이다. AK45-36의 극쌍수(NPP)와 토크 상수(KT)가
  매뉴얼에 없어 ERPM→rad/s, 전류→토크 변환식을 만들 수 없다. 원본 ERPM 값은
  `/diagnostics` 의 `speed_erpm` KeyValue 에 실린다.
- `position` 이 **출력축 기준인지 모터축 기준인지 미확정**이다. 모터축이면 변환식에
  `/ 36.0` 이 추가되어야 한다. `ros2.md` §11.4 V3 절차로 판별한다.
  **`max_command_deg` 기본값 15.0 이 보수적인 것은 이것 때문이다.** 판별 전에는 올리지 말 것.
  ⚠️ 명령을 줬을 때 피드백이 따라오는 것은 **판별이 아니다** — 명령과 피드백이 같은 축을
  쓴다는 것은 이미 알고 있다. **출력축이 실제로 몇 도 돌았는지 사람이 눈으로 봐야** 끝난다.
- 관절별 부호(±1)·영점 오프셋은 URDF 확정 후 결정한다. Phase 1은 적용하지 않는다.
