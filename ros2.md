# ROS2 AK45 상태 발행 노드 — 구현 명세서

> **합격 기준**: 이 문서 §0~§13만 읽고 `colcon build` → 동작하는 노드까지 추가 질문 없이 도달할 수 있어야 한다.
> 부록 E·F는 이력 기록이므로 구현자는 읽지 않아도 된다.
>
> **문서 버전**: v4.0 (2026-08-27)
> **Phase**: 1(읽기 전용 상태 발행) + **2(명령 수신 — §13)**. 둘은 **같은 노드 `ak45_node` 하나**다.
>
> ⚠️ **§1~§12는 Phase 1 기준으로 쓰였다.** Phase 2에서 바뀐 것(노드명·실행 파일명·클래스명, 타이머 3개,
> 파라미터 4개 추가, 진단 KeyValue 9개)은 **§13이 정본**이며 충돌 시 §13을 따른다.

## 근거 표기와 검증 상태

| 표기 | 대상 | v3.0 검증 상태 |
|---|---|---|
| `[매뉴얼 p.N]` | CubeMars AK Series Module Driver Manual V1.0.18 | **PDF 원문 대조 완료** |
| `[lib .h:행]` / `[lib .cpp:행]` | `Can-main/ak45_36_socketcan_control.{h,cpp}` | **소스 직접 대조 완료.** 행 번호 전건 확인 |
| `[main.cpp:행]` / `[demo_ramp.cpp:행]` / `[Makefile]` / `[AGENTS.md:행]` | 같은 저장소 | **대조 완료** |
| `[rsp]` | humble `robot_state_publisher` / MoveIt2 소스 | 웹 검색으로 확인. QoS 결정 근거 (§4.5) |
| `[실측]` | 이 문서 작성 중 실제로 컴파일·링크해 확인한 것 | §1.3 참조 |

---

## 0. 에이전트 작업 규칙

1. 이 문서에 적힌 **이름·값·로그 문구를 그대로** 쓴다. "더 나은 이름"으로 바꾸지 않는다.
2. `[미정]` 표시 항목을 임의로 채우지 않는다. 필요해지면 코드를 쓰지 말고 질문한다.
3. `ak45_36_socketcan_control.h` / `.cpp`는 **한 글자도 수정하지 않는다.**
4. ~~**모터 이동 명령을 보내지 않는다.**~~ → **Phase 2에서 해제됨(§13).** 단 **해제된 것은 `ak45_set_position` 하나뿐이다.**
   `set_rpm` / `set_current` / `set_duty` / `set_pos_spd` / `set_origin` / `set_current_brake` 를 직접 호출하는 코드는
   **여전히 작성하면 안 된다.** (종료 시 라이브러리 내부에서 나가는 브레이크 프레임은 예외 — §7.5)
5. 커스텀 메시지 패키지를 만들지 않는다 (§4.4).
6. 노드 코드에 `std::mutex`, `std::thread`가 등장하면 설계를 잘못 이해한 것이다 (§6.3).
7. 상수를 바꾸면 코드·주석·이 문서·`STATUS.md` **네 곳을 같은 커밋에서** 고친다 (부록 E).

---

## 1. 환경

| 항목 | 값 |
|---|---|
| ROS2 배포판 | **Humble Hawksbill** |
| OS | **Ubuntu 22.04 LTS** |
| 언어 | **rclcpp (C++17)** — 단일 노드 |
| 빌드 타입 | **ament_cmake** |
| RMW | **rmw_fastrtps_cpp** (Humble 기본). `RMW_IMPLEMENTATION`을 설정하지 않는다 |
| ROS_DOMAIN_ID | 설정하지 않음 (기본 0) |
| 워크스페이스 | `~/ros2_ws` |
| ROS 의존 | `rclcpp`, `sensor_msgs`, `diagnostic_msgs` — **모두 `ros-humble-desktop`에 포함.** 추가 설치 없음 |
| 비-ROS 의존 | `pthread` **만** (§1.3) |
| 쓰지 않는 것 | `ros2_socketcan`, `ros2_control`, `can_msgs`, MoveIt2 (§1.2) |

### 1.1 실행 대상 아키텍처

**x86_64 개발 PC에서 먼저 전부 검증하고, 그 다음 Jetson(aarch64)으로 옮긴다.** 코드에 아키텍처 의존이 없다 — `int16` 파싱은 라이브러리가 바이트 시프트로만 처리한다 `[lib .cpp:100–127]`.

Jetson으로 옮길 때 확인할 것 두 가지:

- CAN 어댑터 인식 (`ip link show can0`)
- **양쪽 RMW 구현체가 같아야 한다.** 한쪽만 cyclonedds면 토픽이 서로 안 보인다. 둘 다 기본값(fastrtps)이면 문제없다.

### 1.2 `ros2_socketcan` / `ros2_control`을 쓰지 않는 이유

- `ros2_socketcan`은 CAN 프레임을 `can_msgs/Frame` 토픽으로 올리는 브리지다. 우리는 이미 **수신 스레드 + 파싱 + 워치독까지 검증된 라이브러리**를 갖고 있다. 브리지를 끼우면 파싱을 노드에 다시 구현해야 하고, 라이브러리의 `flock` 단일 프로세스 보장(부록 B C1)과도 충돌한다.
- `ros2_control`은 정석이지만 `controller_manager` + URDF + `ros2_control` 태그 + controller yaml이 전부 딸려온다. **URDF가 미정(부록 C 10-2)이라 지금은 착수할 수 없다.**

→ Phase 3에서 MoveIt2를 붙일 때 `ros2_control` 전환을 재검토한다.

### 1.3 링크 의존을 `pthread` 하나로 줄인 근거 `[실측]`

원본 Makefile은 `-lpthread -lm`이다 `[Makefile]`. 그런데 실제로 확인해 보니:

- `.cpp`가 `<math.h>`를 include하지만 **수학 함수를 하나도 호출하지 않는다** (`clampf`는 비교 연산뿐) `[lib .cpp:8, 73–81]`
- `-lm` 없이 링크해도 성공한다 `[실측]`
- 애초에 Ubuntu 22.04는 glibc 2.35라 **libm이 libc에 병합**되어 있어 `-lm`이 no-op이다

→ `Threads::Threads`만 링크한다. `-lm`을 추가해도 해롭진 않지만 불필요한 의존이므로 넣지 않는다.

---

## 2. 패키지

### 2.1 이름

> ⚠️ **이 표는 Phase 1 시점 값이다. Phase 2에서 전부 바뀌었다 — §13.1이 정본.**

| 구분 | Phase 1 (구) | **현재 확정값 (§13.1)** |
|---|---|---|
| 패키지명 | `ak45_ros2` | `ak45_ros2` |
| `add_executable` 이름 | `ak45_state_publisher_node` | **`ak45_node`** |
| 런타임 노드명 | `ak45_state_publisher` | **`ak45_node`** |
| 클래스명 | `Ak45StatePublisher` | **`Ak45Node`** |

### 2.2 디렉토리 트리

```
~/ros2_ws/src/ak45_ros2/
├── package.xml
├── CMakeLists.txt
├── README.md
├── config/
│   └── ak45_state_publisher.yaml           ← 신규 (§4.6)
├── include/ak45_ros2/
│   └── ak45_36_socketcan_control.h         ← 원본 복사, 수정 금지
├── src/
│   ├── ak45_36_socketcan_control.cpp       ← 원본 복사, 수정 금지
│   └── ak45_state_publisher_node.cpp       ← 신규 (부록 A)
└── launch/
    └── state_publisher.launch.py           ← 신규 (§10)
```

- `msg/`, `srv/`, `action/`는 **만들지 않는다** (§4.4).
- 원본 `.cpp`가 `#include "ak45_36_socketcan_control.h"`(따옴표, 경로 없음)이므로 `include/ak45_ros2/`를 include 경로에 넣으면 그대로 컴파일된다.
- 라이브러리는 **submodule이 아니라 파일 복사**로 편입하고, 복사 시점의 원본 커밋 해시를 `README.md`에 적는다.
- `STATUS.md`는 이 패키지가 아니라 **저장소 루트**에 둔다. 프로젝트 전체 상태를 적는 문서이지 이 패키지만의 것이 아니기 때문이다.
- ~~[미정] 이 패키지를 어느 저장소에 커밋할지~~ → **확정(2026-08-27): `team-jax/Can` 저장소 안 `ak45_ros2/` 디렉토리.** `~/ros2_ws/src/ak45_ros2`는 그곳을 가리키는 **심볼릭 링크**다 (§13.1). 파일명은 §13.1 표대로 바뀌었다.

### 2.3 `package.xml` (전문)

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ak45_ros2</name>
  <version>0.1.0</version>
  <description>CubeMars AK45-36 SocketCAN state publisher for Team JAX humanoid arm</description>
  <maintainer email="team-jax@example.com">Team JAX</maintainer>
  <license>TODO</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>diagnostic_msgs</depend>

  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

| 태그 | 이유 |
|---|---|
| `rclcpp` | 노드 베이스, 타이머, 퍼블리셔 |
| `sensor_msgs` | `JointState` |
| `diagnostic_msgs` | `DiagnosticArray` / `DiagnosticStatus` / `KeyValue` |
| `launch`, `launch_ros` | launch 파일을 설치하므로 실행 시점 의존 |

- `<license>`는 `TODO`로 둔다. **`team-jax/Can`에 LICENSE 파일이 없고 README·AGENTS.md에도 표기가 없음을 확인했다** — 즉 아무도 정한 적이 없다. 임의로 MIT라고 적으면 사실이 아닌 정보가 된다 (부록 C 10-8).
- `ament_lint_auto`는 **넣지 않는다.** 컴파일 경고와 별개로 `ament_uncrustify`/`cpplint`는 **서식 규칙**이고, 원본 서식을 맞추려면 작업 규칙 3을 어기게 된다.

### 2.4 `CMakeLists.txt` (전문)

```cmake
cmake_minimum_required(VERSION 3.8)
project(ak45_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Threads REQUIRED)

add_executable(ak45_state_publisher_node
  src/ak45_state_publisher_node.cpp
  src/ak45_36_socketcan_control.cpp
)

# -Wall -Wextra: 원본 Makefile이 이미 쓰는 옵션이라 원본도 통과가 보증된다.
# -Wpedantic: 원본 Makefile에 없어 보증되지 않으므로 우리 노드 파일에만 적용한다.
target_compile_options(ak45_state_publisher_node PRIVATE -Wall -Wextra)
set_source_files_properties(src/ak45_state_publisher_node.cpp
  PROPERTIES COMPILE_OPTIONS "-Wpedantic")

target_include_directories(ak45_state_publisher_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ak45_ros2>
)

ament_target_dependencies(ak45_state_publisher_node
  rclcpp
  sensor_msgs
  diagnostic_msgs
)

# 라이브러리가 수신 pthread를 쓴다. -lm은 불필요 (§1.3)
target_link_libraries(ak45_state_publisher_node Threads::Threads)

install(TARGETS ak45_state_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

**경고 옵션을 이렇게 나눈 근거**: GCC 13에서 `-std=c++17 -Wall -Wextra -Wpedantic`으로 원본 라이브러리를 컴파일하면 경고 0개다 `[실측]`. 그러나 **타깃은 Ubuntu 22.04(GCC 11/12)이고 GCC 버전이 다르면 경고 집합도 다르다.** 원본 Makefile이 실제로 쓰는 옵션(`-Wall -Wextra` `[Makefile]`)만 원본에 적용하고, 보증 없는 `-Wpedantic`은 우리 파일에만 거는 것이 안전하다. 만약 22.04에서 원본에 경고가 뜨면, **원본을 고치지 말고** 이 문서 부록 E에 기록한 뒤 해당 파일의 경고 옵션을 낮춘다.

### 2.5 `README.md`에 반드시 적을 5가지

1. 복사해 온 `team-jax/Can` 원본 **커밋 해시**
2. **CAN 인터페이스는 `can0` 고정.** 바꾸려면 `include/ak45_ros2/ak45_36_socketcan_control.h`의 `CAN_INTERFACE`를 수정하고 재빌드해야 한다 (부록 B C4)
3. 실행 전 준비 명령 (§11.1)
4. **읽기 전용 노드**이며 모터를 움직이지 않는다. 단 종료 시 브레이크 0A 프레임 6개가 나간다 (§7.5)
5. `joint_names`에 **중복 이름을 넣으면 안 된다.** 노드는 검증하지 않고, `robot_state_publisher`가 마지막 값으로 덮어써서 TF가 **조용히** 어긋난다

---

## 3. 노드와 토픽 매트릭스

### 3.1 `ak45_state_publisher`

| 항목 | 값 |
|---|---|
| 네임스페이스 | **`/` (루트)** → 완전 이름 `/ak45_state_publisher` |
| 책임 (한 줄) | AK45 모터 6개의 CAN 피드백을 폴링해 `/joint_states`와 `/diagnostics`로 발행한다. 명령은 보내지 않는다. |
| pub | `/joint_states`, `/diagnostics` |
| sub | **없음** |
| 서비스 / 액션 | **없음** (파라미터 서비스는 rclcpp 자동 생성) |
| TF broadcast | **없음** (§5.2) |
| 노드가 만드는 스레드 | **0개** (§6.1) |

이 패키지의 노드는 **이것 하나뿐이다.**

### 3.2 매트릭스

| 인터페이스 | `ak45_state_publisher` | `robot_state_publisher` | `diagnostic_aggregator` |
|---|---|---|---|
| `/joint_states` (`sensor_msgs/msg/JointState`) | **pub** | sub (예정, Phase 1에선 미실행) | — |
| `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`) | **pub** | — | sub (예정, 미실행) |
| `/parameter_events`, `/rosout` | pub (rclcpp 자동) | — | — |

**고아 토픽 — Phase 1에서는 이게 정상이다.**

`/joint_states`도 `/diagnostics`도 구독자가 없다. `robot_state_publisher`는 URDF가 확정돼야 띄울 수 있고(부록 C 10-2), `diagnostic_aggregator`는 분석 규칙 yaml이 필요해 Phase 1 범위 밖이다.

→ **`rqt_graph`에서 이 노드의 화살표 2개가 아무 데도 닿지 않는 것이 기대 상태다** (§11.3 V13). "끊긴 연결 없어야 함"이라는 일반 규칙을 그대로 적용하면 정상을 결함으로 오판한다.

---

## 4. 인터페이스 계약

> 추측 금지 구역. 여기 적힌 이름·타입·QoS·단위를 그대로 쓴다.

### 4.1 `/joint_states`

| 항목 | 값 |
|---|---|
| 생성 시 이름 | `joint_states` — **상대 이름.** 앞에 `/`를 붙이지 않는다. 네임스페이스가 없으므로 결과는 `/joint_states` |
| 타입 | `sensor_msgs/msg/JointState` |
| 발행 주기 | **50.0 Hz** (파라미터 `publish_rate_hz`). 주기 고정, 이벤트 기반 아님 |
| Reliability / Durability / History / Depth | **RELIABLE / VOLATILE / KEEP_LAST / 10** |
| Lifespan·Deadline·Liveliness | 전부 기본값. 명시 설정하지 않음 |
| 생성 코드 | `rclcpp::QoS(rclcpp::KeepLast(10))` |

| 필드 | 의미 | 단위 | 범위 | 비고 |
|---|---|---|---|---|
| `header.stamp` | 발행 시각 | — | — | `this->now()` (§5.3) |
| `header.frame_id` | **항상 빈 문자열 `""`** | — | — | 파라미터로 만들지 않는다 (§4.5) |
| `name[k]` | 관절 이름 | — | `joint_names_` 중 하나 | §4.3 인덱스 규칙 |
| `position[k]` | 관절각 | **rad** | `±55.85` (= ±3200° 하드웨어 범위 `[매뉴얼 p.44]`) | 다회전 언랩 없음 |
| `velocity` | — | — | **항상 빈 배열 `[]`** | NPP 미확정 (§7.4) |
| `effort` | — | — | **항상 빈 배열 `[]`** | KT 미확정 (§7.4) |

### 4.2 `/diagnostics`

| 항목 | 값 |
|---|---|
| 생성 시 이름 | `diagnostics` (상대 이름) → `/diagnostics` |
| 타입 | `diagnostic_msgs/msg/DiagnosticArray` |
| 발행 주기 | **2.0 Hz** (파라미터 `diagnostics_rate_hz`) |
| QoS | `/joint_states`와 동일 |

`status` 배열은 **길이가 항상 정확히 6.** 피드백이 없는 모터도 포함한다(그게 진단의 목적). 순서는 §4.3 고정.

| 필드 | 값 |
|---|---|
| `name` | `"ak45/" + joint_names_[i]` (예: `ak45/ak45_1`) |
| `hardware_id` | `snprintf("ak45_id_0x%02X", id)` (예: `ak45_id_0x01`) |
| `level` | §8.2 판정표 |
| `message` | §8.2 판정표 |
| `values` | KeyValue 6개, §4.4 |

### 4.3 배열 인덱스 순서 규칙

```
index : 0     1     2     3     4     5
CAN ID: 0x01  0x02  0x03  0x04  0x05  0x06
joint : joint_names_[0] ... joint_names_[5]
```

- `/diagnostics`의 `status[]`는 이 순서를 그대로, **항상 6개** 유지한다 → 인덱스 접근이 안전하다.
- `/joint_states`의 `name[]`/`position[]`은 순서는 유지하지만 **`valid == 0`인 모터가 빠진다** → **배열 길이가 0~6으로 변하고 인덱스가 고정이 아니다.**
  → **소비자는 반드시 `name`으로 매칭해야 한다.** `position[2]`가 3번 모터라고 가정하는 코드를 쓰면 안 된다. `robot_state_publisher`는 이름 매칭이라 문제없다.

### 4.4 `/diagnostics` KeyValue — 키 이름 고정, 6개 전부 항상 포함

| key | 생성 방법 | 단위 |
|---|---|---|
| `position_deg` | `fmt2(st.position_deg)` | deg (변환 전 원본) |
| `speed_erpm` | `fmt2(st.speed_erpm)` | **ERPM** (전기적 RPM, §7.4) |
| `current_a` | `fmt2(st.current_a)` | A |
| `temperature_c` | `std::to_string(static_cast<int>(st.temperature_c))` | ℃ |
| `error_code` | `std::to_string(static_cast<int>(st.error_code))` | 0~7 |
| `watchdog_ok` | `wd_ok ? "1" : "0"` | — |

`fmt2()` 구현: `char buf[32]; std::snprintf(buf, sizeof(buf), "%.2f", v); return std::string(buf);`
`std::to_string(double)`은 로케일·자릿수 문제가 있어 쓰지 않는다.

**커스텀 메시지를 만들지 않는 이유**: 발행할 데이터가 (a) 관절각 → `JointState`, (b) 모터 상태·에러 → `DiagnosticArray`로 **전부 표준 메시지에 들어간다.** 커스텀 메시지를 만들면 `rosidl` 빌드 의존이 붙고 `robot_state_publisher`·`rqt_robot_monitor` 같은 기성 도구를 못 쓴다. ERPM·온도·에러코드처럼 표준 필드에 대응이 없는 값을 `DiagnosticStatus.values`에 넣는 것이 표준 관례다.

### 4.5 QoS를 RELIABLE로 고정한 이유 (BEST_EFFORT로 바꾸지 말 것)

- humble `robot_state_publisher`는 `joint_states`를 `rclcpp::SensorDataQoS()`로 구독한다 = **BEST_EFFORT** `[rsp]`
- MoveIt2 `current_state_monitor`는 SensorDataQoS를 쓰지 않아, joint_states가 BEST_EFFORT로 발행되면 **구독하지 못한다**는 이슈가 보고돼 있다 `[rsp]`

DDS 호환 규칙: **RELIABLE pub → BEST_EFFORT sub 는 연결됨. BEST_EFFORT pub → RELIABLE sub 는 연결 안 됨.**
→ RELIABLE로 발행하면 양쪽 모두와 붙는다. `SensorDataQoS()`로 바꾸면 나중에 MoveIt2를 붙일 때 원인을 못 찾는 버그가 된다.

**`frame_id` 파라미터를 삭제한 이유 (v2.1 → v3.0 변경)**: `JointState`와 `DiagnosticArray`의 `header.frame_id`는 소비자(`robot_state_publisher`, `diagnostic_aggregator`)가 **읽지 않는다.** 즉 아무 효과가 없는 파라미터였다. 최소 복잡도 원칙에 따라 없애고, 두 메시지 모두 `frame_id`를 빈 문자열로 둔다(기본 생성자가 이미 `""`이므로 대입 코드조차 필요 없다). 필요해지면 그때 추가한다.

### 4.6 파라미터

전부 `ak45_state_publisher` 소속. **런타임 변경 불가** — `add_on_set_parameters_callback`을 구현하지 않는다.

| 이름 | 타입 | 기본값 | 허용 범위 | 위반 시 |
|---|---|---|---|---|
| `publish_rate_hz` | `double` | `50.0` | `> 0.0` && `<= 500.0` | FATAL 후 `std::runtime_error` |
| `diagnostics_rate_hz` | `double` | `2.0` | `> 0.0` && `<= 50.0` | FATAL 후 `std::runtime_error` |
| `joint_names` | `string[]` | `["ak45_1", ..., "ak45_6"]` | **길이가 정확히 6** | FATAL 후 `std::runtime_error` |

만들지 않는 파라미터:

| 안 만드는 것 | 이유 |
|---|---|
| `can_interface` | `CAN_INTERFACE`가 `#define "can0"` **컴파일 타임 상수**다 `[lib .h:15]`. 파라미터를 만들면 값을 넣어도 무시되는 가짜 설정이 된다 (부록 B C4) |
| `frame_id` | 소비자가 읽지 않는다 (§4.5) |
| `motor_ids` | 라이브러리 상수 `CONTROLLER_ID_1~6`과 1:1이며 변경 금지 |
| `watchdog_timeout_ms` | 라이브러리 `#define WATCHDOG_TIMEOUT_MS 200` |
| 관절별 부호(±1) / 영점 오프셋 | URDF 미확정. 지금 만들면 값이 전부 추측(±1, 0.0)이라 "설정은 있는데 아무도 안 채운 파라미터"가 된다 (§5.2) |

`publish_rate_hz` 상한 500 근거: 서보 모드 피드백 업로드 주파수 상한이 500Hz다 `[매뉴얼 p.44]`. 그보다 빠르게 폴링해도 같은 값을 중복 발행할 뿐이다.

`joint_names` 중복 이름은 **검증하지 않는다.** 검증 코드 대신 README 경고로 처리한다 — 최소 복잡도.

#### `config/ak45_state_publisher.yaml` (전문)

```yaml
# ak45_ros2 / ak45_state_publisher 파라미터
# '/**' 와일드카드: 노드명·네임스페이스가 바뀌어도 그대로 적용된다.
/**:
  ros__parameters:
    # 상태 발행 주기(Hz). 0 < x <= 500. 피드백 업로드 상한 500Hz [매뉴얼 p.44]
    publish_rate_hz: 50.0

    # 진단 발행 주기(Hz). 0 < x <= 50. 표준 진단 도구 관례는 1~2Hz (§6.2)
    diagnostics_rate_hz: 2.0

    # AK45 CAN ID 0x01~0x06 순서와 1:1 대응. 정확히 6개. 중복 이름 금지.
    # URDF 확정 전 임시값 (부록 C 10-2)
    joint_names:
      - "ak45_1"
      - "ak45_2"
      - "ak45_3"
      - "ak45_4"
      - "ak45_5"
      - "ak45_6"
```

#### 파라미터 선언 시 함정

`declare_parameter<std::vector<std::string>>("joint_names", {"a","b"})` 처럼 중괄호 리스트를 직접 넘기면 템플릿 인자 추론이 실패할 수 있다. **기본값 변수를 먼저 만들어 넘긴다.**

```cpp
const std::vector<std::string> default_joint_names = {
  "ak45_1", "ak45_2", "ak45_3", "ak45_4", "ak45_5", "ak45_6"};
joint_names_ = this->declare_parameter<std::vector<std::string>>(
                 "joint_names", default_joint_names);
```

---

## 5. 좌표계 / 시간

### 5.1 frame_id

이 노드는 **어떤 프레임 이름도 쓰지 않는다.** 두 메시지의 `header.frame_id`는 빈 문자열이다 (§4.5).

### 5.2 TF

**이 노드는 TF broadcaster가 아니다.** static도 dynamic도 없다. `package.xml`에 `tf2` 의존이 없는 것이 정상이다. TF 트리는 나중에 `robot_state_publisher`가 URDF + `/joint_states`로 만든다.

→ **URDF가 미정이므로 TF 트리를 이 문서에 그릴 수 없다** (부록 C 10-2).
→ Phase 1 검증에 `ros2 run tf2_tools view_frames`를 **포함하지 않는다** (아무 프레임도 안 나오는 게 정상).

**관절 회전 방향(부호) — [미정]**: 모터 엔코더의 + 방향과 URDF joint `axis`의 + 방향이 반대인 관절이 있으면, **값은 정상인데 RViz에서 그 관절만 반대로 움직인다.** Phase 1은 부호 반전·영점 오프셋을 적용하지 않는다(추측값 방지). URDF 확정 후 판단한다 (부록 C 10-11).

REP-103 준수: 각도 rad **준수**(§7.4). 길이 m·우수 좌표계는 **해당 없음** — 이 노드는 관절각 스칼라만 발행하고 벡터·자세를 만들지 않는다.

### 5.3 시간

| 항목 | 값 |
|---|---|
| `header.stamp` | `this->now()` — **발행 시각** |
| 센서 타임스탬프 | **쓰지 않는다** |
| `use_sim_time` | **설정하지 않는다** |
| 타이머 클럭 | `create_wall_timer()` — 시스템 steady clock |

**센서 타임스탬프를 안 쓰는 이유**: `MotorState`에 `struct timespec last_rx`(CLOCK_MONOTONIC)가 있어 "언제 받았는지"는 안다 `[lib .h]`. 그런데 CLOCK_MONOTONIC은 기준점이 부팅 시각인 임의값이라 ROS 시각(에폭 기준)으로 바꾸려면 두 클럭의 오프셋을 따로 추정해야 한다. 50Hz 피드백에서 폴링 지연은 최대 20ms이고 Phase 1 목적에 이 오차는 문제가 안 된다. → `this->now()`로 통일한다.

**`use_sim_time`을 금지하는 이유 (조용한 함정)**: `this->now()`는 노드 클럭을 쓰므로 `use_sim_time:=true`면 `/clock`을 따르지만, `create_wall_timer()`는 **`use_sim_time`을 무시하고 항상 시스템 시각으로 발화한다.** `/clock` 발행자가 없는데 `use_sim_time:=true`를 켜면 `this->now()`가 **0을 반환** → `header.stamp = 0` → `robot_state_publisher`/`tf2`가 "너무 오래된 데이터"로 버린다. **노드 로그에는 아무 에러도 안 찍힌다.** 실기 하드웨어를 읽는 노드라 시뮬 시각을 쓸 이유가 애초에 없다.

---

## 6. 실행 모델

### 6.1 Executor

| 항목 | 값 |
|---|---|
| Executor | **SingleThreadedExecutor** — `rclcpp::spin(node)` 기본값 |
| 콜백 그룹 | **기본 그룹(MutuallyExclusive) 하나만.** `create_callback_group`을 호출하지 않는다 |
| 노드가 만드는 스레드 | **0개** |

근거: 수신은 라이브러리 내부 pthread가 처리하고, `ak45_get_state()`는 mutex만 잡고 즉시 값을 복사해 반환한다 `[lib .cpp:131–156, 392–403]`. **타이머 콜백에서 폴링해도 executor가 막히지 않는다** (부록 B C2).

### 6.2 타이머와 실행 시간 예산

| 타이머 | 주기 | 예산 | 하는 일 |
|---|---|---|---|
| `state_timer_` | 20 ms (50 Hz) | **< 1 ms** | `ak45_get_state()` 6회 + 메시지 1개 발행 |
| `diag_timer_` | 500 ms (2 Hz) | **< 2 ms** | `ak45_get_state()` 6회 + `ak45_is_watchdog_ok()` 6회 + `snprintf` 36회 + 발행 |

두 타이머가 같은 MutuallyExclusive 그룹이라 직렬 실행된다. `diag_timer_`가 20ms를 넘기면 `state_timer_`가 밀리는데, 예산 2ms는 20ms 대비 10%로 여유가 충분하다.

**진단을 2Hz로 분리한 이유**: `/diagnostics`는 `diagnostic_aggregator` 같은 표준 도구가 소비하며 관례가 1~2Hz다. 50Hz로 쏘면 모터 6개 × KeyValue 6개를 **초당 50번 직렬화**해 대역·CPU를 낭비하고 `ros2 topic echo`로 사람이 읽을 수도 없다. 타이머 2개는 상태 공유가 없어 복잡도가 사실상 늘지 않는다.

### 6.3 공유 변수 보호

| 데이터 | 접근 | 보호 |
|---|---|---|
| `joint_names_`, `publish_rate_hz_`, `diagnostics_rate_hz_` | 생성자에서 쓰기 1회, 콜백에서 **읽기만** | **불필요.** 런타임 변경이 없으므로 쓰기 경쟁이 없다 |
| `MotorState` | 라이브러리 내부 스레드가 쓰고 콜백이 읽음 | **라이브러리 내부 mutex가 처리** `[lib .cpp:392–403]`. `ak45_get_state()`가 값 복사를 반환하므로 노드는 잠금 불필요 |
| `can_ready_` | 생성자 쓰기, 소멸자 읽기·쓰기 | 콜백에서 건드리지 않는다. 소멸자는 spin 종료 후 실행 |

→ **노드 코드에 `std::mutex`가 등장하면 설계를 잘못 이해한 것이다.**

동기 서비스 호출 데드락 위험: **구조적으로 없다.** 이 노드는 서비스 클라이언트를 만들지 않는다.

---

## 7. 하드웨어 연동

### 7.1 물리 인터페이스

| 항목 | 값 | 근거 |
|---|---|---|
| 인터페이스 | SocketCAN | — |
| 디바이스 | **`can0` 고정** (컴파일 타임 상수) | `[lib .h:15]` |
| 비트레이트 | **1 Mbps** | `[매뉴얼 p.8]` |
| 프레임 | CAN 2.0B **Extended (29bit)**, **big-endian** | `[매뉴얼 p.35–37]` |
| 어댑터 | CANable 계열 | 프로젝트 현황 |
| 모드 | **Servo 모드** (MIT 아님) | 프로젝트 결정 |

### 7.2 CAN ID 규칙

| 방향 | ID 계산 | candump 표시 (Extended = 8자리) |
|---|---|---|
| 명령 (노드 → 모터) | `(control_mode << 8) \| controller_id` `[매뉴얼 p.36]` | `0x000000MM`~`0x000006MM` |
| 피드백 (모터 → 노드) | `(0x29 << 8) \| controller_id`, DLC 8 `[매뉴얼 p.44]` | `0x00002901`~`0x00002906` |

`control_mode`: `0`=Duty, `1`=Current, `2`=CurrentBrake, `3`=RPM, `4`=Position, `5`=SetOrigin, `6`=PosSpd `[매뉴얼 p.36]`

**Phase 1에서 명령 프레임은 실행 중 하나도 나가지 않는다** (종료 시 예외 — §7.5).

### 7.3 피드백 프레임 포맷

CAN ID `0x2900 | id`, DLC 8, big-endian. **파싱은 라이브러리가 한다** `[lib .cpp:100–127]`. 노드는 `MotorState`를 받아 쓸 뿐이다.

| 오프셋 | 타입 | 스케일 | 물리량 | 범위 | 근거 |
|---|---|---|---|---|---|
| `data[0..1]` | `int16` | `× 0.1` | 위치 (deg) | ±3200° | `[매뉴얼 p.44]` |
| `data[2..3]` | `int16` | `× 10` | 속도 (ERPM) | ±320000 | `[매뉴얼 p.44]` |
| `data[4..5]` | `int16` | `× 0.01` | 전류 (A) | ±60 | `[매뉴얼 p.44]` |
| `data[6]` | `int8` | `× 1` | 온도 (℃) | −20~127 | **오프셋 없음.** MIT 모드의 −40과 다르다 `[매뉴얼 p.44–45]` |
| `data[7]` | `uint8` | — | 에러코드 | 0~7 | `[매뉴얼 p.45]` |

### 7.4 값 변환 (여기 적힌 식만 사용)

**위치 (deg → rad) — 이것만 변환한다:**

```cpp
js.position.push_back(static_cast<double>(st.position_deg) * M_PI / 180.0);
```

프레임부터 최종 출력까지 전 구간 숫자 예시:

```
CAN 프레임: ID=0x00002901  DLC=8
data = 07 08 | 00 64 | 00 C8 | 19 | 00

data[0..1] = 0x0708 = int16 1800  → position_deg  = 1800 × 0.1  = 180.0 deg
data[2..3] = 0x0064 = int16  100  → speed_erpm    =  100 × 10   = 1000.0 ERPM
data[4..5] = 0x00C8 = int16  200  → current_a     =  200 × 0.01 = 2.00 A
data[6]    = 0x19   = int8    25  → temperature_c = 25 ℃
data[7]    = 0x00                 → error_code    = 0

→ /joint_states : name = ["ak45_1"], position = [3.14159265358979]
→ /diagnostics  : level 0(OK), message "정상",
                  position_deg="180.00", speed_erpm="1000.00", current_a="2.00",
                  temperature_c="25", error_code="0", watchdog_ok="1"
```

추가 검산:

| raw `data[0..1]` | `position_deg` | `js.position` |
|---|---|---|
| `0x07 0x08` (1800) | `180.0` | `3.14159265` |
| `0xFC 0x18` (−1000) | `−100.0` | `−1.74532925` |
| `0x00 0x00` (0) | `0.0` | `0.0` (단 `valid==1`일 때만 발행) |

**속도·토크는 변환하지 않는다:**

| 필드 | 왜 못 채우는가 |
|---|---|
| `velocity` | `speed_erpm`은 **전기적 RPM**이다. 출력축 rad/s로 바꾸려면 극쌍수 NPP가 필요하고 식은 `rad/s = ERPM / NPP / 36 × 2π / 60` 형태가 된다. **매뉴얼 V1.0.18 전문에 AK45-36의 NPP가 없다**(확인 완료). 추측값을 넣지 않는다. 원본 ERPM은 `/diagnostics`에 실린다 |
| `effort` | 전류→토크는 `Torque = Iq × KT`인데 매뉴얼은 식만 주고 **AK45-36의 KT를 제공하지 않는다** `[매뉴얼 p.35]` |

참고: 매뉴얼 MIT 파라미터 표는 AK45-36의 출력축 속도 ±6.0 rad/s, 토크 ±34 N·m를 명시한다 `[매뉴얼 p.63]`. **상한값이지 변환식이 아니다.** 이 숫자로 스케일을 역산하지 말 것.

**감속비 36:1 처리 — [미정, V3으로 판별]**

매뉴얼 전문을 확인했으나 **위치 피드백이 출력축 기준인지 모터축 기준인지 명시한 문장이 없다.** 모터축이면 출력축 각도는 `/36`을 해야 한다.

다만 아래 둘은 소스로 확정이다:

1. **명령과 피드백은 같은 축 기준이다.** `demo_ramp.cpp`가 피드백 `s.position_deg`를 램프 시작값으로 잡고 `[demo_ramp.cpp:105]`, 같은 변수를 `ak45_set_position()`에 넣는다 `[demo_ramp.cpp:143]`. 단위가 다르면 이 코드는 애초에 동작하지 않는다.
2. 라이브러리 주석도 `SOFT_LIMIT_POS_DEG 360.0f  // ±1회전` `[lib .h:29]`으로 360° = 1회전을 전제한다.

→ 남은 질문은 "그 1회전이 출력축이냐 모터축이냐" 하나뿐이며 **§11.4 V3 절차 1회로 종결된다.** 판별 결과를 이 문서 §7.4와 코드 주석에 **같은 커밋으로** 기록한다.

### 7.5 종료 시 CAN 프레임이 나간다

`ak45_close()`는 첫 줄에서 `ak45_emergency_stop()`을 호출하고, 이는 모터 6개 각각에 **Current Brake 0A 프레임**을 보낸다 `[lib .cpp:236, 376–389]`.
→ 종료 시 `candump can0`에 `0x00000201`~`0x00000206` 6개가 보이는 것은 **정상**이다. 0A라 모터는 움직이지 않는다.
→ V7("명령 프레임 없음")은 **실행 중**을 보는 것이고, 종료 시는 V8이 따로 있다.

### 7.6 피드백은 자동으로 오지 않는다 (가장 흔한 오진 원인)

서보 모드 피드백은 **주기 업로드 방식**이고, <cite index="22-1">업로드 주파수는 1~500Hz로 설정할 수 있으며 업로드 바이트는 8바이트다</cite>. 이 주파수는 **CubeMarsTool → Application Functions → "Send status over CAN" 체크 + Rate(Hz)** 에서 켜야 한다 `[매뉴얼 p.23]`.

**Rate = 0이면 `/joint_states`가 영원히 빈 배열이다.** → §11.1 준비 3번을 먼저 확인.

### 7.7 라이브러리가 stderr로 직접 출력한다

`parse_feedback()`은 `error_code != 0`인 프레임을 받을 때마다 `fprintf(stderr, ...)`로 경고를 찍는다 `[lib .cpp:124–127]`. 피드백이 50Hz면 **초당 50줄**이 나온다.
→ 노드 버그가 아니다. **노드가 같은 내용을 또 찍어 중복시키지 말 것** (§9.2).

---

## 8. 로직 / 상태 전이

### 8.1 노드 생애주기

| 현재 | 이벤트 | 다음 | 부수효과 |
|---|---|---|---|
| (시작) | `main()` 진입 | `INIT` | `rclcpp::init()` |
| `INIT` | 파라미터 검증 실패 | `FAILED` | FATAL → `throw` → **`ak45_init()` 미호출**, 종료 코드 1 |
| `INIT` | `ak45_init() != 0` | `FAILED` | FATAL 3줄 → `throw` → 종료 코드 1 |
| `INIT` | `ak45_init() == 0` | `RUNNING` | `can_ready_ = true`, 퍼블리셔·타이머 2개 생성, INFO 1줄 |
| `RUNNING` | `state_timer_` (20ms) | `RUNNING` | `/joint_states` 발행 (§8.3) |
| `RUNNING` | `diag_timer_` (500ms) | `RUNNING` | `/diagnostics` 발행 (§8.2) |
| `RUNNING` | SIGINT | `SHUTDOWN` | `spin()` 반환 → 소멸자 → `ak45_close()`(브레이크 0A 6프레임) → `rclcpp::shutdown()` |

### 8.2 모터별 진단 판정 (위에서부터 **먼저 걸리는 것 하나만**)

| 순위 | 조건 | level | message |
|---|---|---|---|
| 1 | `st.valid == 0` | `WARN` (1) | `"피드백 수신 없음 (CubeMarsTool의 Send status over CAN Rate(Hz) 확인)"` |
| 2 | `wd_ok == false` → 즉 `(now − last_rx) >= 200 ms` | `ERROR` (2) | `"피드백 끊김: 200ms 초과"` |
| 3 | `st.error_code != 0` | `ERROR` (2) | `ak45_error_str(st.error_code)` 반환 문자열 |
| 4 | 그 외 | `OK` (0) | `"정상"` |

**순위를 이 순서로 고정하는 이유**: `ak45_is_watchdog_ok()`는 `valid == 0`일 때도 `0`을 반환한다 `[lib .cpp:166,169]`. 워치독을 먼저 판정하면 **한 번도 연결된 적 없는 모터가 "끊김(ERROR)"으로 표시**되어 원인 진단이 어긋난다. `valid`를 먼저 보면 "아직 안 붙음(WARN)"과 "붙었다가 끊김(ERROR)"이 구분된다.

**경계값 주의**: 판정식이 `elapsed < WATCHDOG_TIMEOUT_MS` 이므로 `[lib .cpp:169]` **정확히 200ms는 실패로 친다.**

에러 문자열은 `ak45_error_str()`이 이미 한국어로 반환한다 `[lib .cpp:405–418]`: 정상 / 모터 과열 / 과전류 / 과전압 / 저전압 / 엔코더 고장 / MOSFET 과열 / 모터 스톨. **노드에서 다시 정의하지 말 것.**

level은 `diagnostic_msgs::msg::DiagnosticStatus::OK` / `::WARN` / `::ERROR` 상수를 쓴다. **숫자 리터럴 0/1/2 금지.**

### 8.3 `/joint_states` 포함 조건

```
for i in 0..5:
    st = ak45_get_state(kMotorIds[i])
    if st.valid != 0:          # ← 유일한 분기 조건
        name.push_back(joint_names_[i])
        position.push_back(st.position_deg * M_PI / 180.0)
```

**`valid == 0`인 모터를 넣지 않는 이유**: 피드백을 한 번도 못 받은 모터의 `position_deg`는 `memset` 결과인 `0.0f`다 `[lib .cpp:217, 394–397]`. 발행하면 **"관절이 0°에 있다"는 거짓 정보**가 되고 `robot_state_publisher`가 그 자세로 TF를 만든다. `name`과 `position`을 항상 함께 push_back 하므로 "모든 배열 크기가 같아야 한다"는 `JointState` 규칙은 지켜진다.

**주의**: `valid == 1`인데 워치독이 끊긴 모터는 **`/joint_states`에 계속 들어간다**(마지막 값). 의도된 동작이다 — 끊김은 `/diagnostics`가 level 2로 알린다. 소비자가 끊긴 관절을 무시하려면 `/diagnostics`의 `watchdog_ok`를 봐야 한다.

### 8.4 타임아웃 / 디바운스 / 재시도

| 항목 | 값 |
|---|---|
| 피드백 워치독 | **200 ms** (`WATCHDOG_TIMEOUT_MS`), 경계값 포함 실패 |
| 디바운스 | **없다.** level이 경계에서 떨릴 수 있으나 진단 주기 500ms가 완화한다 |
| 재시도 | **없다.** Phase 1은 명령을 안 보낸다 |
| CAN 재연결 | **없다.** `can0`이 죽으면 노드를 재시작한다 |
| 로그 스로틀 | 전 모터 무응답 WARN: **5000 ms** |

**명령 재송신 루프는 라이브러리에 없다** — 실제 100ms 루프는 애플리케이션에 있다 `[main.cpp:164]`, `[demo_ramp.cpp:19,163]`. Phase 1엔 영향 없지만 **Phase 2에서 반드시 노드가 구현해야 한다** (부록 B C3, 부록 D).

---

## 9. 실패 처리

### 9.1 케이스별 동작

| # | 상황 | 감지 | 동작 | 로그 |
|---|---|---|---|---|
| F1 | `can0` down / 어댑터 미인식 | `ak45_init()` `-1` | **시작 실패.** FATAL 3줄 → `throw` → 종료 코드 1 | `FATAL` |
| F2 | 다른 AK45 프로세스 실행 중 | `flock` 실패 → `-1` | F1과 동일 경로 | `FATAL` |
| F3 | 파라미터 범위/개수 위반 | 생성자 검증 | **시작 실패.** `ak45_init()` 호출 **전에** `throw` | `FATAL` |
| F4 | 모터 1개가 피드백을 한 번도 안 줌 | `st.valid == 0` | 계속 동작. `/joint_states`에서 **제외**, `/diagnostics` level 1 | 없음 (진단 토픽으로만) |
| F5 | 모터 1개 피드백 끊김 (≥200ms) | `ak45_is_watchdog_ok()==0` | 계속 동작. 마지막 값 계속 발행, level 2 | 없음 |
| F6 | **6개 전부** 무응답 | `valid_count == 0` | 계속 동작. 빈 `/joint_states` 발행 | `WARN` (5초 스로틀) |
| F7 | 모터 에러코드 발생 | `st.error_code != 0` | 계속 동작. level 2 + `ak45_error_str()` | **노드는 안 찍는다** (§7.7) |
| F8 | 위치가 int16 범위를 넘어 랩어라운드 | **감지하지 않는다** | 그대로 발행 | 없음 |
| F9 | 구독 토픽 없음 | **해당 없음** — 구독하지 않는다 | — | — |
| F10 | 정상 종료 (Ctrl+C) | `spin()` 반환 | 소멸자 → `ak45_close()` → 브레이크 0A 6프레임 → 소켓·락 해제 | 없음 |
| F11 | SIGKILL | — | 브레이크 프레임이 **안 나간다.** `flock`은 OS가 자동 해제하므로 재실행은 가능 | — |

F8을 감지하지 않는 이유: 다회전 언랩은 Phase 1 범위 밖이고, 임계값을 넣으려면 정상 범위를 URDF에서 가져와야 한다(미정).

### 9.2 로그 정책 — 노드가 남기는 로그는 이 3종류뿐

| # | 시점 | 레벨 | 문구 |
|---|---|---|---|
| L1 | 생성자 성공 | `INFO` | `"ak45_state_publisher 시작. 인터페이스=can0, 상태=%.1fHz, 진단=%.1fHz, 읽기 전용 모드"` |
| L2 | 생성자 실패 | `FATAL` | 부록 A.3의 문구 그대로 |
| L3 | 전 모터 무응답 | `WARN` (5초 스로틀) | `"6개 모터 모두 피드백 없음. CubeMarsTool의 'Send status over CAN' Rate(Hz)가 0인지 확인하세요."` |

**그 외에는 아무것도 찍지 않는다.** 특히 주기 콜백 안에서 스로틀 없는 `RCLCPP_*`를 쓰면 안 된다(50Hz면 초당 50줄).

### 9.3 `ak45_init()` 실패 시 `rclcpp::shutdown()`이 아니라 `throw`인 이유

`rclcpp::shutdown()`을 생성자에서 부르면 `main()`이 **정상 종료(코드 0)**로 끝나 스크립트에서 실패를 감지할 수 없다. `throw` → `main()`의 catch → `return 1`이어야 `echo $?`로 검증할 수 있다 (V9).

또한 생성자가 끝나지 않으면 소멸자가 호출되지 않는데, **이 시점엔 락도 소켓도 아직 안 잡힌 상태라 그게 맞다.** 라이브러리 내부에서 flock 성공 후 socket/bind 단계에서 실패하면 **락을 해제하지 않고 `-1`을 반환하지만** `[lib .cpp:190–225]`, 프로세스가 죽으면 OS가 flock을 회수하므로 문제되지 않는다 (부록 B C6).

---

## 10. launch

| 항목 | 값 |
|---|---|
| 파일 | `launch/state_publisher.launch.py` |
| launch argument | `params_file` — 기본값 `<share>/ak45_ros2/config/ak45_state_publisher.yaml` |
| remapping | **없음.** 토픽 이름을 확정했으므로 리맵할 이유가 없다 |
| 네임스페이스 | **없음** (루트) |
| `use_sim_time` | **설정하지 않는다** (§5.3) |

`params_file`을 인자로 둔 이유: URDF가 확정될 때까지 `joint_names`를 자주 바꿔 실험해야 하는데, 그때마다 launch 파일을 고치면 diff가 더러워진다. **인자는 이 하나만** 만든다.

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory('ak45_ros2'),
        'config',
        'ak45_state_publisher.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='ak45_state_publisher 파라미터 YAML 경로',
        ),
        Node(
            package='ak45_ros2',
            executable='ak45_state_publisher_node',
            name='ak45_state_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
```

`emulate_tty=True`는 로그 색상 유지와 출력 버퍼링 회피용이다. C++ 노드의 `RCLCPP_FATAL`은 stderr로 나가 보통 없어도 보이지만, launch를 통해 실행할 때 출력 순서가 뒤엉키는 걸 막아준다.

---

## 11. 검증

### 11.1 사전 준비 (사람이 먼저 할 일)

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

**3번이 안 되면 노드를 아무리 잘 짜도 `/joint_states`는 빈 배열이다.**

### 11.2 빌드·실행

```bash
cd ~/ros2_ws
colcon build --packages-select ak45_ros2
source install/setup.bash
ros2 launch ak45_ros2 state_publisher.launch.py
```

### 11.3 검증 항목

| # | 명령 / 조작 | 기대 결과 |
|---|---|---|
| **V0** | `ros2 topic list` | `/joint_states`, `/diagnostics`, `/parameter_events`, `/rosout` **4개.** 그 외 이 패키지가 만든 토픽이 보이면 명세 위반 |
| **V1** | `ros2 topic hz /joint_states` | 약 **50 Hz** |
| **V2** | `ros2 topic hz /diagnostics` | 약 **2 Hz** |
| **V3** | §11.4 절차 | 기준축 판별 (출력축 / 모터축) |
| **V4** | `ros2 topic echo /joint_states` | `velocity: []`, `effort: []`, `frame_id: ''` |
| **V5** | `ros2 topic echo /diagnostics` | `status` **정확히 6개**, 정상 모터는 `level: 0`, `message: 정상`, KeyValue 6개 |
| **V6** | 모터 1개를 CAN 버스에서 분리 (§11.4 주의) | **1초 이내**에 해당 status가 `level: 2`, `"피드백 끊김: 200ms 초과"`.<br>(워치독 200ms + 진단 주기 최대 500ms) |
| **V7** | **실행 중** `candump can0` | 명령 프레임이 **하나도 없음.** `0x000029xx` 피드백만 흐름 |
| **V8** | Ctrl+C 직후 `candump can0` | `0x00000201`~`0x00000206` **6개만** (Current Brake 0A — 정상, §7.5) |
| **V9** | 노드를 **두 번째로** 실행 → `echo $?` | 즉시 FATAL 3줄, **종료 코드 1** |
| **V10** | `ros2 param get /ak45_state_publisher joint_names` | 6개 이름 반환 |
| **V11** | `ros2 run ak45_ros2 ak45_state_publisher_node --ros-args -p "joint_names:=['a','b','c']"` → `echo $?` | FATAL `"joint_names는 정확히 6개여야 합니다..."`, **종료 코드 1**<br>※ 문자열 배열은 작은따옴표로 감싸야 파싱된다 |
| **V12** | 모터 전원을 **아예 안 켠** 상태로 실행 | `/diagnostics` 6개 전부 `level: 1`, `/joint_states` 빈 배열, **5초마다 WARN 1줄** |
| **V13** | `rqt_graph` | 노드에서 나가는 화살표 2개가 **어디에도 닿지 않는 것이 정상** (§3.2) |
| **V14** | `ros2 topic info /joint_states --verbose` | Publisher 1개, **RELIABLE / VOLATILE / KEEP_LAST (10)** |
| **V15** | `ros2 topic info /diagnostics --verbose` | V14와 동일 |

### 11.4 V3 기준축 판별 절차 (v3.0 신규 — 방법 자체를 확정)

**v2.1은 "출력축을 손으로 90° 돌린다"고 했는데, 이 방법이 실행 불가능할 수 있다.** AK45-36은 36:1 감속기가 달려 있어 출력축을 손으로 역구동하기 어렵거나 아예 불가능할 수 있다. 아래 순서로 시도한다.

**방법 A — 손으로 역구동 (가능하면 가장 안전)**
1. 모터 24V 전원을 끈다(무여자). CAN·피드백이 끊기므로 `/joint_states`는 비지만, 전원을 다시 켜면 절대 엔코더가 현재 위치를 보고한다.
2. 노드를 켠 상태에서 출력축을 손으로 잡고 돌려본다.
3. **돌아가지 않으면 방법 B로 간다.** 억지로 힘을 주지 말 것 — 감속기 파손 위험.

**방법 B — 기존 CLI로 명령 후 각도기 측정 (노드와 병행 불가)**
1. **노드를 끈다.** `flock` 때문에 노드와 `ak45_ctrl`을 동시에 실행할 수 없다 (부록 B C1).
2. 출력축에 마커를 붙이고 시작 각도를 기록한다.
3. 기존 CLI로 소각도 명령을 준다: `./ak45_ctrl 15` (작은 값부터. 저장소 `run.txt`에도 45/90° 실행 기록이 있다)
4. 출력축이 실제로 몇 도 돌았는지 각도기로 잰다.
5. CLI를 끄고 노드를 켠 뒤 `/joint_states`의 `position` 값과 비교한다.

**판정**

| 관측 | 결론 | 조치 |
|---|---|---|
| 출력축 실제 회전각 ≈ `position`(rad→deg 환산) | **출력축 기준** | §7.4 식 그대로. 변경 없음 |
| 출력축 실제 회전각 ≈ `position` / 36 | **모터축 기준** | 변환식에 `/ 36.0` 추가. §7.4·코드 주석·`STATUS.md`를 **같은 커밋에서** 갱신 |

> **V6의 "모터 1개 분리"도 주의**: 24V 전원이 공통 배선이면 모터 하나만 끄기 어려울 수 있다. 그럴 땐 **해당 모터의 CAN 커넥터만 뽑는다.** 단, 그 모터가 버스 끝단이라 종단저항이 그쪽에 있으면 버스 전체가 불안정해질 수 있으니, 가능하면 **중간에 달린 모터**를 대상으로 한다.

### 11.5 커밋

V0~V15 전부 통과 후 커밋한다.

```
feat(ros2): AK45 상태 발행 노드 추가 (읽기 전용, 검증 V0~V15 통과)
```

같은 커밋에서 `STATUS.md`를 갱신한다. **커밋 대상 저장소는 아직 미정이다** (§2.2, 부록 C 10-12).

---

## 12. 트러블슈팅 (확인 순서대로)

| 증상 | 확인 순서 |
|---|---|
| `/joint_states`가 계속 빈 배열 | ① `candump can0`에 `0x000029xx`가 오는가 → 없으면 CubeMarsTool Rate(Hz) 문제(§7.6) ② `/diagnostics`가 전부 level 1인지 확인 |
| `ak45_init() 실패` | ① `ip link show can0`가 UP인가 ② `fuser /tmp/ak45_ctrl.lock`로 다른 프로세스 확인 ③ **락 파일 자체를 지울 필요는 없다** — 프로세스 종료 시 OS가 자동 해제 |
| position 크기가 예상의 **1/36** | **모터축 기준 피드백이다.** §7.4·§11.4 참조 |
| position 값이 튀거나 부호가 갑자기 반대 | 피드백 위치는 int16 ±3200° 범위다 `[매뉴얼 p.44]`. 다회전으로 넘으면 랩어라운드한다 (F8) |
| RViz에서 관절이 **반대로** 움직임 | 부호 규약 미정 문제 (§5.2). URDF `axis`와 엔코더 + 방향 불일치 |
| RViz에서 관절이 **아예 안** 움직임 | URDF joint 이름과 `joint_names` 파라미터 일치 확인 |
| stderr에 에러 문구 도배 | 라이브러리 자체 출력 (§7.7). 모터 에러코드를 먼저 해결 |
| `header.stamp`가 0, TF 경고 | `use_sim_time`이 켜졌는지 확인 (§5.3) |
| MoveIt2가 joint_states를 못 받음 | QoS를 BEST_EFFORT/SensorDataQoS로 바꾸지 않았는지 확인 (§4.5) |
| `ros2 topic hz`가 40대 | §6.2 예산 초과가 아니라 DDS 발행 지연을 먼저 의심 |

---


---

# 13. Phase 2 — 명령 수신 (2026-08-27 구현·실기 검증 완료)

> **§13이 Phase 2의 정본이다.** §1~§12와 충돌하면 여기를 따른다.
> Phase 1 상태 발행(§4·§7·§8)은 **한 줄도 바뀌지 않았다** — 진단 KeyValue 3개 추가만 예외(§13.6).

## 13.1 확정된 이름 (Phase 1에서 변경됨)

| 구분 | Phase 1 (구) | **확정값** |
|---|---|---|
| 패키지명 | `ak45_ros2` | `ak45_ros2` (동일) |
| `add_executable` | `ak45_state_publisher_node` | **`ak45_node`** |
| 런타임 노드명 | `ak45_state_publisher` | **`ak45_node`** |
| 클래스명 | `Ak45StatePublisher` | **`Ak45Node`** |
| 소스 파일 | `src/ak45_state_publisher_node.cpp` | **`src/ak45_node.cpp`** |
| launch | `launch/state_publisher.launch.py` | **`launch/ak45_node.launch.py`** |
| config | `config/ak45_state_publisher.yaml` | **`config/ak45_node.yaml`** |

**노드는 하나다.** 상태 발행과 명령 수신을 별도 실행 파일/노드로 나누지 않는다 — 라이브러리가
`flock(/tmp/ak45_ctrl.lock)`으로 프로세스당 1개만 `ak45_init()`에 성공시키기 때문이다(부록 B C1).
나중에 뜬 쪽은 반드시 죽는다(V9/P9로 실측 확인).

**저장소 위치 확정 (부록 C 10-12 종결)**: 패키지는 **`team-jax/Can` 저장소 안 `ak45_ros2/`** 에 둔다.
`~/ros2_ws/src/ak45_ros2` 는 그곳을 가리키는 **심볼릭 링크**다. 라이브러리 `.h`/`.cpp` 는 여전히
저장소 루트 원본의 **복사본**이며 수정 금지 규칙(§0 규칙 3)은 그대로다.

## 13.2 구독 토픽 계약

| 항목 | 값 |
|---|---|
| 생성 시 이름 | `ak45/command` — **상대 이름.** 앞에 `/`를 붙이지 않는다 → `/ak45/command` |
| 타입 | `sensor_msgs/msg/JointState` |
| QoS | `rclcpp::QoS(rclcpp::KeepLast(10))` — 발행 토픽과 동일(RELIABLE / VOLATILE) |
| 커스텀 메시지 | **만들지 않는다** (§4.4) |

| 필드 | 사용 |
|---|---|
| `name[]` | 관절 이름. `joint_names` 파라미터와 **문자열 매칭** |
| `position[]` | 목표 관절각, **단위 rad** |
| `velocity`, `effort` | **무시한다.** 채워져 있어도 읽지 않는다 |
| `header` | 무시한다 |

**인덱스가 아니라 이름으로 매칭하는 이유**: `/joint_states`는 `valid == 0` 모터를 빼기 때문에 배열
길이가 0~6으로 변한다(§4.3). 명령도 인덱스 기반으로 하면 같은 함정에 빠진다. 이름 매칭이면
모터 1대만 보내도 안전하다.

### 메시지 검증 — 위반 시 **그 메시지 전체**를 버리고 WARN 스로틀 5000ms

| 조건 | 로그 문구 |
|---|---|
| `name.size() != position.size()` | `"name과 position 길이가 다릅니다 (name=%zu, position=%zu). 메시지를 무시합니다."` |
| `name`이 `joint_names`에 없음 | `"알 수 없는 관절 이름: %s. 메시지를 무시합니다."` |
| `position`이 NaN/inf | `"유효하지 않은 목표값 (관절 %s). 메시지를 무시합니다."` |

구현은 **2패스**다. 패스 1에서 위 3가지를 전건 검증하고, 하나라도 걸리면 아무것도 적용하지 않고
반환한다. 패스 2에서만 관절별로 목표값을 갱신한다. **S1(각도 상한)은 메시지 전체가 아니라
그 관절 하나만 거부**하므로 패스 2에 있다.

### 13.2.1 `ak45_deg` — 도 단위 입력 헬퍼 (2026-08-27 추가)

토픽 단위는 **rad 그대로 유지한다**(REP-103, §7.4). 표준 메시지를 쓰는 이유가
"정책 노드·기성 도구가 그대로 붙는 것"인데 단위를 도로 바꾸면 그 이유가 무너진다.

대신 **사람이 CLI로 칠 때만 쓰는 환산 래퍼**를 패키지에 넣는다.

| 항목 | 값 |
|---|---|
| 경로 | `scripts/ak45_deg` (Python3, `install(PROGRAMS ...)` 로 `lib/ak45_ros2` 에 설치) |
| 실행 | `ros2 run ak45_ros2 ak45_deg 10` |
| 하는 일 | 도 → rad 환산 후 `ros2 topic pub -1` 로 **표준 토픽에 그대로 발행** |
| 하지 않는 일 | **상한 검사를 하지 않는다.** S1 판정은 노드가 단독으로 한다 |

```
ak45_deg 10                     # joint_names[1] (기본 ak45_2) 을 10도로
ak45_deg ak45_2 10              # 관절 명시
ak45_deg ak45_2 10 ak45_3 -5    # 여러 관절
ak45_deg --list                 # /joint_states 를 도 단위로 출력
```

**상한 검사를 스크립트에 넣지 않은 이유**: 넣으면 상한이 두 곳(노드 파라미터 +
스크립트 상수)에 생겨 반드시 어긋난다. 안전 판정은 노드 한 곳에만 둔다.
스크립트는 거부 여부를 모르므로 **노드 로그를 봐야 한다**는 점을 출력에 명시한다.

## 13.3 파라미터 (§4.6의 3개에 추가)

| 이름 | 타입 | 기본값 | 허용 범위 | 위반 시 |
|---|---|---|---|---|
| `command_resend_rate_hz` | `double` | `10.0` | `> 0.0` && `<= 50.0` | FATAL 후 `std::runtime_error` |
| `max_command_deg` | `double` | `15.0` | `> 0.0` && `<= 360.0` | FATAL 후 `std::runtime_error` |
| `command_timeout_sec` | `double` | `0.0` | `>= 0.0` | FATAL 후 `std::runtime_error` |
| `max_temperature_c` | `double` | `60.0` | **범위 검증 없음** | — |

- `command_resend_rate_hz` — 100ms 재송신. **값을 올릴 이유는 없다.**
- `max_command_deg` — 목표각 절대값 상한. **위치 피드백이 출력축 기준인지 모터축 기준인지 아직
  판별되지 않아서(부록 C 10-10) 보수적으로 둔 값이다.** §11.4 V3 판별 후 사람이 yaml에서 올린다.
  라이브러리 `SOFT_LIMIT_POS_DEG 360.0f`와 상한을 맞췄다.
- `command_timeout_sec` — `0.0`이면 타임아웃 없음(마지막 목표를 계속 홀딩). 0보다 크면 그 시간 동안
  새 명령이 없을 때 재송신을 멈춘다. **기본이 `0.0`인 이유**: 강화학습 정책이 끊겨도 팔이 갑자기
  힘을 놓으면 중력으로 떨어진다. 홀딩 유지가 기본이고 끊김 감지는 사람이 켠다. 판정은 관절별이
  아니라 **전역**이다(`last_command_time_` 하나).
- `max_temperature_c` — 무부하(전류 0A)에서도 26 → 53℃까지 오른 실측 이력이 있고 **원인이 미상**이라
  마진을 두고 60℃로 둔다. **범위 검증을 넣지 않은 것은 의도적이다** — 프롬프트가 범위를 주지 않았고
  §0 규칙 2("[미정] 항목을 임의로 채우지 않는다")를 따랐다.

**런타임 변경 불가**는 그대로다. `add_on_set_parameters_callback`을 구현하지 않는다.

## 13.4 실행 모델 — 타이머 3개

| 타이머 | 주기 | 하는 일 |
|---|---|---|
| `state_timer_` | 50 Hz | `/joint_states` 발행 (§A.5, 변경 없음) |
| `diag_timer_` | 2 Hz | `/diagnostics` 발행 (§A.6 + KeyValue 3개) |
| **`command_timer_`** | **10 Hz** | **`ak45_set_position()` 재송신 (신규)** |

**전부 기본 콜백 그룹(MutuallyExclusive)에 둔다.** `create_callback_group`을 호출하지 않는다.

구독 콜백과 `command_timer_`가 목표값 멤버를 공유하지만 **SingleThreadedExecutor +
MutuallyExclusive 그룹이라 동시에 실행되지 않는다.**
→ `std::mutex`, `std::thread`, `MultiThreadedExecutor` 를 **쓰지 않는다.** 필요 없다(§6.3).

이것으로 부록 D 6번("명령 타이머가 추가되면 §6 실행 모델을 재검토해야 한다")이 **종결됐다.**
재검토 결과는 "잠금도 콜백 그룹 분리도 불필요"다.

### 왜 콜백에서 바로 보내지 않고 타이머로 재송신하는가

라이브러리에는 **재송신 루프가 없다.** `ak45_set_position()` 1회 호출 = **CAN 프레임 1개**다
(부록 B C3, `[lib .cpp:319–336]`에 루프 없음). 재송신을 멈추면 모터 쪽 명령 타임아웃으로
**홀딩 토크가 풀린다** (`[demo_ramp.cpp:141–143]` 주석 근거).
→ 구독 콜백은 목표값을 멤버에 **저장만** 하고, 별도 타이머가 매 주기 다시 호출한다.
콜백에서 바로 보내면 재송신 주기가 발행자 쪽 주기에 끌려다니게 된다.

### 목표값 저장 구조

```cpp
std::array<double, kNumMotors> target_deg_{};      // deg 로 저장(라이브러리가 deg를 받는다)
std::array<bool,   kNumMotors> has_target_{};      // 전부 false. 첫 명령 전 송신 금지
std::array<bool,   kNumMotors> temp_locked_{};     // S2 래치
std::array<bool,   kNumMotors> command_active_{};  // 진단 표시용
rclcpp::Time last_command_time_;
```

`command_active_`는 프롬프트 원안에 없지만 진단 필드 `command_active`를 채우려면 필요하다.
재송신 타이머가 쓰고 진단 타이머가 읽으며, 같은 그룹이라 잠금이 필요 없다.

## 13.5 안전장치 S1~S4

| # | 조건 | 동작 | 로그 (스로틀 5000ms) |
|---|---|---|---|
| **S1** | `\|목표각(deg)\| > max_command_deg` | **그 관절의 목표값을 갱신하지 않는다.** 다른 관절은 정상 적용 | WARN `"목표각 %.1f도가 상한 %.1f도를 넘습니다. 무시합니다. (기준축 판별 전)"` |
| **S2** | `st.temperature_c >= max_temperature_c` | 그 모터를 재송신 대상에서 **영구 제외**(`temp_locked_` 래치) | ERROR `"ID 0x%02X 온도 %d도. 명령 송신을 중단합니다."` |
| **S3** | `st.valid == 0` 또는 `ak45_is_watchdog_ok() == 0` | 그 모터를 재송신 대상에서 제외(래치 아님, 복귀 가능) | 없음 |
| **S4** | `ak45_set_position()`이 `-1` 반환 | `command_active_ = false` | ERROR `"ID 0x%02X 명령 송신 실패 (에러코드 %d: %s)"` |

**S1은 클램프가 아니라 거부다.** 클램프하면 사람이 잘못 보낸 것을 모르는 채로 모터가 상한까지
움직여버린다. 거부하면 로그가 남고 이전 목표가 유지된다.

**S2는 한 번 걸리면 온도가 내려가도 자동 복귀하지 않는다. 노드를 재시작해야 한다.**
전류 0A에서 계속 오른 이력이 있어 원인이 미상이므로 보수적으로 간다.
→ 검증 중 명령 송신이 멈추면 **코드 결함이 아니라 설계대로 동작한 것**이다.

**S3이 없으면** 지금 버스에 ID 0x02밖에 없으므로 **없는 모터 5대에 100ms마다 프레임을 쏘게 된다.**

**S4**는 `error_code != 0`이면 라이브러리가 `-1`을 반환하고 프레임을 쏘지 않는 비대칭(부록 B C5)에
대응한다. 에러 문자열은 `ak45_error_str()`이 이미 한국어로 반환하므로 **노드에서 다시 정의하지 않는다.**

### 첫 명령 전에는 아무것도 보내지 않는다

`has_target_`이 전부 `false`로 초기화되고, `false`인 관절은 재송신 루프에서 건너뛴다.
목표값을 `0.0`으로 초기화해 두고 바로 송신하면 **노드를 켜는 순간 모터가 현재 위치에서 0도로 튄다.**
P1로 실측 확인했다(6초/300프레임 전부 피드백, 명령 프레임 0건).

## 13.6 진단 확장 — KeyValue 6개 → 9개

§4.4의 기존 6개는 **키 이름·순서·생성 방법 그대로** 두고 뒤에 3개를 붙인다.

| key | 생성 방법 |
|---|---|
| `target_deg` | `has_target_[i] ? fmt2(target_deg_[i]) : "none"` |
| `command_active` | `command_active_[i] ? "1" : "0"` — 재송신 중이면 `1`, S1~S4로 제외면 `0` |
| `position_error_deg` | `has_target_[i] ? fmt2(target_deg_[i] - st.position_deg) : "none"` |

`status` 배열 길이는 여전히 **항상 정확히 6**이다.

## 13.7 로그 정책 (§9.2 확장)

§9.2는 "노드가 남기는 로그는 3종류뿐"이라고 못 박았다. Phase 2에서 **5종류가 추가된다.**
전부 **스로틀 5000ms**이며 주기 콜백 안에 스로틀 없는 `RCLCPP_*`는 여전히 금지다.

| # | 레벨 | 시점 |
|---|---|---|
| L4 | WARN | 길이 불일치 (§13.2) |
| L5 | WARN | 알 수 없는 관절 이름 (§13.2) |
| L6 | WARN | 유효하지 않은 목표값 (§13.2) |
| L7 | WARN | S1 각도 상한 초과 |
| L8 | ERROR | S2 온도 / S4 송신 실패 |

L1(시작 INFO) 문구도 바뀐다 — Phase 1의 `"... 읽기 전용 모드"`는 이제 거짓이다.

## 13.8 하지 말 것 (Phase 2에서도 유효)

- 커스텀 메시지 패키지 생성 금지 (§4.4)
- `std::mutex` / `std::thread` / `MultiThreadedExecutor` 금지 (§13.4)
- **목표각 클램프 금지** — S1은 거부이지 클램프가 아니다
- `ak45_set_rpm` / `set_current` / `set_duty` / `set_pos_spd` / `set_origin` 호출 금지.
  **위치 제어(`ak45_set_position`)만 쓴다.**
- 라이브러리 `.h`/`.cpp` 수정 금지 (§0 규칙 3)
- 서비스 / 액션 추가 금지

## 13.9 실기 검증 결과 (2026-08-27, 모터 ID 0x02 1대)

| # | 항목 | 결과 |
|---|---|---|
| **P1** | 노드만 켠 상태 | ✅ 6초/**300프레임 전부 피드백**, 명령 프레임 **0건**. `target_deg="none"`, `command_active="0"` |
| **P2** | 첫 명령 (0.0 rad) | ✅ `00000402` **92프레임/9.2초 = 정확히 10Hz**, 페이로드 `00 00 00 00`. `target_deg="0.00"`, `command_active="1"`, 전류 0.00→**0.13A**(홀딩 토크 실제 인가) |
| **P3** | 값 변경 추종 (0.1745 rad) | ✅ 페이로드 `00 00 00 00` → **`00 01 86 8D`**(=99981÷10000=**9.998도**). 피드백 위치 `00 00`(0도) → **`00 63`(9.9도)**. 즉시 추종 |
| **P4** | 상한 거부 (6.28 rad) | ✅ WARN 1줄 `"목표각 359.8도가 상한 15.0도를..."`, 명령 프레임 0건, `target_deg` 이전 값 유지 |
| **P5** | 없는 모터 (`ak45_1`) | ✅ 목표는 저장(`target_deg="0.00"`)되지만 S3으로 **송신 0건**, `command_active="0"` |
| **P6** | 잘못된 이름 | ✅ WARN 1줄, 메시지 전체 무시 |
| **P7** | 길이 불일치 | ✅ WARN 1줄 `"name=2, position=1"`, 메시지 전체 무시 |
| **P8** | 종료 (SIGINT) | ✅ **브레이크 0A 프레임 6개 전부**(`0x201`~`0x206`) 확인. 종료 코드 0 |
| **P9** | 이중 실행 | ✅ 두 번째 노드 FATAL 후 **종료 코드 1**. `fuser`가 첫 노드 PID 지목 |
| **P10** | 상태 발행 | ✅ `/joint_states` **49.997Hz**, `/diagnostics` **2.000Hz**, `velocity`/`effort` 빈 배열 유지 |

**빌드**: `colcon build` 경고 **0건**(GCC 11, `-Wall -Wextra` + 노드 파일 `-Wpedantic`).

**P8이 부록 F/RUN.md 「발견 1」을 갱신한다.** 앞 세션에서는 종료 시 브레이크 프레임이 4~6개만
확인돼 "호스트에서 검증 불가"로 남았는데, 이번에는 **6개가 모두 확인됐다.** 다만 3회 실측이
아니라 1회이므로 "해결"이 아니라 "이번엔 6개 나왔다"까지가 사실이다.

### 검증 중 확인된 미해결 사항

**V3(기준축 판별)은 여전히 미실시다.** P3에서 목표 10도를 주자 피드백이 9.9도로 따라왔지만,
이것은 **명령과 피드백이 같은 축을 쓴다는 이미 알던 사실**(`[demo_ramp.cpp:105,143]`)을 재확인할
뿐 **그 축이 출력축인지 모터축인지는 말해주지 않는다.** 판별하려면 §11.4대로 **출력축이 실제로
몇 도 돌았는지 사람이 눈으로 봐야 한다.** 그때까지 `max_command_deg`는 15.0으로 둔다.

**온도**: 검증 전 52℃ → P2 후 53℃ → P3 후 **54℃**. 이동할 때마다 1℃씩 올랐다.
`max_temperature_c` 60.0까지 여유가 **6℃**다. 추가 검증 전에 온도를 먼저 본다.
# 부록 A. `ak45_node.cpp` 골격 (Phase 1 부분)

> ⚠️ **아래 골격은 Phase 1 시점의 것이다.** 클래스명은 `Ak45Node`, 파일명은 `src/ak45_node.cpp`로
> 바뀌었고 타이머가 3개·파라미터가 7개다. **Phase 2 최종 형태는 §13**을 본다.
> A.5(`onStateTimer`)와 A.6(`onDiagTimer`)의 **로직 자체는 그대로 유효**하다(A.6은 KeyValue 3개 추가).

### A.1 include

```cpp
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include "ak45_36_socketcan_control.h"
```

### A.2 클래스 정의 (이름·시그니처 고정)

```cpp
class Ak45StatePublisher : public rclcpp::Node
{
public:
  Ak45StatePublisher();
  ~Ak45StatePublisher() override;

private:
  void onStateTimer();   // 50Hz, /joint_states
  void onDiagTimer();    // 2Hz,  /diagnostics

  diagnostic_msgs::msg::DiagnosticStatus makeStatus(
    std::size_t index, uint8_t motor_id,
    const MotorState & st, bool watchdog_ok) const;

  static std::string fmt2(double v);   // 소수 2자리 (snprintf)

  static constexpr std::size_t kNumMotors = 6;
  static constexpr uint8_t kMotorIds[kNumMotors] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};

  std::vector<std::string> joint_names_;
  double publish_rate_hz_{50.0};
  double diagnostics_rate_hz_{2.0};
  bool can_ready_{false};   // ak45_init() 성공 여부. 소멸자 가드에 사용 (부록 B C7)

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
};
```

> C++17에서 `static constexpr` 데이터 멤버는 암묵적 inline이라 클래스 밖 정의가 불필요하다. 링크 에러가 나면 `.cpp` 하단에 `constexpr uint8_t Ak45StatePublisher::kMotorIds[];`를 추가한다.

### A.3 생성자 (이 순서 그대로)

```
1. rclcpp::Node("ak45_state_publisher")

2. 파라미터 선언·읽기 (§4.6의 default_joint_names 방식)
     publish_rate_hz_     = declare_parameter<double>("publish_rate_hz", 50.0);
     diagnostics_rate_hz_ = declare_parameter<double>("diagnostics_rate_hz", 2.0);
     joint_names_         = declare_parameter<std::vector<std::string>>("joint_names", default_joint_names);

3. 파라미터 검증 (실패 시 RCLCPP_FATAL 후 throw) — ak45_init() 보다 먼저
     "publish_rate_hz는 0보다 크고 500 이하여야 합니다. 입력값=%.3f"
       → throw std::runtime_error("invalid publish_rate_hz")
     "diagnostics_rate_hz는 0보다 크고 50 이하여야 합니다. 입력값=%.3f"
       → throw std::runtime_error("invalid diagnostics_rate_hz")
     "joint_names는 정확히 6개여야 합니다(AK45 ID 0x01~0x06 순서). 입력 개수=%zu"
       → throw std::runtime_error("invalid joint_names size")

4. CAN 초기화
     if (ak45_init() != 0) {
       RCLCPP_FATAL(get_logger(), "ak45_init() 실패. 다음을 확인하세요:");
       RCLCPP_FATAL(get_logger(), "  1) can0 활성화: sudo ip link set can0 up type can bitrate 1000000");
       RCLCPP_FATAL(get_logger(), "  2) 다른 AK45 프로세스 실행 여부: fuser /tmp/ak45_ctrl.lock");
       throw std::runtime_error("ak45_init failed");
     }
     can_ready_ = true;

5. 퍼블리셔 (상대 이름, 앞에 '/' 금지)
     joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(
                    "joint_states", rclcpp::QoS(rclcpp::KeepLast(10)));
     diag_pub_  = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                    "diagnostics",  rclcpp::QoS(rclcpp::KeepLast(10)));

6. 타이머 2개
     const auto to_ns = [](double hz) {
       return std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / hz));
     };
     state_timer_ = create_wall_timer(to_ns(publish_rate_hz_),
                      std::bind(&Ak45StatePublisher::onStateTimer, this));
     diag_timer_  = create_wall_timer(to_ns(diagnostics_rate_hz_),
                      std::bind(&Ak45StatePublisher::onDiagTimer, this));

7. RCLCPP_INFO(get_logger(),
       "ak45_state_publisher 시작. 인터페이스=can0, 상태=%.1fHz, 진단=%.1fHz, 읽기 전용 모드",
       publish_rate_hz_, diagnostics_rate_hz_);
```

**검증(3)을 `ak45_init()`(4)보다 먼저 두는 이유**: 파라미터가 틀렸는데 CAN을 먼저 열면, `throw` 시 소멸자가 안 불려 소켓과 `flock`이 프로세스 종료까지 남는다. **순서를 바꾸지 말 것.**

### A.4 소멸자

```cpp
Ak45StatePublisher::~Ak45StatePublisher()
{
  if (can_ready_) {
    ak45_close();   // 내부에서 브레이크 0A 6프레임 → 소켓/락 해제 (§7.5)
    can_ready_ = false;
  }
}
```

**`can_ready_` 가드는 편의가 아니라 필수다.** `ak45_close()`는 `pthread_join(g_rx_thread, NULL)`을 무조건 호출하는데 `[lib .cpp:243]`, init 실패로 스레드가 생성되지 않았다면 **초기화되지 않은 `pthread_t`를 join = 미정의 동작**이다 (부록 B C7). 절대 빼지 말 것.

### A.5 `onStateTimer()` — 50 Hz

```
sensor_msgs::msg::JointState js;
js.header.stamp = this->now();
// js.header.frame_id 는 기본값 "" 그대로 둔다 (§4.5)
// js.velocity, js.effort 도 손대지 않는다 (§7.4)

for (std::size_t i = 0; i < kNumMotors; ++i) {
    const MotorState st = ak45_get_state(kMotorIds[i]);
    if (st.valid != 0) {
        js.name.push_back(joint_names_[i]);
        js.position.push_back(static_cast<double>(st.position_deg) * M_PI / 180.0);
    }
}

joint_pub_->publish(js);
```

### A.6 `onDiagTimer()` — 2 Hz

```
diagnostic_msgs::msg::DiagnosticArray da;
da.header.stamp = this->now();
da.status.reserve(kNumMotors);

int valid_count = 0;

for (std::size_t i = 0; i < kNumMotors; ++i) {
    const uint8_t id    = kMotorIds[i];
    const MotorState st = ak45_get_state(id);
    const bool wd_ok    = (ak45_is_watchdog_ok(id) != 0);
    if (st.valid != 0) { ++valid_count; }
    da.status.push_back(makeStatus(i, id, st, wd_ok));   // 항상 6개
}

diag_pub_->publish(da);

if (valid_count == 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "6개 모터 모두 피드백 없음. CubeMarsTool의 'Send status over CAN' Rate(Hz)가 0인지 확인하세요.");
}
```

### A.7 `main()`

```cpp
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int ret = 0;
  try {
    rclcpp::spin(std::make_shared<Ak45StatePublisher>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("ak45_state_publisher"), "노드 시작 실패: %s", e.what());
    ret = 1;
  }
  rclcpp::shutdown();
  return ret;
}
```

`spin()`이 반환되면 임시 `shared_ptr`이 파괴되고 → 소멸자 → `ak45_close()`가 `rclcpp::shutdown()`보다 **먼저** 실행된다. 이 순서가 맞다.

---

# 부록 B. 라이브러리 API와 제약

파일: `ak45_36_socketcan_control.h` / `.cpp` · 링크: `pthread` (§1.3) · `extern "C"` 래핑 없음 → C++에서 그냥 include.

### B.1 Phase 1에서 쓰는 함수는 이 5개뿐

```c
int  ak45_init(void);                             // 0=성공, -1=실패
void ak45_close(void);                            // 내부에서 ak45_emergency_stop() 먼저 호출
MotorState ak45_get_state(uint8_t controller_id); // 값 복사 반환, 논블로킹
int  ak45_is_watchdog_ok(uint8_t controller_id);  // 1=정상, 0=타임아웃/미수신
const char *ak45_error_str(uint8_t code);         // 한국어 문자열 반환
```

```c
typedef struct {
    float    position_deg;   // 도(deg)
    float    speed_erpm;     // ERPM(전기적 RPM)
    float    current_a;      // A
    int8_t   temperature_c;  // ℃
    uint8_t  error_code;     // 0~7
    struct timespec last_rx; // CLOCK_MONOTONIC
    int      valid;          // 0 = 피드백을 한 번도 못 받음
} MotorState;
```

상수: `NUM_MOTORS 6`, `CONTROLLER_ID_1~6 = 0x01~0x06` `[lib .h:8–14]`, `CAN_INTERFACE "can0"` `[lib .h:15]`, `WATCHDOG_TIMEOUT_MS 200` `[lib .h:32]`, `SOFT_LIMIT_CURRENT_A 5.0f` `[lib .h:24]`, `SOFT_LIMIT_ERPM 500` `[lib .h:27]`, `SOFT_LIMIT_POS_DEG 360.0f` `[lib .h:29]`.

### B.2 설계에 영향을 주는 제약 7가지

| # | 제약 | 근거 | 대응 |
|---|---|---|---|
| **C1** | `flock(/tmp/ak45_ctrl.lock, LOCK_EX\|LOCK_NB)` — **프로세스당 1개만** `ak45_init()` 성공 | `[lib .cpp:177–188]` | 시스템 전체에 AK45 노드 1개. Phase 2도 **같은 프로세스**로 합친다. V3 방법 B에서 노드를 꺼야 하는 이유이기도 하다 |
| **C2** | 수신은 내부 pthread, `ak45_get_state()`는 mutex만 잡고 즉시 반환 | `[lib .cpp:131–156, 392–403]` | SingleThreadedExecutor로 충분. **노드에 스레드 만들지 않는다** |
| **C3** | **명령 재송신 루프가 라이브러리에 없다.** 명령 함수 1회 = CAN 프레임 1개 | `[lib .cpp:319–336 루프 없음]`, 실제 루프는 `[main.cpp:164]`, `[demo_ramp.cpp:19,163]` | Phase 1 무관. **Phase 2에서 노드가 반드시 구현** |
| **C4** | `CAN_INTERFACE`가 `#define "can0"` 컴파일 타임 상수 | `[lib .h:15, .cpp:197]` | `can_interface` 파라미터를 **만들지 않는다** |
| **C5** | `error_code != 0`이면 `set_position`/`set_rpm`/`set_current`가 `-1` 반환. `set_current_brake`/`set_duty`/`set_origin`은 **차단 없음** | `[lib .cpp:273–277, 305–309, 323–327]` | Phase 1 무관. Phase 2 설계 시 이 비대칭을 기억 |
| **C6** | `ak45_init()`이 flock 성공 후 socket/bind/pthread_create에서 실패하면 **락을 해제하지 않고 `-1` 반환** | `[lib .cpp:190–225]` (해제 코드 없음) | 노드가 `throw` → 프로세스 종료 → OS가 flock 회수. **그래서 `throw`가 맞다** (§9.3) |
| **C7** | `ak45_close()`가 `pthread_join`을 **무조건** 호출. init 실패로 스레드 미생성이면 미정의 동작 | `[lib .cpp:243]` | `can_ready_` 가드가 **필수** (부록 A.4) |

---

# 부록 C. 미확정 항목 (Phase 1 구현을 막지 않음)

| # | 항목 | 상태 / 대응 |
|---|---|---|
| 10-1 | AK45-36 극쌍수 NPP, 토크 상수 KT | 매뉴얼 전문 확인 결과 **없음** 확정 → `velocity`/`effort` 비움으로 처리 완료.<br>⚠️ **함정: 기존 CubeMarsTool 스크린샷의 NPP=21을 쓰면 안 된다.** Motor type이 `AK80_3`으로 잘못 선택된 상태의 프리셋 값이다 `[AGENTS.md:190]`. Motor type을 AK45-36으로 바꾼 뒤 Read Parameters로 실제 값을 확인할 것 |
| 10-2 | 최종 joint 이름 / URDF | 7DOF/8DOF 조율 중 → 임시 `ak45_1~6`, 파라미터로 즉시 교체. TF 트리(§5.2)도 여기 의존 |
| 10-3 | `SOFT_LIMIT_ERPM` | ~~불일치 실재 확인: `[lib .h:27]` `500`("시연용") vs `[AGENTS.md:42]` `10000`~~ → **문서-코드 불일치는 2026-08-20에 해소**(전 문서 `500`으로 통일, 2026-08-27 재확인). **남은 것은 정식값 확정뿐이고 10-1의 NPP 미확보로 막혀 있다.** 단 이 상수는 `set_rpm` 전용이라 `set_position`만 쓰는 Phase 2를 막지 않는다 |
| 10-4 | AK45-36 정격/피크 전류 | 매뉴얼은 **드라이버 보드별** 정격만 제시 (V2.1 20A/60A, V2.2 10A/30A, V1.0 mini 10A/20A) `[매뉴얼 p.8–11]`. **실물 보드의 실크 표기를 눈으로 확인**해야 확정된다.<br>※ 함께 받은 `AK40-2410-1A-A1` 문서의 `AK-DRV-6S20A_V1.00`(24V / 16–28V / 10A / 20A / 39×42mm / 14bit)은 **매뉴얼 p.11 "mini size" 보드와 스펙이 동일하고 p.11 그림에도 같은 실크(`AK-DRV-6S20A -V1.00`)가 찍혀 있다** — 즉 같은 보드의 별도 설치 문서다. 따라서 "다른 모델용이라 무관"이 아니라 **"AK45-36에 이 보드가 달렸는지가 미확인"**이 정확한 상태다 |
| 10-5 | 다회전 위치 언랩 | Phase 1은 하지 않는다 (F8) |
| 10-6 | 모터 측 Timeout(ms) 실제 설정값 | Phase 2 재송신 주기 결정에 필요 `[매뉴얼 p.23]`. Phase 1 무관 |
| 10-7 | 듀얼 엔코더 탑재 여부 | `set_origin(id, 1)`(영구 영점)은 듀얼 엔코더 모델 전용 `[매뉴얼 p.42]`. 확인 전까지 `permanent=1` 호출 금지 |
| 10-8 | 저장소 라이선스 | **LICENSE 파일 없음 확정.** README·AGENTS.md에도 표기 없음 → 팀이 정해야 `<license>`를 채울 수 있다. 그때까지 `TODO` |
| 10-10 | `position_deg`가 출력축인지 모터축인지 | 매뉴얼에 명시 없음 **확정**. 명령↔피드백 동일 축은 소스로 확정 → **§11.4 절차 1회로 종결** |
| 10-11 | 관절별 부호 규약 / 영점 오프셋 | URDF 확정 후 결정 (§5.2) |
| ~~**10-12**~~ | ~~ROS2 패키지를 어느 저장소에 둘지~~ | **종결(2026-08-27): `team-jax/Can` 안 `ak45_ros2/`.** `~/ros2_ws/src/ak45_ros2`는 심볼릭 링크 (§13.1) |

---

# 부록 D. Phase 2 예고 → **완료 (2026-08-27)**

> **이 부록은 이력이다. Phase 2는 §13으로 구현·검증이 끝났다.**
> 아래 6개 항목이 §13에서 어떻게 처리됐는지는 각 줄 끝에 적었다.

Phase 1 검증(V0~V15) 통과 후 ~~별도 사양서를 작성한다~~ → **§13으로 이 문서 안에 넣었다.** 지킬 것:

1. **C1 때문에 Phase 1 노드와 같은 프로세스여야 한다.** 별도 프로세스로 띄우면 `ak45_init()`이 실패한다. 최종 형태는 **이 노드에 명령 기능을 추가**하는 것. → ✅ **단일 노드 `ak45_node`. P9로 실측 확인**(두 번째 인스턴스 종료 코드 1)
2. **C3 — 100ms 주기 재송신 타이머 필수.** 목표값은 콜백에서 멤버에 저장만 하고, 별도 타이머가 매 주기 `ak45_set_position()`을 다시 호출한다. `[demo_ramp.cpp:141–143]` 주석이 근거: 재송신을 멈추면 모터 쪽 명령 타임아웃으로 홀딩 토크가 풀린다. → ✅ **`command_timer_` 10Hz. P2로 실측 확인**(92프레임/9.2초)
3. **C5 — 명령 함수의 `-1` 반환을 삼키지 말 것.** → ✅ **S4로 구현**(§13.5). ERROR 스로틀 5000ms + `ak45_error_str()`
4. 워치독 실패 모터는 재송신 대상에서 제외한다. → ✅ **S3으로 구현**(§13.5). `valid==0`도 함께 제외. **P5로 실측 확인**
5. 명령 인터페이스 타입 미정. **커스텀 메시지를 만들기 전에 표준 메시지로 되는지 먼저 검토.** → ✅ **표준 `sensor_msgs/msg/JointState`로 충분.** 커스텀 메시지 만들지 않음 (§13.2)
6. 명령 타이머가 추가되면 **§6의 실행 모델을 재검토해야 한다.** 명령 콜백과 명령 타이머가 목표값 멤버를 공유하므로, 그때 처음으로 잠금 또는 콜백 그룹 분리가 필요해진다. → ✅ **재검토 완료(§13.4): 둘 다 불필요.** SingleThreadedExecutor + MutuallyExclusive 기본 그룹이면 동시 실행이 없다

---

# 부록 E. 문서 동기화 규칙 및 이력

상수를 바꾸면 **네 곳을 같은 커밋에서** 고친다:
`ak45_36_socketcan_control.h`(값) → 같은 줄 주석 → `AGENTS.md` 소프트리밋 절 → 이 문서 부록 B.1·부록 C.

| 날짜 | 버전 | 변경 |
|---|---|---|
| 2026-08-20 | v0.1 | 최초 작성 |
| 2026-08-20 | v1.0 | 이름·값 확정, 빌드 파일 전문, QoS 근거 |
| 2026-08-20 | v1.1 | 검토 1회차: 진단 주기 분리, 파라미터 선언 방식, `makeStatus` 누락, level 순서 근거, 라이선스 임의 기재 삭제 |
| 2026-08-20 | v2.0 | 골격 재배치. 검토 2회차: `use_sim_time` 함정, 감속비·부호 미확정, 고아 토픽, YAML, 인덱스 가변성, V0/V13/V14/V15 |
| 2026-08-20 | v2.1 | 실물 소스 대조: C4 행번호, 워치독 경계, C6·C7 신설, NPP=21 함정, AK40 문서 판단 정정 |
| **2026-08-20** | **v3.0** | **검토 4회차(부록 F).** V3 절차 실행 가능성 문제 해결, `-lm` 제거, 경고 옵션 근거 정정, `frame_id` 파라미터 삭제, 부록 번호 정리, 미결 질문을 문서 밖으로 분리, 저장소 위치 미정 신규 |
| **2026-08-27** | **v4.0** | **§13 Phase 2 신설** — 명령 수신·재송신·S1~S4·파라미터 4개·진단 KeyValue 9개. 이름 확정(`ak45_node`), 저장소 위치 확정(10-12 종결), 부록 D 6개 항목 전건 처리, P1~P10 실기 검증 통과. §0 규칙 4 부분 해제 |
| — | — | 라이브러리 복사 시점 커밋 해시: **(복사 시 기재)** |

### E.1 Phase 1 착수 전 정리 작업 (별도 커밋)

- [x] ~~`AGENTS.md` **Git 병합 충돌 마커 제거**~~ → **완료.** 2026-08-27 재확인 시 `<<<<<<<` / `=======` / `>>>>>>>` **0건**. (v3.0 작성 시점 실재 확인 위치는 83·84·245행이었다)
- [x] ~~`STATUS.md` 신규 생성 (저장소 루트)~~ → **완료.** 저장소 루트에 존재하며 요구된 7개 절(했던 일 / 하는 일 / 할 일 / 전체 목표 / 세부 목표 / 사용 기술 / 문제점·해결방안)을 **그 순서 그대로** 갖췄다. **단 Git 미추적 상태**(`??`)라 커밋이 남아 있다
- [ ] `SOFT_LIMIT_ERPM` 통일 (10-3) — Phase 2 착수 전까지 — **절반 완료 (2026-08-27 확인)**
  - [x] ~~**문서-코드 불일치 해소**~~ → **완료.** `AGENTS.md`(초본 `10000`)를 실제 헤더값 `500`에 맞췄다. 현재 `.h:27` · `AGENTS.md:82` · `README.md:155,173` **전부 `500`으로 일치**한다 (`STATUS.md` P5 = 해결)
  - [ ] **정식값 확정** → **미완. NPP 확정이 선행 조건이고 그 NPP는 10-1에서 "매뉴얼에 없음" 확정 상태**라 현재로선 계산이 불가능하다. 공식: `출력축 6 rad/s × NPP × 36`. CubeMarsTool에서 Motor type을 AK45-36으로 바꾼 뒤 Read Parameters로 실측해야 풀린다
  - ℹ️ **Phase 2(위치 제어)를 막지는 않는다.** 이 상수는 `ak45_set_rpm()`에만 걸리는데(`[lib .cpp:311]`) Phase 2는 `ak45_set_position()`만 쓰고 `set_rpm` 호출이 금지돼 있다. 위치 명령을 제한하는 것은 `SOFT_LIMIT_POS_DEG 360.0f` 쪽이다
- [x] ~~저장소 라이선스 확인~~ → **LICENSE 파일 없음 확정.** 팀이 정해야 함 (10-8)
- [x] ~~매뉴얼·소스 인용 실물 대조~~ → **v2.1·v3.0에서 완료**

---

# 부록 F. 자체 검토 4회차 (v2.1 → v3.0)

v2.1을 다시 읽고 원본을 재실행·재대조하며 발견한 것.

### F.1 v2.1이 틀렸거나 위험했던 것

| # | 항목 | v2.1 서술 | 실제 | 조치 |
|---|---|---|---|---|
| **R4-1** | **V3 검증 방법** | "출력축을 손으로 90° 돌린다" | AK45-36은 **36:1 감속기**가 달려 있어 출력축 역구동이 어렵거나 불가능할 수 있다. **검증 절차 자체가 실행 불가능할 수 있었다** | §11.4에 방법 A(역구동)/방법 B(CLI 명령 + 각도기) 2단 절차 신설. 방법 B는 C1 때문에 노드를 꺼야 한다는 점까지 명시 |
| **R4-2** | **V6 "모터 1개 전원 차단"** | 그냥 끄라고만 함 | 24V가 공통 배선이면 개별 차단이 어렵다. CAN 커넥터 분리가 대안인데 **종단저항 위치에 따라 버스 전체가 불안정**해질 수 있다 | 대안 + 주의사항 추가 (§11.4 하단) |
| **R4-3** | **`-lm` 링크** | 원본 Makefile을 그대로 따름 | 라이브러리가 `math.h`를 include만 하고 **수학 함수를 호출하지 않는다.** `-lm` 없이 링크 성공 `[실측]`. Ubuntu 22.04는 glibc 2.35라 libm이 libc에 병합돼 어차피 no-op | `Threads::Threads`만 링크 (§1.3) |
| **R4-4** | **경고 옵션 전역화 근거** | "경고 0개 실측했으니 `add_compile_options` 전역 적용" | 실측 환경이 **GCC 13**이고 **타깃은 Ubuntu 22.04(GCC 11/12)**다. 컴파일러가 다르면 경고 집합도 다르다. 근거가 타깃 환경을 커버하지 못했다 | 원본 Makefile이 실제 쓰는 `-Wall -Wextra`만 전체 적용, 보증 없는 `-Wpedantic`은 노드 파일에만 (§2.4) |
| **R4-5** | **부록 번호 체계** | 부록 A, B, **E, F, G** — C·D가 없고, 부록 뒤에 §11·§12가 다시 나옴 | 문서 구조 결함. 참조 시 혼란 | 본문 §0~§12 → 부록 A~F로 정리 |
| **R4-6** | **미결 질문 7개가 문서 안에 있음** | §12에 Q1~Q7 | "이 문서만 읽고 코드를 짠다"는 합격 기준과 정면 충돌. 에이전트가 질문을 보고 멈춘다 | 질문은 문서 밖(채팅)으로 분리. 문서에는 **결정된 값**과 `[미정]` 표시만 남김 |
| **R4-7** | **`frame_id` 파라미터** | v2.1 스스로 "아무 효과 없다"고 적어놓고 그대로 유지 | 효과 없는 파라미터는 최소 복잡도 위반 | **삭제.** 두 메시지 모두 기본값 `""` 사용 (§4.5) |
| **R4-8** | **`emulate_tty=True` 근거** | "없으면 FATAL이 화면에 안 나온다" | C++ `RCLCPP_FATAL`은 stderr로 나가 보통 없어도 보인다. **과장된 서술** | 표현 완화 (§10) |
| **R4-9** | **커밋 대상 저장소** | "커밋한다"고만 함 | `~/ros2_ws/src/ak45_ros2`가 `team-jax/Can` 안인지 새 저장소인지 **아무 데도 안 정해져 있다** | `[미정]` 10-12 신설, §2.2·§11.5에 표시 |
| **R4-10** | **V11 명령 문법** | `-p joint_names:="[a,b,c]"` | 문자열 배열은 작은따옴표가 필요해 그대로 치면 파싱이 안 될 수 있다 | `-p "joint_names:=['a','b','c']"`로 수정 |

### F.2 v2.1이 맞았고 v3.0에서도 유지한 것

원본 소스 재대조 결과 아래는 전부 일치했다:

`CAN_INTERFACE` = `.h:15`, `SOFT_LIMIT_ERPM 500` = `.h:27`, 워치독 판정 `elapsed < 200` = `.cpp:169`(경계 200ms 포함 실패), `pthread_join` 무조건 호출 = `.cpp:243`, flock 미해제 = `.cpp:190–225`, 재송신 루프 위치 = `main.cpp:164` / `demo_ramp.cpp:19,163`, 명령↔피드백 동일 축 근거 = `demo_ramp.cpp:105,143`, NPP=21 오염 경고 = `AGENTS.md:190`, `SOFT_LIMIT_ERPM` 불일치 = `AGENTS.md:42`, LICENSE 파일 없음, `AGENTS.md` 충돌 마커 83/84/245행.

매뉴얼 쪽: 피드백 스케일 4종, 에러코드 0~7, CAN ID 계산식, 업로드 주파수 1~500Hz·DLC 8, 1Mbps, AK45-36 MIT 상한(±6.0 rad/s, ±34 N·m), 보드별 전류 정격.

C++17 + `-Wall -Wextra -Wpedantic`에서 **경고 0개** — 단 GCC 13 기준 `[실측]` (R4-4 참조).

### F.3 값이 없어서 코드를 못 짜는 지점

**Phase 1에는 없다.** 미정 항목 전부가 (a) 빈 배열 처리(NPP/KT), (b) 임시값 + 파라미터 교체(joint 이름), (c) V3 절차로 판별(감속비), (d) Phase 2 이후(부호·오프셋)로 우회된다.

단 **§11.4 V3 결과에 따라 §7.4 변환식에 `/36.0`이 추가될 수 있다.** 코드 1줄이며 V3 없이는 미리 결정할 수 없다.