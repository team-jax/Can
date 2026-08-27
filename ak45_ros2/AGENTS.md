<!-- Generated: 2026-08-20 | Updated: 2026-08-20 -->

# ak45_ros2

## Purpose
CubeMars AK45-36 모터 6대(CAN ID 0x01~0x06)의 SocketCAN 피드백을 폴링해 `/joint_states`(50Hz)와 `/diagnostics`(2Hz)로 발행하는 **ROS2 Humble 단일 노드 패키지**. 프로젝트 Phase 1에 해당하며 **읽기 전용이다 — 모터 이동 명령을 보내지 않는다**(종료 시 라이브러리 내부에서 나가는 Current Brake 0A 프레임 6개는 예외).

**이 패키지의 사양서는 저장소 밖에 있다**: `/home/jaejun/바탕화면/Can/ros2.md` (v3.0). 이름·값·QoS·로그 문구·검증 항목 V0~V15가 전부 확정값으로 적혀 있고, **구현은 그 문서를 그대로 따르는 것이 계약이다.** 프로젝트 전체 상태는 같은 저장소의 `STATUS.md`를 본다.

> **이 패키지는 아직 어느 Git 저장소에도 속하지 않는다** (`STATUS.md` P11). `~/ros2_ws/src/` 아래에만 있어 버전 관리되지 않으므로, 파일을 고치면 되돌릴 수 없다. 커밋 위치 결정(`team-jax/Can`에 디렉토리 추가 vs 별도 저장소)이 선행 과제다.
>
> 상위 디렉토리(`~/ros2_ws/src/`)에는 AGENTS.md가 없다. 이 파일이 자기 트리의 루트이므로 `<!-- Parent: -->` 태그를 두지 않는다.

## Key Files

| File | Description |
|------|-------------|
| `package.xml` | ament_cmake 패키지 매니페스트. 의존은 `rclcpp`/`sensor_msgs`/`diagnostic_msgs` 3개 + 실행시점 `launch`/`launch_ros`. **`<license>TODO</license>`** — 저장소에 LICENSE가 없어 팀이 정해야 채울 수 있다(`STATUS.md` P12). `ament_lint_auto`는 **의도적으로 넣지 않았다**(서식 린터가 수정 금지 원본 파일과 충돌) |
| `CMakeLists.txt` | C++17, 실행 파일 `ak45_state_publisher_node`. **경고 옵션이 파일별로 다르다** — `-Wall -Wextra`는 전체, `-Wpedantic`은 `set_source_files_properties`로 우리 노드 파일에만. 원본 Makefile이 보증하지 않는 옵션을 수정 금지 파일에 걸지 않기 위한 것이다(`ros2.md` §2.4) |
| `README.md` | 사용자용 문서. **라이브러리 복사 원본 커밋 해시(`893d663fdb26f1bf5c203067545a9f9888e4e301`)** 를 기록하는 곳이므로 복사본을 갱신하면 여기도 같이 고친다 |

## Subdirectories

| Directory | Purpose |
|-----------|---------|
| `src/` | 노드 구현 + 복사해 온 제어 라이브러리 `.cpp` (see `src/AGENTS.md`) |
| `include/` | 헤더 경로 컨테이너 (see `include/AGENTS.md`) |
| `config/` | 파라미터 YAML (see `config/AGENTS.md`) |
| `launch/` | launch 파일 (see `launch/AGENTS.md`) |

`msg/`, `srv/`, `action/`는 **만들지 않는다.** 발행 데이터가 전부 표준 메시지에 들어가고, 커스텀 메시지를 만들면 `rosidl` 의존이 붙어 `robot_state_publisher`·`rqt_robot_monitor` 같은 기성 도구를 못 쓴다(`ros2.md` §4.4).

## For AI Agents

### 작업 규칙 7개 (`ros2.md` §0 — 어기면 사양 위반)
1. 사양서에 적힌 **이름·값·로그 문구를 그대로** 쓴다. "더 나은 이름"으로 바꾸지 않는다.
2. `[미정]` 항목을 임의로 채우지 않는다. 필요해지면 코드를 쓰지 말고 질문한다.
3. `ak45_36_socketcan_control.h`/`.cpp`는 **한 글자도 수정하지 않는다.**
4. **모터 이동 명령을 보내지 않는다.** `ak45_set_position`/`set_rpm`/`set_current`/`set_duty`/`set_pos_spd`/`set_origin`/`set_current_brake`를 직접 호출하는 코드를 쓰면 안 된다.
5. 커스텀 메시지 패키지를 만들지 않는다.
6. 노드 코드에 `std::mutex`, `std::thread`가 등장하면 설계를 잘못 이해한 것이다.
7. 상수를 바꾸면 **코드·주석·`ros2.md`·`STATUS.md` 네 곳을 같은 커밋에서** 고친다.

### Working In This Directory
- **`can0`은 컴파일 타임 상수다.** `CAN_INTERFACE`가 `#define "can0"`(헤더 15행)이므로 `can_interface` ROS 파라미터는 **의도적으로 존재하지 않는다.** 만들면 값을 넣어도 무시되는 가짜 설정이 된다. 인터페이스를 바꾸려면 헤더를 고치고 재빌드해야 한다.
- **`flock` 단일 프로세스 제약**: 라이브러리가 `/tmp/ak45_ctrl.lock`을 잠그므로 시스템 전체에 AK45 프로세스가 1개뿐이다. 이 노드와 저장소의 `ak45_ctrl` CLI를 **동시에 띄울 수 없다.** Phase 2의 명령 기능도 별도 노드가 아니라 **이 노드에 추가**해야 한다.
- **QoS를 `SensorDataQoS()`/BEST_EFFORT로 바꾸지 말 것.** RELIABLE pub은 BEST_EFFORT sub과도 붙지만 그 역은 성립하지 않는다. 바꾸면 나중에 MoveIt2를 붙일 때 원인을 못 찾는 버그가 된다(`ros2.md` §4.5).
- **`use_sim_time`을 켜지 말 것.** `create_wall_timer`는 이를 무시하는데 `this->now()`는 따르므로, `/clock` 발행자 없이 켜면 `header.stamp = 0`이 되고 **아무 에러도 안 찍힌 채** TF가 데이터를 버린다.
- 파라미터는 **런타임 변경 불가**다. `add_on_set_parameters_callback`을 구현하지 않았다. 값은 YAML에서 바꾸고 재시작한다.

### Testing Requirements
```bash
cd ~/ros2_ws
colcon build --packages-select ak45_ros2   # GCC 11.4.0에서 경고 0개여야 한다
source install/setup.bash
ros2 launch ak45_ros2 state_publisher.launch.py
```

검증 항목은 `ros2.md` §11.3의 **V0~V15**가 정본이다. 현재 상태:

| 구분 | 상태 |
|---|---|
| 빌드(경고 0개, GCC 11.4.0) · 사양 문구 감사 | **통과** |
| 실패 경로 — 파라미터 3종 위반, `ak45_init()` 실패 | **통과** (FATAL 문구 일치 + 종료 코드 1) |
| V0~V15 | **미실행.** `can0` 인터페이스 필요 |
| V3(위치 피드백 기준축 판별) | **실모터 필요.** Phase 1의 유일한 실질 미결 항목 |

`can0`이 없을 때: `sudo ip link add dev can0 type vcan && sudo ip link set up can0`으로 가상 인터페이스를 만들면 V3을 제외한 대부분을 돌릴 수 있다. `can-utils`가 없는 환경이면 raw SocketCAN 프레임 주입기를 직접 컴파일해 대체한다.

### Common Patterns
- 진단 판정은 **위에서부터 먼저 걸리는 것 하나만**: `valid == 0`(WARN) → 워치독 실패(ERROR) → `error_code != 0`(ERROR) → 정상(OK). **`valid`를 워치독보다 먼저 보는 순서가 핵심이다** — `ak45_is_watchdog_ok()`는 `valid == 0`일 때도 0을 반환하므로, 순서를 바꾸면 한 번도 연결된 적 없는 모터가 "끊김"으로 표시된다.
- level은 `DiagnosticStatus::OK`/`::WARN`/`::ERROR` 상수를 쓴다. **숫자 리터럴 0/1/2 금지.**
- 주기 콜백 안에서는 **스로틀 없는 `RCLCPP_*`를 쓰지 않는다** (50Hz면 초당 50줄). 노드가 남기는 로그는 시작 INFO 1줄, 생성자 실패 FATAL, 전 모터 무응답 WARN(5초 스로틀) **3종류뿐**이다.

## Dependencies

### Internal
- `src/ak45_state_publisher_node.cpp` → `include/ak45_ros2/ak45_36_socketcan_control.h`
- 원본 라이브러리 출처: `/home/jaejun/바탕화면/Can` 저장소 (커밋 `893d663`). **submodule이 아니라 파일 복사**다

### External
- `rclcpp` — 노드 베이스, 타이머, 퍼블리셔
- `sensor_msgs` — `JointState`
- `diagnostic_msgs` — `DiagnosticArray`/`DiagnosticStatus`/`KeyValue`
- `Threads::Threads` — 라이브러리의 수신 pthread. **`-lm`은 링크하지 않는다** (원본 Makefile엔 있으나 no-op — `ros2.md` §1.3)
- 전부 `ros-humble-desktop`에 포함되어 추가 설치가 없다

<!-- MANUAL: 아래에 수동 메모를 추가하면 재생성 시 보존된다 -->
