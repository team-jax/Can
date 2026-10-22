<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-08-20 | Updated: 2026-08-20 -->

# src

## Purpose
**이 디렉터리에는 노드 구현 파일 하나만 있다.** 2026-10-15 CAN 계층 분리 전에는 제어 라이브러리 `.cpp` 사본이 함께 있었지만, 지금은 `../../ak45_can/src/` 의 원본을 `CMakeLists.txt` 가 직접 컴파일한다.

## Key Files

| File | Description |
|------|-------------|
| `ak45_node.cpp` | **우리 코드.** 노드 클래스 전체 + `main()`. 단일 파일이며 헤더를 따로 두지 않는다(노드가 하나뿐이라 외부에 노출할 인터페이스가 없다) |

빌드에 들어가지만 **여기 없는** 파일:

| 경로 | 취급 |
|---|---|
| `../../ak45_can/src/ak45_36_socketcan_control.cpp` | CAN 계층의 원본. SocketCAN 초기화, 수신 pthread, 피드백 파싱(0x29), 워치독, `flock` 잠금 |
| `../../ak45_can/include/ak45_36_socketcan_control.h` | 두 계층이 공유하는 유일한 인터페이스 |

## For AI Agents

### CAN 라이브러리를 고쳐야 할 때
**사본이 없으므로 복사·동기화 작업은 없다.** `ak45_can/` 에서 고치면 여기 빌드에 바로 반영된다.
대신 **한 번의 수정이 CLI 와 ROS 노드 양쪽에 동시에 영향을 준다** — 둘 다 빌드해 확인한다:

```bash
cd ~/바탕화면/Can/ak45_can && make clean && make
cd ~/ros2_ws && colcon build --packages-select ak45_ros2
```

라이브러리 `.cpp` 는 `-Wpedantic` 없이 컴파일된다. `ak45_can/Makefile` 이 `-Wall -Wextra`만 쓰므로 그 이상은 통과가 보증되지 않기 때문이다. 경고가 뜨면 **라이브러리를 고치지 말고** `../../docs/ros2.md` 부록 E에 기록한 뒤 경고 옵션을 낮춘다.

### 노드 파일 구조 (`../../docs/ros2.md` 부록 A가 정본)
생성자는 **이 순서를 바꾸면 안 된다**:

1. `rclcpp::Node("ak45_state_publisher")`
2. 파라미터 선언·읽기
3. **파라미터 검증** (실패 시 FATAL → `throw`)
4. `ak45_init()` (실패 시 FATAL 3줄 → `throw`), 성공하면 `can_ready_ = true`
5. 퍼블리셔 2개 (**상대 이름** `"joint_states"`/`"diagnostics"` — 앞에 `/`를 붙이지 않는다)
6. 타이머 2개
7. 시작 INFO 1줄

**3을 4보다 먼저 두는 이유**: 파라미터가 틀렸는데 CAN을 먼저 열면 `throw` 시 소멸자가 불리지 않아 소켓과 `flock`이 프로세스 종료까지 남는다.

### 절대 빼면 안 되는 것 — `can_ready_` 가드
소멸자의 `if (can_ready_)`는 편의 기능이 아니다. `ak45_close()`가 `pthread_join()`을 **무조건** 호출하므로, `ak45_init()` 실패로 수신 스레드가 생성되지 않은 상태에서 부르면 **초기화되지 않은 `pthread_t`를 join = 미정의 동작**이다.

### `throw`를 쓰고 `rclcpp::shutdown()`을 쓰지 않는 이유
생성자에서 `rclcpp::shutdown()`을 부르면 `main()`이 **정상 종료(코드 0)** 로 끝나 스크립트가 실패를 감지할 수 없다. `throw` → `main()`의 catch → `return 1`이어야 `echo $?`로 검증된다(V9·V11).

### 발행 규칙
- `/joint_states`: `valid != 0`인 모터만 포함 → **배열 길이가 0~6으로 변한다.** 피드백을 못 받은 모터의 `position_deg`는 `memset` 결과인 `0.0f`이고, 발행하면 "관절이 0°에 있다"는 거짓 정보가 되기 때문이다. **소비자는 인덱스가 아니라 `name`으로 매칭해야 한다.**
- `/diagnostics`: `status`가 **항상 정확히 6개.** 피드백 없는 모터도 포함한다(그게 진단의 목적).
- `velocity`/`effort`는 **손대지 않고 빈 배열로 둔다.** NPP·KT가 미확정이라 변환식을 만들 수 없다. 추측값을 넣지 말 것.
- 워치독이 끊긴 모터는 `valid == 1`이므로 **`/joint_states`에 마지막 값이 계속 들어간다.** 의도된 동작이다.

### 라이브러리가 stderr로 직접 출력한다
`parse_feedback()`이 `error_code != 0`인 프레임마다 `fprintf(stderr, ...)`를 찍는다. 50Hz 피드백이면 초당 50줄이다. **노드 버그가 아니며, 노드가 같은 내용을 또 찍어 중복시키지 말 것.**

## Testing Requirements
```bash
cd ~/ros2_ws && colcon build --packages-select ak45_ros2   # 경고 0개
grep -nE "std::mutex|std::thread" src/ak45_node.cpp    # 출력 없어야 정상
grep -nE "ak45_set_|emergency_stop" src/ak45_node.cpp  # 출력 없어야 정상
```

`can0` 없이도 실패 경로는 검증된다(전부 종료 코드 1이어야 한다):
```bash
ros2 run ak45_ros2 ak45_node --ros-args -p "joint_names:=['a','b','c']"; echo $?
ros2 run ak45_ros2 ak45_node --ros-args -p publish_rate_hz:=600.0; echo $?
```

## Common Patterns
- `fmt2()`로 소수 2자리 서식. **`std::to_string(double)`을 쓰지 않는다** — 로케일·자릿수 문제가 있다.
- 정수 KeyValue는 `std::to_string(static_cast<int>(...))`.
- `hardware_id`는 `snprintf("ak45_id_0x%02X", id)`.

## Dependencies

### Internal
- `../../ak45_can/include/ak45_36_socketcan_control.h` — 빌드에 들어가는 두 `.cpp` 모두 이것만 include한다. `CMakeLists.txt` 의 `target_include_directories` 가 `${AK45_CAN_DIR}/include` 를 넣어 준다

### External
- `rclcpp`, `sensor_msgs`, `diagnostic_msgs`, `pthread`

<!-- MANUAL: -->
