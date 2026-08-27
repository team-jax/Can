<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-08-20 | Updated: 2026-08-20 -->

# include

## Purpose
헤더 경로 컨테이너. 파일을 직접 담지 않고 패키지명 하위 디렉토리만 둔다 — ROS2 패키지의 표준 배치다.

## Subdirectories

| Directory | Purpose |
|-----------|---------|
| `ak45_ros2/` | 복사해 온 제어 라이브러리 헤더 (see `ak45_ros2/AGENTS.md`) |

## For AI Agents

`CMakeLists.txt`가 include 경로로 지정하는 것은 이 디렉토리가 아니라 **한 단계 아래인 `include/ak45_ros2`** 다:

```cmake
target_include_directories(ak45_state_publisher_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ak45_ros2>)
```

원본 `.cpp`가 `#include "ak45_36_socketcan_control.h"`(따옴표, 경로 없음)로 되어 있고 **그 파일을 수정할 수 없기** 때문이다. 헤더를 `ak45_ros2/` 안에 두면서도 원본을 고치지 않으려면 경로를 그 디렉토리까지 내려야 한다.

→ 노드 코드에서도 `#include "ak45_ros2/ak45_36_socketcan_control.h"`가 아니라 **`#include "ak45_36_socketcan_control.h"`** 로 쓴다.

<!-- MANUAL: -->
