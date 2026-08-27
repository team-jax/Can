<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-08-20 | Updated: 2026-08-20 -->

# config

## Purpose
`ak45_state_publisher` 노드의 파라미터 YAML. 이 패키지의 파라미터는 **런타임 변경이 불가능**하므로, 값을 바꾸려면 이 파일을 고치고 노드를 **재시작**해야 한다.

## Key Files

| File | Description |
|------|-------------|
| `ak45_state_publisher.yaml` | 파라미터 3개(`publish_rate_hz`, `diagnostics_rate_hz`, `joint_names`). `/**` 와일드카드를 써서 노드명·네임스페이스가 바뀌어도 그대로 적용된다 |

## For AI Agents

### 파라미터 3개가 전부다

| 이름 | 타입 | 기본값 | 허용 범위 | 위반 시 |
|---|---|---|---|---|
| `publish_rate_hz` | `double` | `50.0` | `> 0.0` && `<= 500.0` | FATAL → 종료 코드 1 |
| `diagnostics_rate_hz` | `double` | `2.0` | `> 0.0` && `<= 50.0` | FATAL → 종료 코드 1 |
| `joint_names` | `string[]` | `["ak45_1" ... "ak45_6"]` | **길이가 정확히 6** | FATAL → 종료 코드 1 |

상한 500 근거: 서보 모드 피드백 업로드 주파수 상한이 500Hz다. 그보다 빠르게 폴링해도 같은 값을 중복 발행할 뿐이다.

### 파라미터를 새로 추가하기 전에 확인할 것
아래는 **의도적으로 만들지 않은** 파라미터다. "설정이 없네" 하고 추가하면 사양 위반이다.

| 안 만드는 것 | 이유 |
|---|---|
| `can_interface` | `CAN_INTERFACE`가 컴파일 타임 `#define`이라 값을 넣어도 무시되는 **가짜 설정**이 된다 |
| `frame_id` | 소비자(`robot_state_publisher`, `diagnostic_aggregator`)가 읽지 않아 아무 효과가 없다 |
| `motor_ids` | 라이브러리 상수와 1:1이며 변경 금지 |
| `watchdog_timeout_ms` | 라이브러리 `#define` |
| 관절별 부호(±1) / 영점 오프셋 | URDF 미확정. 지금 만들면 값이 전부 추측(±1, 0.0)이라 "설정은 있는데 아무도 안 채운 파라미터"가 된다 |

### `joint_names` 주의
- **개수(6개)만 검증하고 중복은 검증하지 않는다.** 중복 이름을 넣으면 `robot_state_publisher`가 마지막 값으로 덮어써서 **TF가 조용히 어긋난다** — 에러도 경고도 뜨지 않는다.
- 배열 순서가 **CAN ID 0x01~0x06과 1:1**이다. 순서를 섞으면 관절과 모터의 대응이 어긋난다.
- 현재 값 `ak45_1`~`ak45_6`은 **URDF 확정 전 임시값**이다(7DOF/8DOF 조율 중). URDF가 정해지면 여기서 교체한다 — 그래서 launch가 `params_file`을 인자로 받는다.

### 선언 시 함정
`declare_parameter<std::vector<std::string>>("joint_names", {"a","b"})`처럼 중괄호 리스트를 직접 넘기면 템플릿 인자 추론이 실패할 수 있다. **기본값 변수를 먼저 만들어 넘긴다.**

## Testing Requirements
```bash
ros2 param get /ak45_state_publisher joint_names     # 6개 반환 (V10)
ros2 launch ak45_ros2 state_publisher.launch.py params_file:=/경로/실험용.yaml
```

이 디렉토리는 `install(DIRECTORY launch config ...)`로 **통째로** `share/ak45_ros2/`에 설치된다. 여기 파일을 추가하면 그대로 설치물에 포함된다.

<!-- MANUAL: -->
