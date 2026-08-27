<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-08-20 | Updated: 2026-08-20 -->

# launch

## Purpose
노드 실행용 launch 파일. 이 패키지의 노드는 하나뿐이므로 launch 파일도 하나다.

## Key Files

| File | Description |
|------|-------------|
| `state_publisher.launch.py` | `ak45_state_publisher` 노드 1개를 띄운다. launch argument는 `params_file` **하나뿐**이며 기본값은 `<share>/ak45_ros2/config/ak45_state_publisher.yaml` |

## For AI Agents

### 의도적으로 넣지 않은 것

| 안 넣는 것 | 이유 |
|---|---|
| remapping | 토픽 이름(`/joint_states`, `/diagnostics`)을 사양에서 확정했으므로 리맵할 이유가 없다 |
| 네임스페이스 | 루트(`/`)에 둔다. 완전 이름이 `/ak45_state_publisher`가 되어야 한다 |
| `use_sim_time` | **설정하면 안 된다.** `create_wall_timer`는 이를 무시하는데 `this->now()`는 따르므로, `/clock` 발행자 없이 켜면 `header.stamp = 0`이 되고 **아무 에러 없이** TF가 데이터를 버린다 |
| 다른 launch argument | `params_file` 하나로 충분하다. URDF 확정 전까지 `joint_names`를 자주 바꿔 실험해야 하는데, 그때마다 launch 파일을 고치면 diff가 더러워지기 때문에 둔 인자다 |
| `robot_state_publisher` / `diagnostic_aggregator` | Phase 1 범위 밖. 전자는 URDF가 확정돼야 띄울 수 있고 후자는 분석 규칙 yaml이 필요하다 |

### `emulate_tty=True`
로그 색상 유지와 출력 버퍼링 회피용이다. C++ `RCLCPP_FATAL`은 stderr로 나가 보통 없어도 보이지만, launch를 통해 실행할 때 출력 순서가 뒤엉키는 걸 막아준다.

### 실행 후 기대 상태 — 고아 토픽이 정상이다
`/joint_states`와 `/diagnostics` 모두 **구독자가 없다.** `rqt_graph`에서 이 노드의 화살표 2개가 아무 데도 닿지 않는 것이 Phase 1의 기대 상태다. "끊긴 연결이 없어야 한다"는 일반 규칙을 그대로 적용하면 정상을 결함으로 오판한다.

## Testing Requirements
```bash
ros2 launch ak45_ros2 state_publisher.launch.py
ros2 launch ak45_ros2 state_publisher.launch.py params_file:=/경로/실험용.yaml
ros2 topic list        # /joint_states /diagnostics /parameter_events /rosout — 4개 (V0)
```

launch로 띄우면 `ros2 run`과 달리 노드 이름이 `name='ak45_state_publisher'`로 고정된다. `ros2 param get`이 이 이름을 쓰므로 검증 시 일치해야 한다.

이 디렉토리는 `install(DIRECTORY launch config ...)`로 통째로 `share/ak45_ros2/`에 설치된다.

## Dependencies

### External
- `launch`, `launch_ros` — `package.xml`의 `<exec_depend>`
- `ament_index_python` — `get_package_share_directory()`

<!-- MANUAL: -->
