# STATUS — AK45-36 로봇팔 제어 프로젝트

> 갱신: 2026-08-20 · 기준 커밋: `893d663`
> 이 문서는 **프로젝트 전체 상태**를 적는다. 개별 패키지 사용법은 각 README,
> 프로토콜 상세는 `AGENTS.md`, ROS2 Phase 1 사양은 `ros2.md`(v3.0)를 본다.

---

## 1. 했던 일

### Phase 0 — C++ 제어 라이브러리 (완료)

- `ak45_36_socketcan_control.{h,cpp}` — SocketCAN 초기화, 수신 pthread, 피드백 파싱(0x29),
  명령 함수(클램핑·에러 차단), 모터별 워치독(200ms), `flock` 단일 프로세스 잠금
- 단일 모터 → **ID 6번까지 확장 완료** (`g_state[NUM_MOTORS]` 배열화, `motor_index()` 조회)
- `main.cpp` → `ak45_ctrl` : 피드백 모니터링 + 터미널 목표각도 입력 CLI
- `demo_ramp.cpp` → `ak45_ctrl_demo` : 시연용 계단식 완속 이동 (논블로킹 타이머)
- 프로토콜 값 전건을 매뉴얼 V1.0.18 원문과 대조 완료 (`AGENTS.md` §2, §8)

### Phase 1 — ROS2 상태 발행 노드 (**실기 검증 통과**)

- `~/ros2_ws/src/ak45_ros2` 패키지 생성. 라이브러리는 **수정 없이 파일 복사** (원본 `893d663`)
- `ak45_state_publisher` 노드 구현 — `rclcpp`, 단일 노드, **노드 생성 스레드 0개, mutex 0개**
- `colcon build` 통과 — **GCC 11.4.0(Ubuntu 22.04 타깃)에서 경고 0개**.
  `ros2.md` R4-4가 "실측이 GCC 13뿐이라 타깃 보증 없음"으로 유보했던 항목이 해소됐다
- 사양 준수 감사 통과: 작업 규칙 7개, 로그 문구 3종, FATAL 문구 6줄, 진단 message 4종,
  KeyValue 키 6종, QoS, 토픽 상대이름 — 전부 `ros2.md` 문구·값과 일치 확인
- 하드웨어 없이 가능한 검증 통과:

| 검증 | 결과 |
|---|---|
| V11 `joint_names` 3개 | FATAL + **종료 코드 1** |
| `publish_rate_hz:=600.0` | FATAL + 종료 코드 1 |
| `diagnostics_rate_hz:=0.0` | FATAL + 종료 코드 1 |
| F1 `ak45_init()` 실패(can0 없음) | FATAL 3줄 + 종료 코드 1 |
| 경계 `publish_rate_hz:=500.0` (상한 포함) | 검증 통과 → `ak45_init()` 단계까지 진행 |

### Phase 2 — ROS2 명령 수신 (2026-08-27, **P1~P10 실기 검증 통과**)

- **패키지를 저장소 안으로 편입** — `~/ros2_ws/src/ak45_ros2` → `Can/ak45_ros2/`,
  워크스페이스에는 **심볼릭 링크**. `ros2.md` 부록 C **10-12 종결**
- **이름 확정**: `ak45_state_publisher` → **`ak45_node`** (클래스 `Ak45Node`, 실행파일 `ak45_node`,
  launch `ak45_node.launch.py`, config `ak45_node.yaml`). 옛 실행파일은 `build`/`install`에서 제거
- **`/ak45/command` 구독 추가** — `sensor_msgs/msg/JointState`, **이름 매칭**(인덱스 아님).
  커스텀 메시지 만들지 않음
- **타이머 3개** — 상태 50Hz / 진단 2Hz / **명령 재송신 10Hz**. 전부 기본 MutuallyExclusive 그룹이라
  **mutex·thread·MultiThreadedExecutor 여전히 0개**. `ros2.md` 부록 D 6번 재검토 종결
- **안전장치 S1~S4** — 각도 상한(**거부**, 클램프 아님) / 온도 래치 / 피드백 없는 모터 제외 / 송신 실패 로깅
- **진단 KeyValue 6개 → 9개** (`target_deg`, `command_active`, `position_error_deg`)
- `colcon build` **경고 0개**(GCC 11, `-Wall -Wextra` + 노드 파일 `-Wpedantic`)
- 사양서 `ros2.md`에 **§13 신설**(v4.0). §0 규칙 4를 `set_position`에 한해 해제

| 검증 | 결과 |
|---|---|
| P1 첫 명령 전 | 명령 프레임 **0건** (6초/300프레임 전부 피드백) |
| P2 첫 명령 | `00000402` **10.0Hz**, 전류 0.00→**0.13A**(홀딩 인가) |
| P3 값 변경 추종 | 페이로드 → `00 01 86 8D`(9.998도), 피드백 0도→**9.9도** |
| P4 상한 거부 | WARN 1줄, 프레임 0건, `target_deg` 이전 값 유지 |
| P5 없는 모터 | 목표는 저장, **송신 0건**(S3) |
| P6·P7 검증 실패 | 각각 WARN 1줄, 메시지 전체 무시 |
| P8 종료 | 브레이크 0A **6개 전부** 확인 (앞 세션 「발견 1」보다 개선) |
| P9 이중 실행 | FATAL + **종료 코드 1** |
| P10 발행 주기 | **49.997Hz / 2.000Hz** |

⛔ **V3(기준축 판별)은 여전히 미실시다.** P3의 추종은 판별이 아니다 — 출력축이 실제로 몇 도
도는지 사람이 눈으로 봐야 한다. 그래서 `max_command_deg`를 **15.0으로 묶어 뒀다.**

### 문서 정리 (2026-08-20)

- `AGENTS.md` **Git 병합 충돌 마커 제거**(83/84/245행) — 상단 18행이 하단 §1~§8을 참조하므로
  **양쪽 모두 유지**하고 마커 3줄만 삭제
- `AGENTS.md`의 `SOFT_LIMIT_ERPM` **문서-코드 불일치 정정** — 문서는 초본값 `10000`, 코드는 시연용 `500`
  이었다. 문서를 실제값에 맞추고 Phase 2 전 확정 항목으로 표시 (값 자체는 NPP 확정 후 결정)
- `ros2.md` v3.0 확정 (자체 검토 4회차)
- `STATUS.md` 신규 작성 (이 문서)
- **Phase 1 재검증** — `build/install/log` 삭제 후 클린 `colcon build`. GCC 11.4.0에서 **경고 0개**,
  실패 경로 5건(파라미터 위반 3종 + `ak45_init()` 실패 + 경계값 500.0) 전부 명세 문구·종료 코드 일치.
  금지 구문(`std::mutex`/`std::thread`/`create_callback_group`)·모터 명령 호출 **0건**,
  라이브러리 복사본이 원본과 **바이트 단위 동일**, README 커밋 해시가 실제 `893d663` 전체 해시와 일치 확인
- **`AGENTS.md` 자동 생성 구역 갱신** — `<!-- MANUAL -->` 아래 수작성 §1~§8은 바이트 단위 보존.
  드리프트 4건 정정: ① 빌드 산출물 Git 추적·`.gitignore` 부재 명시 ② `ak45_demo`가 현재 Makefile
  타겟이 아님 ③ Makefile `-std=c++11` vs ROS2 C++17 이중 표준 ④ `libm` 설명 정정(라이브러리 `.cpp`는
  수학 함수 미호출이나 `demo_ramp.cpp`는 `fabsf()` 호출). 누락 파일 5개·`Subdirectories` 절 추가
- **`~/ros2_ws/src/ak45_ros2`에 AGENTS.md 계층 6개 생성** — 루트/`src`/`include`/`include/ak45_ros2`/
  `config`/`launch`. Parent 참조 5개 전부 해석 확인, 재빌드 경고 0개.
  부작용: `install(DIRECTORY launch config ...)`가 `config`·`launch`의 AGENTS.md도 `share/`에 설치한다
- **`README.md` 전면 재작성** — 기존 README는 `AGENTS.md` §1~§8의 **낡은 복사본**이었고 드리프트가 실재했다:
  존재하지 않는 파일 2개(`AK45_36_ServoMode_CAN.ino`, `ak45_36_canable_servo_control.py`)를 안내,
  "실제 모터 **2대**"(현재 6대), `main.cpp`·`demo_ramp.cpp`·`Makefile` 누락, 코드 펜스(```) 유실로
  코드 블록이 본문으로 렌더링. → **진입점 문서로 재작성**(빌드·실행·CLI 사용법·Phase 상태·문서 지도),
  프로토콜 전문은 중복하지 않고 `AGENTS.md` §1~§8을 정본으로 참조

## 2. 하는 일

**Phase 2 완료(2026-08-27). 다음은 V3 기준축 판별이다.**

Phase 2(`/ak45/command` 명령 수신)를 구현하고 P1~P10을 실기 검증으로 전부 통과시켰다
(`ros2.md` §13.9). 패키지는 `team-jax/Can` 저장소 안 `ak45_ros2/`로 편입됐다(10-12 종결).

| 항목 | 필요 조건 | 상태 |
|---|---|---|
| P1~P10 (Phase 2 검증) | 실모터 | ✅ **전건 통과** (`ros2.md` §13.9) |
| V0·V1·V2·V4·V5·V14·V15 | 노드 실행 | ✅ 통과 (앞 세션 + 이번 P10) |
| V7·V8 — `candump` 명령 프레임 | 실모터 | ✅ 통과. **P8에서 브레이크 프레임 6개 전부 확인** |
| V9 — flock 충돌 | 실모터 | ✅ 통과 (P9, 종료 코드 1) |
| **V3 — 위치 피드백 기준축 판별** | 실모터 + **사람 눈** | ⛔ **여전히 미실시 — 지금 최우선 항목** |
| V6 — 피드백 끊김 감지 | 실모터 (CAN 커넥터 분리) | ⛔ 미실시 |

**V3가 지금의 핵심이다.** 판별 결과가 "모터축 기준"이면 `ros2.md` §7.4 변환식에 `/ 36.0`
한 줄이 추가되고, §7.4·코드 주석·이 문서를 **같은 커밋에서** 갱신해야 한다.

⚠️ **P3에서 목표 10도에 피드백 9.9도가 따라온 것은 V3 판별이 아니다.** 명령과 피드백이 같은
축을 쓴다는 것은 이미 소스로 확정된 사실이다. **출력축이 실제로 몇 도 돌았는지 사람이 눈으로
봐야** 끝난다. 그때까지 `max_command_deg`는 **15.0으로 둔다.**

## 3. 할 일

### 즉시

- [x] ~~**ROS2 패키지를 어느 저장소에 둘지 결정**~~ → **완료(2026-08-27): `team-jax/Can` 안 `ak45_ros2/`.**
      `~/ros2_ws/src/ak45_ros2`는 심볼릭 링크. `ros2.md` 10-12 종결
- [x] ~~`ros2.md`를 Git에 추가~~ → **완료.** `RUN.md`·`STATUS.md`와 함께 추적 시작
- [ ] **V3 기준축 판별** (`ros2.md` §11.4) — **지금 최우선.** 출력축이 실제로 몇 도 도는지 눈으로 확인.
      끝나면 `max_command_deg`를 15.0에서 올릴 수 있다
- [ ] **온도 원인 규명** — 전류 0A에서 26→53℃까지 오른 원인이 미상이다. 이동할 때마다 1℃씩 더 오른다
      (P2 후 53℃, P3 후 54℃). `max_temperature_c` 60.0까지 여유가 6℃뿐이라 검증 전 온도를 먼저 본다
- [x] ~~**`.gitignore` 신규 작성**~~ → **완료(2026-08-27).** 빌드 산출물 `*.o`·`ak45_ctrl`·`ak45_ctrl_demo`·
      `ak45_demo` 를 `git rm --cached` 로 추적 해제하고 `.gitignore` 에 등록. 로컬 파일은 남아 있다.
      **이제 `make` 를 돌려도 working tree 가 더러워지지 않는다.** `build/`·`install/`·`log/` 와
      `.omc/state/`·`.omc/sessions/` 도 함께 무시한다(`.omc/specs/` 는 분석 문서라 추적 유지)
- [ ] **`ak45_demo` 처리 결정** — Makefile이 만들지 않는 구 바이너리다. **Git 추적은 해제됐고**
      로컬 파일만 남아 있다. 로컬에서도 지울지 확인 필요
- [ ] 저장소 **LICENSE 결정** — LICENSE 파일이 없고 README·`AGENTS.md`에도 표기가 없다.
      팀이 정해야 `package.xml`의 `<license>TODO</license>`를 채울 수 있다

### Phase 2 — **완료 (2026-08-27)**

- [x] ~~Phase 2 별도 사양서 작성 및 승인~~ → **`ros2.md` §13**으로 이 문서 안에 넣었다. P1~P10 통과
- [ ] **AK45-36 극쌍수(NPP) 확인** — CubeMarsTool에서 Motor type을 **AK45-36으로 바꾼 뒤** Read Parameters.
      **Phase 2를 막지는 않았다**(위치 제어만 쓰므로). `velocity` 필드와 `SOFT_LIMIT_ERPM`이 여기 걸려 있다
- [ ] **`SOFT_LIMIT_ERPM` 확정** — 헤더는 시연용 `500`. NPP 확정이 선행 조건.
      **Phase 2와 무관하다** — 이 상수는 `set_rpm` 전용이고 노드는 `set_position`만 쓴다
- [ ] AK45-36 정격/피크 전류 확인 → `SOFT_LIMIT_CURRENT_A`(현재 보수적 5A) 갱신
- [ ] 모터 측 Timeout(ms) 실제 설정값 확인 → 현재 재송신 100ms가 충분한지 근거가 아직 없다

### Phase 3

- [ ] **URDF 확정** (7DOF/8DOF 조율 중) — TF 트리, 관절 이름, 부호 규약, 영점 오프셋이 전부 여기 걸려 있다
- [ ] `robot_state_publisher` / `diagnostic_aggregator` 연결
- [ ] `ros2_control` 전환 재검토

## 4. 전체 목표

CubeMars AK45-36 KV80 액추에이터 **6대(CAN ID 0x01~0x06)** 로 구성한 휴머노이드 팔을
Linux + CANable(SocketCAN) 환경에서 **Servo 모드 CAN**으로 제어한다. MIT 모드는 쓰지 않는다.
최종적으로 ROS2(Humble) 위에서 MoveIt2로 궤적을 계획·실행하는 것이 목표다.

| Phase | 내용 | 상태 |
|---|---|---|
| Phase 0 | C++ 라이브러리 + CLI로 6대 직접 제어 | **완료** |
| **Phase 1** | **ROS2 읽기 전용 상태 발행 노드** (`/joint_states`, `/diagnostics`) | **구현 완료, 실기 검증 대기** |
| Phase 2 | ROS2 명령 인터페이스 (모터 이동) | 착수 전 (승인 필요) |
| Phase 3 | URDF + `robot_state_publisher` + MoveIt2, `ros2_control` 전환 재검토 | 미착수 |

## 5. 세부 목표

- **Phase 1**: 모터를 **움직이지 않고** 피드백만 50Hz로 `/joint_states`, 2Hz로 `/diagnostics`에 발행.
  표준 메시지만 사용(커스텀 메시지 없음). 노드에 스레드·mutex를 만들지 않는다.
  검증 항목 V0~V15 전부 통과 후 커밋.
- **Phase 2**: 100ms 주기 **명령 재송신 타이머 필수**(라이브러리에 재송신 루프가 없다).
  `flock` 제약 때문에 Phase 1 노드와 **같은 프로세스**여야 한다. 워치독 실패 모터는 재송신에서 제외.
- **Phase 3**: URDF 확정 → TF 트리 구성 → MoveIt2 연결.

## 6. 사용 기술

| 구분 | 내용 |
|---|---|
| 언어 | C++17 |
| 하드웨어 | CubeMars AK45-36 KV80 ×6 (BLDC + 36:1 유성기어 + 드라이버 일체형) |
| 통신 | SocketCAN, CAN 2.0B Extended(29bit), big-endian, **1Mbps 고정** |
| 어댑터 | CANable (candlelight 펌웨어) |
| 제어 모드 | **Servo 모드** (MIT 모드 미사용) |
| 동시성 | POSIX pthread (수신 스레드 1개 + mutex), `flock` 프로세스 잠금 |
| 빌드 (Phase 0) | GNU Make → `ak45_ctrl`, `ak45_ctrl_demo` |
| ROS2 | **Humble Hawksbill** / Ubuntu 22.04 / `rclcpp` / `ament_cmake` / `rmw_fastrtps_cpp`(기본) |
| ROS 메시지 | `sensor_msgs/JointState`, `diagnostic_msgs/DiagnosticArray` — **표준만 사용** |
| 툴 | CubeMarsTool (모터 파라미터 R/W), `candump` / `cansend` |

## 7. 문제점 · 해결방안

| # | 문제 | 상태 | 해결방안 |
|---|---|---|---|
| P1 | **AK45-36의 NPP(극쌍수)·KT(토크 상수)가 매뉴얼 전문에 없다** | 미해결 (부재 확인됨) | ERPM→rad/s, 전류→토크 변환 불가 → Phase 1은 `velocity`·`effort`를 **빈 배열**로 두고 원본 ERPM을 `/diagnostics`에 실어 우회. CubeMarsTool Read Parameters로 확정 필요 |
| P2 | ⚠️ **NPP=21 함정** | 주의 필요 | 기존 CubeMarsTool 스크린샷의 NPP=21은 Motor type이 **AK80_3으로 잘못 선택된 상태의 프리셋**이다. AK45-36 값으로 쓰면 안 된다 |
| P3 | **위치 피드백이 출력축 기준인지 모터축 기준인지 매뉴얼에 없다** | 판별 절차 확정 | 명령↔피드백이 같은 축인 것은 소스로 확정(`demo_ramp.cpp:105,143`). 남은 건 축 종류 하나뿐 → `ros2.md` §11.4 V3 절차 1회로 종결. 모터축이면 변환식에 `/ 36.0` 추가 |
| P4 | **URDF 미확정** (7DOF/8DOF 조율 중) | 미해결 | TF 트리·관절 부호·영점 오프셋이 전부 차단됨. Phase 1은 관절 이름을 `ak45_1~6` 임시값 + ROS 파라미터로 두어 우회. 추측값(±1, 0.0)을 미리 넣지 않는다 |
| P5 | **`SOFT_LIMIT_ERPM` 문서-코드 불일치** | **해결** (2026-08-20) | 헤더는 시연용 `500`, `AGENTS.md`는 초본값 `10000`으로 어긋나 있었다 → `AGENTS.md`를 실제값에 맞추고 Phase 2 전 확정 항목으로 표시. **값 자체는 NPP 확정 후 결정** |
| P6 | **라이브러리에 명령 재송신 루프가 없다** | Phase 2 과제 | 명령 함수 1회 = CAN 프레임 1개. 실제 100ms 루프는 애플리케이션(`main.cpp:164`)에 있다. 재송신을 멈추면 모터 타임아웃으로 홀딩 토크가 풀린다 → **Phase 2에서 노드가 재송신 타이머를 반드시 구현** |
| P7 | **`flock` 단일 프로세스 제약** | 설계 반영 완료 | `/tmp/ak45_ctrl.lock` 때문에 AK45 프로세스는 시스템에 1개뿐. ROS2 노드와 `ak45_ctrl`을 동시 실행 불가(V3 방법 B에서 노드를 꺼야 하는 이유). Phase 2도 **같은 프로세스**에 합친다 |
| P8 | **피드백이 자동으로 오지 않는다** (가장 흔한 오진) | 문서화 완료 | CubeMarsTool → Application Functions → "Send status over CAN" 체크 + Rate(Hz) 설정 필요. Rate=0이면 `/joint_states`가 영원히 빈 배열 → 노드 실행 전 `candump can0`으로 먼저 확인 |
| P9 | `ak45_close()`가 `pthread_join`을 **무조건** 호출 | 해결 | init 실패로 스레드 미생성 시 미정의 동작 → 노드에 `can_ready_` 가드 필수(구현 완료) |
| P10 | AK45-36 정격/피크 전류 미확정 | 미해결 | 매뉴얼은 드라이버 보드별 정격만 제시. **실물 보드 실크 표기를 눈으로 확인**해야 확정. 그때까지 `SOFT_LIMIT_CURRENT_A 5.0f` 유지 |
| P11 | **ROS2 패키지 저장소 위치 미정** | 미해결 | 현재 `~/ros2_ws/src/ak45_ros2`가 저장소 밖이라 **버전 관리되지 않는다.** 커밋 전 결정 필요 |
| P12 | 저장소 라이선스 없음 | 미해결 | LICENSE 파일 없음 확정. 팀 결정 전까지 `package.xml`은 `TODO` |
| P13 | **빌드 산출물이 Git에 추적된다** (`.gitignore` 부재) | 미해결 | `*.o` 3개 + 바이너리 3개가 커밋되어 있어 `make` 실행만으로 diff가 생긴다. `ak45_demo`는 현재 Makefile 타겟도 아니고 `make clean` 대상도 아닌 **유령 바이너리**다 → `.gitignore` 작성 + `git rm --cached` |
| P14 | **`README.md`가 `AGENTS.md` §1~§8의 낡은 복사본이었다** | **해결** (2026-08-20) | 같은 프로토콜 문서를 두 곳에 두어 실제로 어긋났다(유령 파일 2개 안내, "모터 2대", 코드 펜스 유실). README를 **진입점 문서**로 재작성하고 프로토콜 정본은 `AGENTS.md` 한 곳으로 통일 |
| P15 | **컴파일러 표준이 두 개다** — Makefile `-std=c++11`, ROS2 패키지 C++17 | 주의 필요 | **같은 `.cpp`를 두 표준으로 컴파일**한다. 한쪽에서만 되는 문법을 쓰면 다른 쪽 빌드가 깨진다. 현재는 양쪽 모두 경고 0개로 통과 |
