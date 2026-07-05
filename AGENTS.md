<!-- Generated: 2026-07-05 | Updated: 2026-07-05 -->

# can_ak45

## Purpose
CubeMars AK45-36 KV80 액추에이터를 Linux + CANable(SocketCAN) 환경에서 Servo 모드 CAN으로 제어하는 C++ 프로젝트. MIT 모드는 사용하지 않는다. 프로토콜 근거는 CubeMars AK Series Module Driver Manual V1.0.18(2026.01.19).

## Key Files

| File | Description |
|------|-------------|
| `ak45_36_socketcan_control.h` | API 선언, 프로토콜 상수(CONTROLLER_ID, 소프트 리밋), MotorState 구조체, CanPacketId/MotorError 열거형 |
| `ak45_36_socketcan_control.cpp` | SocketCAN 초기화, 수신 스레드, 피드백 파싱(0x29), 명령 함수(클램핑·에러 차단 포함), 워치독 |
| `main.cpp` | 피드백 모니터링 루프 예제. 실제 명령은 주석 처리된 예시 참조 |
| `Makefile` | `make` / `make clean`. 타겟 바이너리: `ak45_ctrl` |

프로토콜 상세(매뉴얼 대조 완료), 미확정 사항 체크리스트, 안전 규칙은 별도 CLAUDE.md가 아니라 이 문서 하단의 "AK45-36 KV80 CAN Servo 모드 제어 프로젝트 문서" 섹션(§1~§8)에 통합되어 있다.

## For AI Agents

### CAN 프레임 구조 (변경 금지)
- **명령 CAN ID** = `(CanPacketId << 8) | CONTROLLER_ID` — Extended Frame (29bit)
- **피드백 CAN ID** = `(0x29 << 8) | CONTROLLER_ID` = `0x2901` (CONTROLLER_ID=1 기준)
- 바이트 순서: big-endian (MSB first)

### 소프트 리밋 상수 위치
헤더 파일 `ak45_36_socketcan_control.h` 상단:
- `SOFT_LIMIT_CURRENT_A 5.0f` — 전류 한계 미확정, CubeMarsTool 확인 후 갱신
- `SOFT_LIMIT_ERPM 10000` — NPP 미확정, 공식: `출력축 6 rad/s × NPP × 36` 으로 갱신
- `SOFT_LIMIT_POS_DEG 360.0f` — 운영 범위, 필요 시 조정 가능

### Working In This Directory
- 소프트 리밋을 완화하기 전 반드시 본 문서 §7 미확정 사항(NPP, 전류 한계) 확인
- 새 명령 함수 추가 시 에러 코드 차단(`err != ERR_NONE` 체크) 및 클램핑을 반드시 포함
- 피드백 파싱은 `parse_feedback()` 단일 함수에서만 처리 — 스케일 혼동 방지(본 문서 §2.4)
- 모드 6(Position-Velocity)의 속도 필드는 ERPM÷10 스케일 — 모드 3(RPM)의 ERPM 그대로와 혼동 주의

### Testing Requirements
```bash
sudo ip link set can0 up type can bitrate 1000000
make
./ak45_ctrl                  # 피드백 모니터링
candump can0                 # 송수신 프레임 덤프 확인
```

### Common Patterns
- 명령 함수 패턴: 에러 체크 → 클램핑 → `buffer_append_int32` → `can_transmit_eid`
- 상태 접근: 항상 `g_state_mutex` 락 후 읽기/쓰기

## Dependencies

### Internal
- `ak45_36_socketcan_control.h` ↔ `ak45_36_socketcan_control.cpp` — 단일 모듈 쌍
- `main.cpp` → `ak45_36_socketcan_control.h` 만 포함

### External
- `linux/can.h`, `linux/can/raw.h` — SocketCAN 커널 헤더
- `libpthread` — 수신 스레드
- `libm` — 수학 함수

<!-- MANUAL: 미확정 사항 확인 후 소프트 리밋 갱신 내역을 아래에 기록 -->

# AK45-36 KV80 CAN Servo 모드 제어 프로젝트 문서

> 프로토콜 근거: **CubeMars AK Series Module Driver Manual V1.0.18** (2026.01.19, AK 2.0용)
> 이 문서의 프로토콜 값은 매뉴얼 원문과 대조 완료. ✅ = 매뉴얼로 검증됨, ⚠️ = 미확정(실측/CubeMarsTool 확인 필요)

---

## 1. 프로젝트 개요

CubeMars AK45-36 KV80 액추에이터를 Linux + CANable(SocketCAN) 환경에서 **Servo 모드 CAN**으로 제어한다. MIT 모드는 사용하지 않는다.

### 하드웨어 구성
| 항목 | 내용 |
|---|---|
| 모터 | AK45-36 KV80 (BLDC + 36:1 유성기어 + 드라이버 일체형) |
| CAN 인터페이스 | CANable (candlelight 펌웨어, SocketCAN `can0` 인식 확인됨) |
| OS | Linux |
| CAN 속도 | **1Mbps** ✅ (매뉴얼: "1Mbps, No change recommended") |
| 프레임 | Extended Frame (29bit ID) ✅ (Servo 모드는 확장 프레임만 응답 — 공식 FAQ) |

### 파일 구성
| 파일 | 역할 | 상태 |
|---|---|---|
| `AK45_36_ServoMode_CAN.ino` | Arduino + MCP2515 초기 프로토타입 | 레거시 |
| `ak45_36_canable_servo_control.py` | Python(python-can) + CANable | 보조 |
| `ak45_36_socketcan_control.cpp` | C++ + SocketCAN | **메인** |

---

## 2. Servo 모드 CAN 프로토콜 (매뉴얼 5.1~5.2절 검증 완료)

### 2.1 명령 프레임 (호스트 → 모터) ✅

```
CAN ID (29bit) = (제어모드 << 8) | Controller_ID
프레임 타입: Extended, 데이터: big-endian
```

| 모드 | 이름 | 데이터 | 스케일 / 범위 (매뉴얼 원문) |
|---|---|---|---|
| 0 | Duty Cycle | int32, 4B | 값/100000 = 듀티. 기본 허용 0.005–0.95 |
| 1 | Current Loop (토크) | int32, 4B | 값/1000 = A. **−60000~60000 = −60~60A** |
| 2 | Current Brake | int32, 4B | 값/1000 = A. 0~60000 = 0~60A |
| 3 | Velocity | int32, 4B | **그대로 ERPM. −100000~100000** |
| 4 | Position | int32, 4B | 값/10000 = deg. **−360000000~360000000 = −36000°~36000°** |
| 5 | Set Origin | uint8, 1B | 0=임시 원점(전원 차단 시 소멸), 1=영구 영점(듀얼 엔코더 모델 전용) |
| 6 | Position-Velocity | int32+int16+int16, 8B | pos/10000=deg · spd int16(−32768~32767 → **−327680~327680 ERPM**, 즉 송신값=목표ERPM/10) · acc int16(1단위 = 10 ERPM/s²) |

> 사용자 스펙과의 차이: "Velocity: ERPM 그대로 (스케일 검증 필요)" → 모드 3 단독은 **ERPM 그대로 맞음** (매뉴얼 확정). 단 **모드 6(Position-Velocity)의 speed/acc 필드는 ÷10 스케일**이므로 혼동 주의. 매뉴얼 예제 코드도 `spd/10.0`으로 송신.

### 2.2 피드백 프레임 (모터 → 호스트, Function ID 0x29) ✅

주기 업로드 방식. 업로드 주파수 1~500Hz는 CubeMarsTool → Application Functions → "Send status over CAN" + Rate(Hz)에서 설정. DLC 8, Extended, ID = Function ID + Motor ID.

| 바이트 | 내용 | 타입 / 스케일 (매뉴얼 원문) |
|---|---|---|
| [0:1] | Position | int16 × 0.1 → **−3200° ~ 3200°** |
| [2:3] | Speed | int16 × 10 → **−320000 ~ 320000 ERPM** |
| [4:5] | Current | int16 × 0.01 → −60 ~ 60A |
| [6] | 온도 | int8, **−20~127℃** (오프셋 없음 — MIT 모드의 −40 오프셋과 다름) |
| [7] | Error Code | uint8 |

기타 Function ID: `0x09` = 점프 스타트 상태, `0x2C` = Servo 모드 진입 프레임 (응답 0xFA 0xFB 0xFC 0xFD 고정).

### 2.3 에러 코드 ✅
| 코드 | 의미 | 코드 | 의미 |
|---|---|---|---|
| 0 | 정상 | 4 | 저전압 |
| 1 | 모터 과열 | 5 | 엔코더 고장 |
| 2 | 과전류 | 6 | MOSFET 과열 |
| 3 | 과전압 | 7 | 모터 스톨 |

### 2.4 매뉴얼 원문 참조 코드 (송수신) ✅
```c
// 전송 (예: 속도 모드)
void comm_can_set_rpm(uint8_t controller_id, float rpm) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)rpm, &send_index);
    comm_can_transmit_eid(controller_id |
        ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

// 수신 (0x29 피드백 파싱)
int16_t pos_int = (Data[0] << 8) | Data[1];
int16_t spd_int = (Data[2] << 8) | Data[3];
int16_t cur_int = (Data[4] << 8) | Data[5];
motor_pos  = pos_int * 0.1f;    // deg
motor_spd  = spd_int * 10.0f;   // ERPM
motor_cur  = cur_int * 0.01f;   // A
motor_temp = (int8_t)Data[6];   // ℃
motor_err  = Data[7];
```

---

## 3. 단위 변환 (ERPM ↔ 출력축)

```
출력축 RPM = ERPM / (극쌍수 NPP × 기어비 36)
출력축 rad/s = 출력축 RPM × 2π / 60
```

⚠️ **AK45-36의 극쌍수(NPP)는 미확정.** 이전 CubeMarsTool 스크린샷의 NPP=21은 Motor type이 AK80_3으로 잘못 선택된 상태의 프리셋 값이므로 AK45-36 값으로 사용 금지. Motor type을 AK45-36으로 바꾼 뒤 Read Parameters로 실제 값 확인 후 이 문서와 코드 상수를 동시에 갱신할 것.

---

## 4. 물리적 제한값

- Servo 모드의 하드웨어 리밋(전압/전류/온도/듀티)은 CubeMarsTool → **System Settings**에서 모터에 저장된 값이 기준. Read Parameters로 읽어서 확인.
- 참고: MIT 모드 공식 파라미터(매뉴얼 5.3 표, AK45-36): 위치 ±12.5 rad, **속도 ±6.0 rad/s, 토크 ±34.0 N·m**. Servo 모드 CAN 명령 범위(±60A, ±100000 ERPM)는 프로토콜상 한계일 뿐 모터 물리 한계가 아니므로, **코드의 소프트 리밋은 MIT 표 값을 보수적 상한으로 사용 권장** (출력축 6 rad/s ≈ 57 RPM).
- ⚠️ AK45-36 정격/피크 전류는 미확정 — 제품 스펙시트 또는 CubeMarsTool Current Limits로 확인 후 기록.

---

## 5. 안전 규칙 (코드 작성 시 필수)

1. **소프트 리밋**: 명령 송신 전 클램핑. 속도는 출력축 ±6 rad/s 환산값 이하, 전류는 확정 전까지 보수적으로(예: ±5A) 제한.
2. **피드백 워치독**: 0x29 피드백이 N ms(예: 200ms) 이상 끊기면 Current Brake 0A 또는 Duty 0 송신 후 정지.
3. **에러 코드 처리**: error_code ≠ 0이면 즉시 명령 중단. 특히 1/2/6(과열·과전류)은 재시도 금지.
4. **명령 타임아웃**: 모터 쪽 Timeout(ms) 설정(Application Functions)과 호스트 송신 주기를 맞출 것 — 모터 타임아웃보다 짧은 주기로 명령 재송신해야 정지 안 함.
5. 온도 모니터링: 피드백 [6]이 지속 상승하면 듀티/전류 축소.

---

## 6. CANable / SocketCAN 설정

```bash
sudo ip link set can0 up type can bitrate 1000000   # 1Mbps 고정
candump can0                                         # 피드백 확인
# 예: Motor ID 0x68, 위치 모드(4), 100° = 1000000(0x000F4240)
cansend can0 00000468#000F4240
```

---

## 7. 미확정 사항 체크리스트

- [ ] 실제 모터의 `Controller ID` (코드 placeholder 0x01) — CubeMarsTool → Application Functions에서 확인/설정. **변경 절차: Read Parameters → ID 입력 → Write Parameters → System Reset → 재접속 확인** (Read 선행은 매뉴얼 4.1.1.5 빨간 경고 사항)
- [x] ~~Velocity 명령 스케일~~ → 매뉴얼로 확정: 모드 3은 ERPM 그대로, 모드 6은 ÷10
- [x] ~~피드백 Speed 스케일~~ → 매뉴얼로 확정: int16 × 10 ERPM
- [ ] AK45-36 극쌍수(NPP) — CubeMarsTool에서 Motor type을 AK45-36으로 선택 후 Read Parameters로 확인
- [ ] AK45-36 정격/피크 전류 (Current Limits) — 확인 전까지 소프트 리밋 ±5A
- [ ] Servo 모드 하드웨어 리밋 실측값 (System Settings 저장값)
- [x] CANable 펌웨어 candlelight 확인 (SocketCAN 인식 확인됨)

---

## 8. 사용자 스펙 대비 정정 사항 요약

| 항목 | 기존 스펙 | 매뉴얼 확정값 |
|---|---|---|
| Velocity 명령 스케일 | "검증 필요" | 모드 3: ERPM 그대로 / 모드 6: 송신값 = ERPM÷10 ✅ |
| 피드백 Speed | "스케일 검증 필요" | int16 × 10 = ERPM ✅ |
| 피드백 온도 | int8 ℃ | int8, −20~127℃, 오프셋 없음 (MIT의 −40과 다름) ✅ |
| Position 명령 범위 | 명시 없음 | ±36000° (±100회전) ✅ |
| P/V/T 물리 제한 | "CubeMarsTool 확인 예정" | MIT 표 기준 속도 ±6 rad/s·토크 ±34 N·m는 확보. 전류 한계는 여전히 미확정 |

