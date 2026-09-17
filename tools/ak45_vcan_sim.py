#!/usr/bin/env python3
"""ak45_vcan_sim — vcan 위에서 AK45-36 모터 6대를 흉내 낸다.

실기 없이 ak45_node 를 끝까지 돌려보기 위한 개발 전용 도구다.
노드가 쏜 명령 프레임(EID = mode<<8 | id)을 읽어 목표각을 받고,
피드백 프레임(EID = 0x29<<8 | id)을 주기적으로 되쏜다.

프레임 규격은 ak45_36_socketcan_control.cpp 와 1:1 로 맞췄다.
  명령  mode 4 (SET_POS)          : int32 BE, /10000 = deg
  명령  mode 2 (SET_CURRENT_BRAKE): int32 BE, /1000  = A
  피드백 8B: pos int16 BE ×0.1deg | spd int16 BE ×10ERPM
             cur int16 BE ×0.01A | temp int8 | err uint8

사용법
    ./ak45_vcan_sim.py                    6대 전부 정상
    ./ak45_vcan_sim.py --ids 2            ID 0x02 만 피드백 (S3 경로 확인)
    ./ak45_vcan_sim.py --temp 65          온도 65도 (S2 래치 확인)
    ./ak45_vcan_sim.py --error 7          에러코드 7 (S4 경로 확인)
    ./ak45_vcan_sim.py --drop-after 5     5초 뒤 피드백 중단 (워치독 확인)
    ./ak45_vcan_sim.py --iface can0 -v    수신 명령을 전부 출력
"""

import argparse
import socket
import struct
import sys
import time

CAN_FRAME_FMT = "=IB3x8s"          # can_id, can_dlc, pad, data
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)
CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_ERR_FLAG = 0x20000000

FEEDBACK_FUNC_ID = 0x29
MODE_NAMES = {
    0: "SET_DUTY", 1: "SET_CURRENT", 2: "SET_CURRENT_BRAKE", 3: "SET_RPM",
    4: "SET_POS", 5: "SET_ORIGIN", 6: "SET_POS_SPD",
}

# 출력축 이동 속도(도/초). 실기 거동을 흉내 낼 뿐 정확한 모델은 아니다.
SLEW_DEG_PER_SEC = 60.0


class Motor:
    def __init__(self, mid):
        self.id = mid
        self.pos = 0.0
        self.target = 0.0
        self.has_target = False
        self.speed = 0.0
        self.current = 0.0

    def step(self, dt):
        if not self.has_target:
            self.speed = 0.0
            self.current = 0.0
            return
        err = self.target - self.pos
        step = SLEW_DEG_PER_SEC * dt
        if abs(err) <= step:
            self.pos = self.target
            self.speed = 0.0
            self.current = 0.05          # 홀딩 전류
        else:
            move = step if err > 0 else -step
            self.pos += move
            # ERPM = 출력축 deg/s -> 감속비 36, NPP 14 (ros2.md §13.10)
            self.speed = (move / dt) / 6.0 * 36.0 * 14.0
            self.current = 0.45


def clamp_i16(v):
    return max(-32768, min(32767, int(round(v))))


def main():
    ap = argparse.ArgumentParser(description="AK45 vcan 모터 시뮬레이터")
    ap.add_argument("--iface", default="can0", help="CAN 인터페이스 (기본 can0)")
    ap.add_argument("--ids", default="1,2,3,4,5,6",
                    help="피드백을 보낼 모터 ID 목록 (기본 전부)")
    ap.add_argument("--rate", type=float, default=50.0,
                    help="피드백 발행 주기 Hz (기본 50)")
    ap.add_argument("--temp", type=int, default=30, help="보고할 온도 ℃ (기본 30)")
    ap.add_argument("--error", type=int, default=0, help="보고할 에러코드 (기본 0)")
    ap.add_argument("--drop-after", type=float, default=0.0,
                    help="N초 뒤 피드백 중단 (0=중단 없음)")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="수신한 명령 프레임을 전부 출력")
    args = ap.parse_args()

    try:
        ids = [int(x) for x in args.ids.split(",") if x.strip()]
    except ValueError:
        sys.exit(f"오류: --ids 를 해석할 수 없습니다: {args.ids!r}")

    try:
        sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.bind((args.iface,))
    except OSError as e:
        sys.exit(
            f"오류: {args.iface} 바인드 실패 ({e}).\n"
            f"  가상 인터페이스를 먼저 만드세요:\n"
            f"    sudo modprobe vcan\n"
            f"    sudo ip link add dev {args.iface} type vcan\n"
            f"    sudo ip link set {args.iface} up"
        )
    sock.settimeout(0.0)

    motors = {m: Motor(m) for m in range(1, 7)}
    period = 1.0 / args.rate
    t0 = time.monotonic()
    next_tx = t0
    last_step = t0
    rx_count = 0
    tx_count = 0

    print(f"[sim] {args.iface} 에서 대기. 피드백 ID={ids}, {args.rate:.0f}Hz, "
          f"온도={args.temp}℃, 에러코드={args.error}"
          + (f", {args.drop_after:.1f}초 뒤 피드백 중단" if args.drop_after > 0 else ""))
    print("[sim] Ctrl+C 로 종료")

    try:
        while True:
            # ── 수신: 논블로킹으로 큐를 비운다 ──
            while True:
                try:
                    raw = sock.recv(CAN_FRAME_SIZE)
                except BlockingIOError:
                    break
                if len(raw) < CAN_FRAME_SIZE:
                    break
                can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
                if can_id & CAN_ERR_FLAG:
                    continue
                if not (can_id & CAN_EFF_FLAG):
                    continue
                eid = can_id & CAN_EFF_MASK
                mode = (eid >> 8) & 0xFF
                mid = eid & 0xFF
                if mode == FEEDBACK_FUNC_ID or mid not in motors:
                    continue          # 내가 쏜 피드백은 무시
                rx_count += 1
                m = motors[mid]
                if mode == 4 and dlc >= 4:          # SET_POS
                    val = struct.unpack(">i", data[:4])[0]
                    m.target = val / 10000.0
                    m.has_target = True
                    if args.verbose:
                        print(f"[sim] rx ID=0x{mid:02X} SET_POS {m.target:+.3f}도")
                elif args.verbose:
                    print(f"[sim] rx ID=0x{mid:02X} "
                          f"{MODE_NAMES.get(mode, f'mode{mode}')} dlc={dlc} "
                          f"{data[:dlc].hex()}")

            # ── 물리 적분 ──
            now = time.monotonic()
            dt = now - last_step
            last_step = now
            for m in motors.values():
                m.step(dt)

            # ── 피드백 송신 ──
            if now >= next_tx:
                next_tx += period
                dropped = args.drop_after > 0 and (now - t0) >= args.drop_after
                if not dropped:
                    for mid in ids:
                        m = motors.get(mid)
                        if m is None:
                            continue
                        payload = struct.pack(
                            ">hhhbB",
                            clamp_i16(m.pos * 10.0),
                            clamp_i16(m.speed / 10.0),
                            clamp_i16(m.current * 100.0),
                            max(-128, min(127, args.temp)),
                            args.error & 0xFF,
                        )
                        eid = (FEEDBACK_FUNC_ID << 8) | mid
                        frame = struct.pack(CAN_FRAME_FMT,
                                            eid | CAN_EFF_FLAG, 8, payload)
                        sock.send(frame)
                        tx_count += 1

            time.sleep(0.002)
    except KeyboardInterrupt:
        print(f"\n[sim] 종료. 수신 명령={rx_count}, 송신 피드백={tx_count}")
        for m in motors.values():
            if m.has_target:
                print(f"[sim]   ID=0x{m.id:02X} 최종위치={m.pos:+.2f}도 "
                      f"목표={m.target:+.2f}도")


if __name__ == "__main__":
    main()
