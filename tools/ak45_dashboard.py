#!/usr/bin/env python3
"""ak45_dashboard — AK45-36 CAN 제어·감시 웹 대시보드 (로컬 전용).

브라우저에서 모터 상태를 보고 각도를 넣어 테스트한다. **모든 수치를 10진수로 보여준다**
(CAN ID·함수 ID·데이터 바이트 포함. 16진수는 토글로 같이 볼 수 있다).

프레임 규격은 ../ak45_can/src/ak45_36_socketcan_control.cpp 와 1:1 로 맞췄다.
  명령  mode 4 (SET_POS)          : int32 BE, deg * 10000
  명령  mode 2 (SET_CURRENT_BRAKE): int32 BE, A   * 1000
  피드백 EID = 41<<8 | ID (0x29) · 8B
        pos int16 BE x0.1deg | spd int16 BE x10ERPM
        cur int16 BE x0.01A  | temp int8 | err uint8

안전 규칙 — 라이브러리와 같은 것을 지킨다
  * `/tmp/ak45_ctrl.lock` 에 flock 을 건다. ak45_ctrl / ak45_ctrl_demo / ROS2 노드와
    **동시에 띄울 수 없다.** 같은 버스를 두 프로세스가 제어하면 명령이 충돌한다.
    감시만 할 때는 --read-only 로 락 없이 띄운다(프레임을 한 개도 보내지 않는다).
  * 목표각은 100ms 주기로 **재송신**한다. 멈추면 모터 쪽 타임아웃으로 홀딩 토크가 풀린다.
  * 띄우는 것만으로는 아무 프레임도 보내지 않는다. 첫 전송은 사람이 누를 때다.
  * 종료(Ctrl+C) 시 브레이크 0A 를 보낸다 -> **홀딩 토크가 풀려 출력축에 무게가 있으면 떨어진다.**

사용법
    ./ak45_dashboard.py                      can0, 제어 모드, http://127.0.0.1:8777
    ./ak45_dashboard.py --read-only          감시 전용 (노드/CLI 와 동시 실행 가능)
    ./ak45_dashboard.py --max-deg 360        목표각 상한 올리기 (기본 90)
    ./ak45_dashboard.py --iface can0 --port 9000
    ./ak45_dashboard.py --ids 2              ID 2 만 표시

하드웨어 없이 연습: 터미널 1 에서 ./ak45_vcan_sim.py, 터미널 2 에서 이 대시보드.
"""

import argparse
import collections
import errno
import fcntl
import io
import json
import os
import signal
import socket
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# ─── 프로토콜 상수 (include/ak45_36_socketcan_control.h 와 일치) ───────────────
CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)
CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_ERR_FLAG = 0x20000000

FEEDBACK_FUNC_ID = 0x29          # 10진수 41
MODE_SET_CURRENT_BRAKE = 2
MODE_SET_POS = 4
MODE_NAMES = {
    0: "SET_DUTY", 1: "SET_CURRENT", 2: "SET_CURRENT_BRAKE", 3: "SET_RPM",
    4: "SET_POS", 5: "SET_ORIGIN", 6: "SET_POS_SPD", FEEDBACK_FUNC_ID: "FEEDBACK",
}

SOFT_LIMIT_POS_DEG = 360.0       # 라이브러리 상한. --max-deg 는 이 값을 넘을 수 없다
SOFT_LIMIT_CURRENT_A = 5.0
WATCHDOG_TIMEOUT_MS = 200
RESEND_MS = 100
LOCK_FILE_PATH = "/tmp/ak45_ctrl.lock"

ERROR_STR = {
    0: "정상", 1: "모터 과열", 2: "과전류", 3: "과전압", 4: "저전압",
    5: "엔코더 고장", 6: "MOSFET 과열", 7: "모터 스톨",
}
BUS_STATE_STR = {0: "정상", 1: "경고(error-warning)", 2: "수동(error-passive)", 3: "버스오프(bus-off)"}

# linux/can/error.h
CAN_ERR_CRTL = 0x00000004
CAN_ERR_PROT = 0x00000008
CAN_ERR_ACK = 0x00000020
CAN_ERR_BUSOFF = 0x00000040
CAN_ERR_RESTARTED = 0x00000100
CAN_ERR_CRTL_RX_WARNING = 0x04
CAN_ERR_CRTL_TX_WARNING = 0x02
CAN_ERR_CRTL_RX_PASSIVE = 0x10
CAN_ERR_CRTL_TX_PASSIVE = 0x20
CAN_ERR_CRTL_ACTIVE = 0x40
CAN_ERR_PROT_FORM = 0x02
CAN_ERR_PROT_STUFF = 0x04
ERR_MASK_ALL = 0x1FF

# Python 의 socket 모듈은 CAN_RAW_ERR_FILTER 를 노출하지 않는다(3.12 확인).
# linux/can/raw.h 의 열거값 2 를 직접 쓴다 — 켜지 않으면 bus-off 조차 배달되지 않는다.
CAN_RAW_ERR_FILTER = getattr(socket, "CAN_RAW_ERR_FILTER", 2)


class Motor:
    __slots__ = ("cid", "valid", "last_rx", "pos_raw", "spd_raw", "cur_raw",
                 "temp_c", "err_code", "target_deg", "rx_count")

    def __init__(self, cid):
        self.cid = cid
        self.valid = False
        self.last_rx = 0.0
        self.pos_raw = 0
        self.spd_raw = 0
        self.cur_raw = 0
        self.temp_c = 0
        self.err_code = 0
        self.target_deg = None
        self.rx_count = 0

    def snapshot(self, now):
        stale_ms = None if not self.valid else int((now - self.last_rx) * 1000)
        return {
            "id": self.cid,
            "id_hex": "0x%02X" % self.cid,
            "valid": self.valid,
            "stale_ms": stale_ms,
            "watchdog_ok": bool(self.valid and stale_ms is not None
                                and stale_ms < WATCHDOG_TIMEOUT_MS),
            "pos_deg": round(self.pos_raw * 0.1, 2),
            "pos_raw": self.pos_raw,
            "spd_erpm": self.spd_raw * 10,
            "spd_raw": self.spd_raw,
            "cur_a": round(self.cur_raw * 0.01, 2),
            "cur_raw": self.cur_raw,
            "temp_c": self.temp_c,
            "err_code": self.err_code,
            "err_str": ERROR_STR.get(self.err_code, "알 수 없음"),
            "target_deg": self.target_deg,
            "rx_count": self.rx_count,
            "feedback_eid": (FEEDBACK_FUNC_ID << 8) | self.cid,
        }


class Bus:
    """SocketCAN 소켓 하나를 열고 수신 스레드 + 100ms 재송신 스레드를 돌린다."""

    def __init__(self, iface, ids, read_only, max_deg, raw_id=7, log_size=2000):
        self.iface = iface
        self.read_only = read_only
        self.max_deg = max_deg
        # CAN ID 7 은 AK45 가 아니라 사용자가 만든 STM32 보드이고 PWM 으로 모터를
        # 돌린다(2026-10-15 확인). 프로토콜이 다르므로 이 쪽 명령 함수를 쓰지 않고
        # 「임의 프레임 전송」으로만 두드린다. 여기 값은 그 패널의 기본 ID 일 뿐이다.
        self.raw_id = raw_id
        self.lock = threading.Lock()
        self.motors = collections.OrderedDict((i, Motor(i)) for i in ids)
        self.bus = dict(state=0, err_frames=0, bit_stuff_errors=0, form_errors=0,
                        ack_errors=0, busoff_count=0, restart_count=0)
        self.log = collections.deque(maxlen=log_size)
        self.log_seq = 0
        self.rx_frames = 0
        self.tx_frames = 0
        self.tx_errors = 0
        self.last_tx_error = ""
        self._rate_mark = time.monotonic()
        self._rate_rx = 0
        self._rate_tx = 0
        self.rx_hz = 0.0
        self.tx_hz = 0.0
        self.started = time.time()
        self.running = True
        self.sock = None
        self.lock_fd = None

    # ── 수명 관리 ────────────────────────────────────────────────────────────
    def acquire_process_lock(self):
        """라이브러리와 같은 flock. 제어 모드에서만 건다."""
        fd = os.open(LOCK_FILE_PATH, os.O_CREAT | os.O_RDWR, 0o644)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            os.close(fd)
            return False
        self.lock_fd = fd
        return True

    def open(self):
        s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        # 에러 프레임 수신. 기본값 0 이라 켜지 않으면 bus-off 조차 안 배달된다.
        try:
            s.setsockopt(socket.SOL_CAN_RAW, CAN_RAW_ERR_FILTER,
                         struct.pack("=I", ERR_MASK_ALL))
        except (OSError, AttributeError) as e:
            # 실패해도 치명적이지 않다 — 버스 감시만 못 하고 제어 경로는 그대로다.
            print("[경고] CAN_RAW_ERR_FILTER 설정 실패 (버스 감시 비활성화): %s" % e,
                  file=sys.stderr)
        s.bind((self.iface,))
        s.settimeout(0.2)
        self.sock = s

    def close(self):
        self.running = False
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
        if self.lock_fd is not None:
            try:
                fcntl.flock(self.lock_fd, fcntl.LOCK_UN)
                os.close(self.lock_fd)
            except OSError:
                pass
            self.lock_fd = None

    # ── 로그 ─────────────────────────────────────────────────────────────────
    def _log_frame(self, direction, eid, data, note=""):
        self.log_seq += 1
        func = (eid >> 8) & 0xFF
        self.log.append({
            "seq": self.log_seq,
            "t": time.strftime("%H:%M:%S") + ".%03d" % (int(time.time() * 1000) % 1000),
            "dir": direction,
            "eid": eid,
            "func": func,
            "func_name": MODE_NAMES.get(func, "?"),
            "id": eid & 0xFF,
            "dlc": len(data),
            "data": list(data),
            "note": note,
        })

    # ── 송신 ─────────────────────────────────────────────────────────────────
    def _transmit(self, eid, payload, note=""):
        if self.read_only:
            return False, "읽기 전용 모드다. 프레임을 보내지 않는다."
        if self.sock is None:
            return False, "소켓이 닫혔다."
        frame = struct.pack(CAN_FRAME_FMT,
                            (eid & CAN_EFF_MASK) | CAN_EFF_FLAG,
                            len(payload),
                            payload.ljust(8, b"\x00"))
        try:
            self.sock.send(frame)
        except OSError as e:
            with self.lock:
                self.tx_errors += 1
                self.last_tx_error = "%s (errno %d)" % (e.strerror, e.errno or 0)
                self._log_frame("tx-fail", eid, payload, note="송신 실패: %s" % e.strerror)
            return False, "CAN 송신 실패: %s" % e.strerror
        with self.lock:
            self.tx_frames += 1
            self._rate_tx += 1
            self._log_frame("tx", eid, payload, note=note)
        return True, ""

    def send_position(self, cid, deg, note=""):
        payload = struct.pack(">i", int(deg * 10000.0))
        return self._transmit((MODE_SET_POS << 8) | cid, payload, note or "%.2f도" % deg)

    def send_brake(self, cid, amps=0.0):
        payload = struct.pack(">i", int(amps * 1000.0))
        return self._transmit((MODE_SET_CURRENT_BRAKE << 8) | cid, payload,
                              "브레이크 %.1fA" % amps)

    # ── 명령 접수 ────────────────────────────────────────────────────────────
    def send_raw(self, can_id, data, eff=True):
        """임의 CAN 프레임을 1회 보낸다. 재송신하지 않는다.

        ID 7 처럼 AK45 프로토콜이 아닌 장치(STM32 + PWM)를 두드릴 때 쓴다.
        페이로드 해석을 이쪽에서 하지 않으므로 **아무 검사도 걸리지 않는다** —
        상한(S1)·에러 차단(S4) 전부 우회한다. 보내는 사람이 책임진다.
        """
        if self.read_only:
            return False, "읽기 전용 모드다. 프레임을 보내지 않는다."
        if not 0 <= can_id <= (CAN_EFF_MASK if eff else 0x7FF):
            return False, ("CAN ID 가 범위를 넘는다 (%s: 0~%d)."
                           % ("확장" if eff else "표준",
                              CAN_EFF_MASK if eff else 0x7FF))
        if not 0 <= len(data) <= 8:
            return False, "데이터는 0~8바이트다. 지금 %d바이트." % len(data)
        for b in data:
            if not 0 <= b <= 255:
                return False, "바이트 값은 0~255다. 잘못된 값: %r" % b
        payload = bytes(data)
        frame = struct.pack(CAN_FRAME_FMT,
                            (can_id & CAN_EFF_MASK) | CAN_EFF_FLAG if eff else can_id,
                            len(payload), payload.ljust(8, b"\x00"))
        if self.sock is None:
            return False, "소켓이 닫혔다."
        try:
            self.sock.send(frame)
        except OSError as e:
            with self.lock:
                self.tx_errors += 1
                self.last_tx_error = "%s (errno %d)" % (e.strerror, e.errno or 0)
                self._log_frame("tx-fail", can_id, payload,
                                note="임의 전송 실패: %s" % e.strerror)
            return False, "CAN 송신 실패: %s" % e.strerror
        with self.lock:
            self.tx_frames += 1
            self._rate_tx += 1
            self._log_frame("tx", can_id, payload, note="임의 전송 (1회)")
        return True, ("CAN ID %d (%s) 로 %d바이트 1회 송신. %s"
                      % (can_id, "확장" if eff else "표준", len(payload),
                         "재송신하지 않는다." if payload else "데이터 없는 프레임이다."))

    def set_target(self, cid, deg):
        if cid not in self.motors:
            return False, "ID %d 는 이 대시보드가 다루는 모터가 아니다." % cid
        if deg != deg or abs(deg) == float("inf"):
            return False, "각도가 숫자가 아니다."
        if abs(deg) > self.max_deg:
            return False, ("목표각 %.1f도가 상한 %.1f도를 넘는다. 거부했다 (클램프가 아니다). "
                           "--max-deg 로 올릴 수 있다." % (deg, self.max_deg))
        m = self.motors[cid]
        if m.err_code != 0:
            return False, ("ID %d 가 에러 %d(%s) 상태다. 라이브러리와 같게 명령을 막는다."
                           % (cid, m.err_code, ERROR_STR.get(m.err_code, "알 수 없음")))
        ok, msg = self.send_position(cid, deg, note="%.2f도 (수동)" % deg)
        if not ok:
            return False, msg
        with self.lock:
            m.target_deg = round(deg, 3)
        warn = "" if m.valid else " (주의: 이 모터는 피드백이 없다. 프레임만 나간다)"
        return True, "ID %d 목표 %.2f도. 100ms 주기로 재송신한다.%s" % (cid, deg, warn)

    def clear_target(self, cid):
        if self.read_only:
            return False, "읽기 전용 모드다. 프레임을 보내지 않는다."
        if cid not in self.motors:
            return False, "ID %d 는 이 대시보드가 다루는 모터가 아니다." % cid
        with self.lock:
            self.motors[cid].target_deg = None
        self.send_brake(cid, 0.0)
        return True, "ID %d 재송신을 멈추고 브레이크 0A 를 보냈다. 홀딩 토크가 풀린다." % cid

    def stop_all(self):
        if self.read_only:
            return False, "읽기 전용 모드다. 프레임을 보내지 않는다."
        with self.lock:
            for m in self.motors.values():
                m.target_deg = None
        sent = 0
        for cid in self.motors:
            ok, _ = self.send_brake(cid, 0.0)
            sent += 1 if ok else 0
        # ID 7(STM32/PWM)은 AK45 프레임을 안 받으므로 브레이크 대상이 아니다.
        return True, "전체 정지. 브레이크 0A %d개 송신. 홀딩 토크가 풀린다." % sent

    # ── 스레드 ───────────────────────────────────────────────────────────────
    def rx_loop(self):
        while self.running:
            sock = self.sock
            if sock is None:
                break
            try:
                raw = sock.recv(CAN_FRAME_SIZE)
            except socket.timeout:
                self._tick_rate()
                continue
            except OSError as e:
                if e.errno in (errno.EBADF, errno.ENOTCONN) or not self.running:
                    break
                continue
            if len(raw) < CAN_FRAME_SIZE:
                continue
            can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
            data = data[:dlc]

            if can_id & CAN_ERR_FLAG:
                with self.lock:
                    self._handle_error_frame(can_id, data)
                continue
            if not (can_id & CAN_EFF_FLAG):
                continue

            eid = can_id & CAN_EFF_MASK
            func = (eid >> 8) & 0xFF
            cid = eid & 0xFF
            with self.lock:
                self.rx_frames += 1
                self._rate_rx += 1
                self._log_frame("rx", eid, data)
                if func == FEEDBACK_FUNC_ID and dlc >= 8 and cid in self.motors:
                    m = self.motors[cid]
                    m.pos_raw, m.spd_raw, m.cur_raw, m.temp_c, m.err_code = \
                        struct.unpack(">hhhbB", data[:8])
                    m.valid = True
                    m.last_rx = time.monotonic()
                    m.rx_count += 1
            self._tick_rate()

    def _handle_error_frame(self, can_id, data):
        b = self.bus
        b["err_frames"] += 1
        d = data.ljust(8, b"\x00")
        if can_id & CAN_ERR_BUSOFF:
            b["state"] = 3
            b["busoff_count"] += 1
        elif can_id & CAN_ERR_RESTARTED:
            b["state"] = 0
            b["restart_count"] += 1
        elif can_id & CAN_ERR_CRTL:
            c = d[1]
            if c & (CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE):
                b["state"] = 2
            elif c & (CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_WARNING):
                b["state"] = 1
            elif c & CAN_ERR_CRTL_ACTIVE:
                b["state"] = 0
        if can_id & CAN_ERR_PROT:
            if d[2] & CAN_ERR_PROT_STUFF:
                b["bit_stuff_errors"] += 1
            if d[2] & CAN_ERR_PROT_FORM:
                b["form_errors"] += 1
        if can_id & CAN_ERR_ACK:
            b["ack_errors"] += 1

    def _tick_rate(self):
        now = time.monotonic()
        dt = now - self._rate_mark
        if dt >= 1.0:
            with self.lock:
                self.rx_hz = round(self._rate_rx / dt, 1)
                self.tx_hz = round(self._rate_tx / dt, 1)
                self._rate_rx = 0
                self._rate_tx = 0
                self._rate_mark = now

    def resend_loop(self):
        """목표가 있는 모터에만 100ms 주기로 재송신한다. 멈추면 홀딩 토크가 풀린다."""
        while self.running:
            time.sleep(RESEND_MS / 1000.0)
            if self.read_only:
                continue
            with self.lock:
                targets = [(m.cid, m.target_deg, m.err_code)
                           for m in self.motors.values() if m.target_deg is not None]
            for cid, deg, err in targets:
                if err != 0:
                    with self.lock:
                        self.motors[cid].target_deg = None
                    continue
                self.send_position(cid, deg, note="%.2f도 (재송신)" % deg)
            self._tick_rate()

    # ── 상태 스냅샷 ──────────────────────────────────────────────────────────
    def snapshot(self, since=0):
        now = time.monotonic()
        with self.lock:
            frames = [f for f in self.log if f["seq"] > since]
            return {
                "iface": self.iface,
                "read_only": self.read_only,
                "max_deg": self.max_deg,
                "raw_id": self.raw_id,   # 임의 전송 패널의 기본 CAN ID
                "soft_limit_deg": SOFT_LIMIT_POS_DEG,
                "resend_ms": RESEND_MS,
                "watchdog_ms": WATCHDOG_TIMEOUT_MS,
                "feedback_func": FEEDBACK_FUNC_ID,
                "uptime_s": int(time.time() - self.started),
                "motors": [m.snapshot(now) for m in self.motors.values()],
                "bus": dict(self.bus, state_str=BUS_STATE_STR.get(self.bus["state"], "?")),
                "counters": {
                    "rx_frames": self.rx_frames, "tx_frames": self.tx_frames,
                    "rx_hz": self.rx_hz, "tx_hz": self.tx_hz,
                    "tx_errors": self.tx_errors, "last_tx_error": self.last_tx_error,
                },
                "log_seq": self.log_seq,
                "frames": frames[-400:],
            }


# ─── HTTP ────────────────────────────────────────────────────────────────────
class Handler(BaseHTTPRequestHandler):
    server_version = "ak45_dashboard"
    bus = None          # 클래스 속성으로 주입한다

    def log_message(self, fmt, *args):
        pass            # 요청 로그는 끈다. 200ms 폴링이라 도배된다

    def _send(self, code, body, ctype="application/json; charset=utf-8"):
        data = body if isinstance(body, bytes) else body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _json(self, code, obj):
        self._send(code, json.dumps(obj, ensure_ascii=False))

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/":
            self._send(200, load_page(), "text/html; charset=utf-8")
        elif path == "/api/state":
            since = 0
            if "?" in self.path:
                for part in self.path.split("?", 1)[1].split("&"):
                    if part.startswith("since="):
                        try:
                            since = int(part[6:])
                        except ValueError:
                            since = 0
            self._json(200, self.bus.snapshot(since))
        else:
            self._json(404, {"error": "없는 경로다: %s" % path})

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        try:
            n = int(self.headers.get("Content-Length") or 0)
            payload = json.loads(self.rfile.read(n) or b"{}")
        except (ValueError, TypeError):
            self._json(400, {"ok": False, "msg": "JSON 본문을 읽을 수 없다."})
            return

        if path == "/api/command":
            try:
                cid = int(payload["id"])
                deg = float(payload["deg"])
            except (KeyError, TypeError, ValueError):
                self._json(400, {"ok": False, "msg": "id(정수)와 deg(실수)가 필요하다."})
                return
            ok, msg = self.bus.set_target(cid, deg)
            self._json(200, {"ok": ok, "msg": msg})
        elif path == "/api/raw":
            try:
                can_id = int(payload["can_id"])
                data = [int(b) for b in (payload.get("data") or [])]
                eff = bool(payload.get("eff", True))
            except (KeyError, TypeError, ValueError):
                self._json(400, {"ok": False,
                                 "msg": "can_id(정수)와 data(0~255 정수 배열)가 필요하다."})
                return
            ok, msg = self.bus.send_raw(can_id, data, eff)
            self._json(200, {"ok": ok, "msg": msg})
        elif path == "/api/clear":
            try:
                cid = int(payload["id"])
            except (KeyError, TypeError, ValueError):
                self._json(400, {"ok": False, "msg": "id(정수)가 필요하다."})
                return
            ok, msg = self.bus.clear_target(cid)
            self._json(200, {"ok": ok, "msg": msg})
        elif path == "/api/stop":
            ok, msg = self.bus.stop_all()
            self._json(200, {"ok": ok, "msg": msg})
        else:
            self._json(404, {"ok": False, "msg": "없는 경로다: %s" % path})


# ─── 페이지 ──────────────────────────────────────────────────────────────────
# HTML/CSS/JS 는 이 파일에 박아 넣지 않고 옆의 dashboard.html 에 둔다.
# 그래야 웹 쪽만 따로 고치고 브라우저 새로고침으로 바로 확인할 수 있다.
PAGE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dashboard.html")


def load_page():
    """매 요청마다 읽는다. 파일을 고치면 서버 재시작 없이 새로고침만으로 반영된다."""
    try:
        with io.open(PAGE_FILE, encoding="utf-8") as f:
            return f.read()
    except OSError as e:
        return ("<!doctype html><meta charset=utf-8><pre style='padding:2em;font:14px monospace'>"
                "dashboard.html 을 읽을 수 없다.\n\n  경로: %s\n  원인: %s\n\n"
                "ak45_dashboard.py 와 같은 디렉터리에 dashboard.html 이 있어야 한다."
                "</pre>" % (PAGE_FILE, e))




# ─── main ────────────────────────────────────────────────────────────────────
def parse_ids(text):
    ids = []
    for part in text.replace(",", " ").split():
        v = int(part, 0)
        if not 0 <= v <= 255:
            raise argparse.ArgumentTypeError("CAN ID 는 0~255 범위다: %s" % part)
        if v not in ids:
            ids.append(v)
    if not ids:
        raise argparse.ArgumentTypeError("ID 를 하나 이상 줘야 한다.")
    return ids


def main():
    ap = argparse.ArgumentParser(
        description="AK45-36 CAN 제어·감시 웹 대시보드 (모든 수치 10진수)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="자세한 주의사항은 파일 상단 docstring 과 ../RUN.md 를 본다.")
    ap.add_argument("--iface", default="can0", help="CAN 인터페이스 (기본 can0)")
    ap.add_argument("--port", type=int, default=8777, help="HTTP 포트 (기본 8777)")
    ap.add_argument("--host", default="127.0.0.1",
                    help="바인드 주소 (기본 127.0.0.1 — 로컬 전용)")
    ap.add_argument("--ids", type=parse_ids, default=list(range(10)),
                    help="다룰 모터 CAN ID 목록 (기본 0~9). 0 도 유효한 ID 다")
    ap.add_argument("--raw-id", type=int, default=7,
                    help="「임의 프레임 전송」 패널에 미리 채울 CAN ID (기본 7 — "
                         "STM32 PWM 보드). 전송 자체는 패널에서 아무 ID 로나 할 수 있다")
    ap.add_argument("--max-deg", type=float, default=90.0,
                    help="목표각 상한(도). 넘으면 거부한다. 기본 90, 최대 %g"
                         % SOFT_LIMIT_POS_DEG)
    ap.add_argument("--read-only", action="store_true",
                    help="프레임을 한 개도 보내지 않는다. flock 도 걸지 않아 "
                         "ak45_ctrl·ROS2 노드와 동시 실행이 된다")
    args = ap.parse_args()

    if not 0 < args.max_deg <= SOFT_LIMIT_POS_DEG:
        print("[오류] --max-deg 는 0 초과 %g 이하여야 한다 (라이브러리 소프트 리밋)."
              % SOFT_LIMIT_POS_DEG, file=sys.stderr)
        return 1

    if not 0 <= args.raw_id <= CAN_EFF_MASK:
        print("[오류] --raw-id 는 0~%d 범위다." % CAN_EFF_MASK, file=sys.stderr)
        return 1

    bus = Bus(args.iface, args.ids, args.read_only, args.max_deg, args.raw_id)

    if not args.read_only:
        if not bus.acquire_process_lock():
            print("[오류] 이미 다른 AK45 프로세스가 %s 를 잡고 있다." % LOCK_FILE_PATH,
                  file=sys.stderr)
            print("       ak45_ctrl / ak45_ctrl_demo / ROS2 ak45_node 중 하나를 끄거나,",
                  file=sys.stderr)
            print("       감시만 하려면 --read-only 로 띄운다.", file=sys.stderr)
            print("       누가 잡고 있는지:  fuser -v %s" % LOCK_FILE_PATH, file=sys.stderr)
            return 1

    try:
        bus.open()
    except OSError as e:
        print("[오류] %s 를 열 수 없다: %s" % (args.iface, e), file=sys.stderr)
        print("       인터페이스를 먼저 올린다:", file=sys.stderr)
        print("         sudo ip link set %s up type can bitrate 1000000 restart-ms 100"
              % args.iface, file=sys.stderr)
        print("       하드웨어 없이 연습하려면 vcan:", file=sys.stderr)
        print("         sudo modprobe vcan && sudo ip link add dev %s type vcan && "
              "sudo ip link set up %s" % (args.iface, args.iface), file=sys.stderr)
        bus.close()
        return 1

    threading.Thread(target=bus.rx_loop, daemon=True, name="can-rx").start()
    threading.Thread(target=bus.resend_loop, daemon=True, name="can-resend").start()

    Handler.bus = bus
    try:
        httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    except OSError as e:
        print("[오류] %s:%d 바인드 실패: %s" % (args.host, args.port, e), file=sys.stderr)
        bus.close()
        return 1
    httpd.daemon_threads = True

    mode = "읽기 전용 (프레임 송신 없음)" if args.read_only else "제어 (flock 보유)"
    ids_dec = " ".join(str(i) for i in args.ids)
    print("[대시보드] http://%s:%d" % (args.host, args.port))
    print("[대시보드] 인터페이스=%s · 모드=%s" % (args.iface, mode))
    print("[대시보드] 모터 ID(10진수)=%s · 목표각 상한=±%g도 · 재송신=%dms"
          % (ids_dec, args.max_deg, RESEND_MS))
    print("[대시보드] 임의 프레임 전송 기본 CAN ID=%d (STM32 PWM 보드 — AK45 프로토콜 아님)"
          % args.raw_id)
    print("[대시보드] 웹 코드: %s — 고치고 브라우저 새로고침하면 바로 반영된다." % PAGE_FILE)
    if not args.read_only:
        print("[대시보드] 첫 '전송' 을 누를 때까지 프레임을 보내지 않는다.")
        print("[대시보드] Ctrl+C 로 끄면 브레이크 0A 를 보낸다 — 홀딩 토크가 풀린다.")
    print("[대시보드] Ctrl+C 로 종료.")

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    threading.Thread(target=httpd.serve_forever, daemon=True, name="http").start()
    while not stop.wait(0.5):
        pass

    print("\n[대시보드] 종료 중…")
    if not args.read_only:
        bus.stop_all()
        time.sleep(0.05)
    httpd.shutdown()
    bus.close()
    print("[대시보드] 종료.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
