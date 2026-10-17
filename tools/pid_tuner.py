#!/usr/bin/env python3
"""Interactive PID tuner for the STM32 balance car.

Protocol matches the Wildfire PID helper frames implemented in Core/Src/main.c:
header 53 5A 48 59, channel, uint32 length, command, payload, checksum.
"""

from __future__ import annotations

import argparse
import csv
import queue
import struct
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit(
        "pyserial is required. Install it with: python -m pip install pyserial"
    ) from exc


HEADER = b"\x53\x5A\x48\x59"
CHANNEL = 0x01

CMD_UPLOAD_TARGET = 0x01
CMD_UPLOAD_ACTUAL = 0x02
CMD_UPLOAD_PID = 0x03
CMD_SET_PID = 0x10
CMD_SET_TARGET = 0x11
CMD_START = 0x12
CMD_STOP = 0x13
CMD_RESET = 0x14
CMD_SET_PERIOD = 0x15


@dataclass
class Telemetry:
    angle_deg: float | None = None
    target_deg: float | None = None
    kp: float | None = None
    ki: float | None = None
    kd: float | None = None
    frames: int = 0
    last_frame_time: float = 0.0


def build_frame(cmd: int, payload: bytes = b"", channel: int = CHANNEL) -> bytes:
    length = 11 + len(payload)
    frame = bytearray()
    frame.extend(HEADER)
    frame.append(channel & 0xFF)
    frame.extend(struct.pack("<I", length))
    frame.append(cmd & 0xFF)
    frame.extend(payload)
    checksum = sum(frame) & 0xFF
    frame.append(checksum)
    return bytes(frame)


def parse_frames(buffer: bytearray):
    while True:
        pos = buffer.find(HEADER)
        if pos < 0:
            if len(buffer) > 3:
                del buffer[:-3]
            return
        if pos:
            del buffer[:pos]
        if len(buffer) < 11:
            return

        length = struct.unpack_from("<I", buffer, 5)[0]
        if length < 11 or length > 64:
            del buffer[0]
            continue
        if len(buffer) < length:
            return

        frame = bytes(buffer[:length])
        del buffer[:length]
        if (sum(frame[:-1]) & 0xFF) != frame[-1]:
            continue

        cmd = frame[9]
        payload = frame[10:-1]
        yield cmd, payload


class PidTuner:
    def __init__(self, port: str, baud: int, log_path: Path | None):
        self.port = port
        self.baud = baud
        self.log_path = log_path
        self.stop_event = threading.Event()
        self.events: "queue.Queue[tuple[int, bytes]]" = queue.Queue()
        self.telemetry = Telemetry()
        self.lock = threading.Lock()
        self.ser: serial.Serial | None = None
        self.log_file = None
        self.csv_writer = None

    def open(self) -> None:
        self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        if self.log_path:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            self.log_file = self.log_path.open("w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.log_file)
            self.csv_writer.writerow(["time_s", "angle_deg", "target_deg", "kp", "kd"])

        thread = threading.Thread(target=self._reader, daemon=True)
        thread.start()
        pump_thread = threading.Thread(target=self._event_pump, daemon=True)
        pump_thread.start()

    def close(self) -> None:
        self.stop_event.set()
        try:
            self.send_stop()
        except Exception:
            pass
        if self.ser:
            self.ser.close()
        if self.log_file:
            self.log_file.close()

    def _reader(self) -> None:
        assert self.ser is not None
        buffer = bytearray()
        while not self.stop_event.is_set():
            data = self.ser.read(128)
            if not data:
                continue
            buffer.extend(data)
            for cmd, payload in parse_frames(buffer):
                self.events.put((cmd, payload))

    def _event_pump(self) -> None:
        while not self.stop_event.is_set():
            self.pump_events()
            time.sleep(0.02)

    def _write(self, cmd: int, payload: bytes = b"") -> None:
        assert self.ser is not None
        self.ser.write(build_frame(cmd, payload))

    def send_pid(self, kp: float, kd: float, ki: float = 0.0) -> None:
        self._write(CMD_SET_PID, struct.pack("<fff", kp, ki, kd))

    def send_target(self, target_deg: float) -> None:
        self._write(CMD_SET_TARGET, struct.pack("<i", int(round(target_deg * 100.0))))

    def send_period(self, period_ms: int) -> None:
        self._write(CMD_SET_PERIOD, struct.pack("<I", period_ms))

    def send_start(self) -> None:
        self._write(CMD_START)

    def send_stop(self) -> None:
        self._write(CMD_STOP)

    def send_reset(self) -> None:
        self._write(CMD_RESET)

    def pump_events(self) -> None:
        changed = False
        while True:
            try:
                cmd, payload = self.events.get_nowait()
            except queue.Empty:
                break

            now = time.time()
            if cmd == CMD_UPLOAD_ACTUAL and len(payload) >= 4:
                raw = struct.unpack_from("<i", payload)[0]
                with self.lock:
                    self.telemetry.angle_deg = raw / 100.0
                    self.telemetry.frames += 1
                    self.telemetry.last_frame_time = now
                changed = True
            elif cmd == CMD_UPLOAD_TARGET and len(payload) >= 4:
                raw = struct.unpack_from("<i", payload)[0]
                with self.lock:
                    self.telemetry.target_deg = raw / 100.0
            elif cmd == CMD_UPLOAD_PID and len(payload) >= 12:
                kp, ki, kd = struct.unpack_from("<fff", payload)
                with self.lock:
                    self.telemetry.kp = kp
                    self.telemetry.ki = ki
                    self.telemetry.kd = kd

        if changed:
            self._log_latest()

    def _log_latest(self) -> None:
        if not self.csv_writer:
            return
        snapshot = self.snapshot()
        self.csv_writer.writerow(
            [
                f"{time.time():.3f}",
                snapshot.angle_deg,
                snapshot.target_deg,
                snapshot.kp,
                snapshot.kd,
            ]
        )
        self.log_file.flush()

    def status_text(self) -> str:
        snapshot = self.snapshot()
        age = time.time() - snapshot.last_frame_time if snapshot.last_frame_time else None
        age_text = "none" if age is None else f"{age:.2f}s ago"
        return (
            f"angle={fmt(snapshot.angle_deg)} deg, "
            f"target={fmt(snapshot.target_deg)} deg, "
            f"kp={fmt(snapshot.kp)}, kd={fmt(snapshot.kd)}, "
            f"frames={snapshot.frames}, last={age_text}"
        )

    def snapshot(self) -> Telemetry:
        with self.lock:
            return Telemetry(
                angle_deg=self.telemetry.angle_deg,
                target_deg=self.telemetry.target_deg,
                kp=self.telemetry.kp,
                ki=self.telemetry.ki,
                kd=self.telemetry.kd,
                frames=self.telemetry.frames,
                last_frame_time=self.telemetry.last_frame_time,
            )


def fmt(value: float | None) -> str:
    return "--" if value is None else f"{value:.3f}"


def print_ports() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    for port in ports:
        print(f"{port.device}: {port.description}")


def print_help() -> None:
    print(
        """
Commands:
  p <kp> <kd>        set PID P and D, I is sent as 0
  z <deg>            set balance zero/target angle in degrees
  period <ms>        set STM32 upload period, 5..500 ms
  start              enable balance control
  stop               disable control and set motors to zero
  status             print latest received angle/PID
  ports              list serial ports
  reset              reset STM32 through protocol command
  help               show this help
  quit               stop motors and exit

Suggested first run:
  stop
  z 0
  p 80 2
  start
"""
    )


def repl(tuner: PidTuner) -> None:
    print(f"Connected to {tuner.port} @ {tuner.baud}. Type 'help' for commands.")
    print("Safety: keep the car supported; 'stop' cuts motor output.")
    tuner.send_period(20)
    last_status = 0.0

    while True:
        tuner.pump_events()
        now = time.time()
        if now - last_status >= 1.0:
            print(tuner.status_text())
            last_status = now

        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return

        tuner.pump_events()
        if not line:
            continue
        parts = line.split()
        cmd = parts[0].lower()

        try:
            if cmd in {"q", "quit", "exit"}:
                return
            if cmd in {"h", "help", "?"}:
                print_help()
            elif cmd == "ports":
                print_ports()
            elif cmd == "p" and len(parts) == 3:
                kp = float(parts[1])
                kd = float(parts[2])
                tuner.send_pid(kp, kd)
                print(f"sent kp={kp}, kd={kd}")
            elif cmd == "z" and len(parts) == 2:
                target = float(parts[1])
                tuner.send_target(target)
                print(f"sent target={target} deg")
            elif cmd == "period" and len(parts) == 2:
                period = int(parts[1])
                tuner.send_period(period)
                print(f"sent period={period} ms")
            elif cmd == "start":
                tuner.send_start()
                print("control enabled")
            elif cmd == "stop":
                tuner.send_stop()
                print("control stopped")
            elif cmd == "status":
                print(tuner.status_text())
            elif cmd == "reset":
                tuner.send_reset()
                print("reset command sent")
            else:
                print("Unknown command or wrong arguments. Type 'help'.")
        except ValueError:
            print("Bad number format.")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Balance car PID tuner")
    parser.add_argument("--port", default="COM6", help="serial port, default COM6")
    parser.add_argument("--baud", type=int, default=115200, help="baud rate, default 115200")
    parser.add_argument("--log", type=Path, default=None, help="optional CSV log path")
    parser.add_argument("--list", action="store_true", help="list serial ports and exit")
    args = parser.parse_args(argv)

    if args.list:
        print_ports()
        return 0

    tuner = PidTuner(args.port, args.baud, args.log)
    try:
        tuner.open()
        repl(tuner)
    finally:
        tuner.close()
        print("Stopped motors and closed serial port.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
