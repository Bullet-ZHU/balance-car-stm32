#!/usr/bin/env python3
"""Conservative automatic PID trial runner for the balance car.

This script talks to the STM32 Wildfire PID protocol on USART1. It sends short
PID trials, watches the returned angle, and stops the motors on any unsafe
condition.
"""

from __future__ import annotations

import argparse
import csv
import statistics
import sys
import time
from pathlib import Path

from pid_tuner import PidTuner


def wait_samples(tuner: PidTuner, duration_s: float) -> list[tuple[float, float]]:
    samples: list[tuple[float, float]] = []
    end = time.time() + duration_s
    last_frames = -1
    while time.time() < end:
        snap = tuner.snapshot()
        if snap.frames != last_frames and snap.angle_deg is not None:
            last_frames = snap.frames
            samples.append((time.time(), snap.angle_deg))
        time.sleep(0.01)
    return samples


def summarize(samples: list[tuple[float, float]], target: float) -> dict[str, float]:
    angles = [angle for _, angle in samples]
    errors = [angle - target for angle in angles]
    return {
        "count": float(len(samples)),
        "mean": statistics.fmean(angles),
        "std": statistics.pstdev(angles) if len(angles) > 1 else 0.0,
        "max_abs_error": max(abs(err) for err in errors),
        "end_error": errors[-1],
    }


def run_trial(
    tuner: PidTuner,
    kp: float,
    kd: float,
    target: float,
    observe_s: float,
    max_angle: float,
    warmup_s: float,
) -> dict[str, float | str]:
    tuner.send_stop()
    time.sleep(0.2)
    tuner.send_target(target)
    tuner.send_pid(kp, kd)
    time.sleep(0.2)
    tuner.send_start()

    samples: list[tuple[float, float]] = []
    last_frames = -1
    unsafe_reason = ""
    start = time.time()
    end = start + observe_s
    while time.time() < end:
        snap = tuner.snapshot()
        if snap.frames != last_frames and snap.angle_deg is not None:
            last_frames = snap.frames
            now = time.time()
            if now - start >= warmup_s:
                samples.append((now, snap.angle_deg))
            if abs(snap.angle_deg - target) > max_angle:
                unsafe_reason = f"angle error over {max_angle:.1f} deg"
                break
        time.sleep(0.01)

    tuner.send_stop()
    time.sleep(0.2)

    if not samples:
        return {
            "kp": kp,
            "kd": kd,
            "samples": 0.0,
            "mean": 0.0,
            "std": 0.0,
            "max_abs_error": 999.0,
            "end_error": 999.0,
            "result": "no telemetry",
        }

    stats = summarize(samples, target)
    result = "ok" if not unsafe_reason else unsafe_reason
    if stats["std"] > max_angle * 0.55:
        result = f"oscillation std {stats['std']:.2f}"

    return {
        "kp": kp,
        "kd": kd,
        "samples": stats["count"],
        "mean": stats["mean"],
        "std": stats["std"],
        "max_abs_error": stats["max_abs_error"],
        "end_error": stats["end_error"],
        "result": result,
    }


def choose_best(results: list[dict[str, float | str]]) -> dict[str, float | str] | None:
    ok = [item for item in results if item["result"] == "ok"]
    if not ok:
        return None
    return min(ok, key=lambda item: (float(item["max_abs_error"]), float(item["std"])))


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Automatic balance car PID trial runner")
    parser.add_argument("--port", default="COM6")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--settle", type=float, default=3.0, help="seconds to observe while stopped")
    parser.add_argument("--observe", type=float, default=1.2, help="seconds per PID trial")
    parser.add_argument("--warmup", type=float, default=0.4, help="seconds ignored at trial start")
    parser.add_argument("--max-angle", type=float, default=12.0, help="stop if angle error exceeds this")
    parser.add_argument(
        "--max-start-error",
        type=float,
        default=15.0,
        help="do not start a trial if stopped mean is farther than this from target",
    )
    parser.add_argument(
        "--target",
        type=float,
        default=0.0,
        help="balance zero/target angle in degrees; default 0.0, not stopped mean",
    )
    parser.add_argument(
        "--candidates",
        default="60:2,90:3,120:4,160:5,200:6",
        help="comma-separated KP:KD list",
    )
    parser.add_argument("--log", type=Path, default=Path("logs/pid_auto_tune.csv"))
    parser.add_argument("--leave-best-running", action="store_true")
    args = parser.parse_args(argv)

    candidates: list[tuple[float, float]] = []
    for chunk in args.candidates.split(","):
        kp_text, kd_text = chunk.split(":", 1)
        candidates.append((float(kp_text), float(kd_text)))

    tuner = PidTuner(args.port, args.baud, None)
    results: list[dict[str, float | str]] = []
    try:
        tuner.open()
        tuner.send_stop()
        tuner.send_period(20)
        print(f"Connected {args.port}. Motors stopped. Reading angle for {args.settle:.1f}s...")

        stopped_samples = wait_samples(tuner, args.settle)
        if len(stopped_samples) < 10:
            print("No stable telemetry from STM32 PID stream. Keeping motors stopped.")
            return 2

        stopped_angles = [angle for _, angle in stopped_samples]
        target = args.target
        print(
            f"Stopped angle mean={statistics.fmean(stopped_angles):.2f} deg, "
            f"std={statistics.pstdev(stopped_angles):.2f} deg, target={target:.2f} deg"
        )
        if abs(statistics.fmean(stopped_angles) - target) > args.max_start_error:
            print(
                f"Start angle is too far from target by more than "
                f"{args.max_start_error:.1f} deg. Motors remain stopped."
            )
            return 3

        args.log.parent.mkdir(parents=True, exist_ok=True)
        with args.log.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=[
                    "kp",
                    "kd",
                    "samples",
                    "mean",
                    "std",
                    "max_abs_error",
                    "end_error",
                    "result",
                ],
            )
            writer.writeheader()
            for kp, kd in candidates:
                print(f"Trial kp={kp:.1f}, kd={kd:.1f}...")
                result = run_trial(tuner, kp, kd, target, args.observe, args.max_angle, args.warmup)
                results.append(result)
                writer.writerow(result)
                f.flush()
                print(
                    f"  result={result['result']}, max_err={float(result['max_abs_error']):.2f}, "
                    f"std={float(result['std']):.2f}, end_err={float(result['end_error']):.2f}"
                )
                if result["result"] != "ok":
                    print("  stopping candidate sweep after unsafe/non-OK result")
                    break

        best = choose_best(results)
        if best is None:
            print("No acceptable candidate found. Motors remain stopped.")
            return 1

        print(f"Best candidate: kp={best['kp']}, kd={best['kd']}")
        tuner.send_target(target)
        tuner.send_pid(float(best["kp"]), float(best["kd"]))
        if args.leave_best_running:
            tuner.send_start()
            print("Best candidate left running by request.")
        else:
            tuner.send_stop()
            print("Best candidate sent, motors stopped.")
        return 0
    finally:
        if not args.leave_best_running:
            try:
                tuner.send_stop()
            except Exception:
                pass
        tuner.close()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
