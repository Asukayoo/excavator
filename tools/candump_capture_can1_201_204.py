#!/usr/bin/env python3
"""抓取 can1 上 201~204 的带时间戳 CAN 帧。"""

from __future__ import annotations

import argparse
import pathlib
import signal
import subprocess
import sys
import time
from typing import Dict, TextIO


DEFAULT_INTERFACE = "can1"
DEFAULT_IDS = ("200", "201", "202", "203")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="抓取 can1 上 201~204 的带时间戳 CAN 帧。")
    parser.add_argument("--interface", default=DEFAULT_INTERFACE, help="CAN 接口名，默认 can1。")
    parser.add_argument(
        "--ids",
        nargs="+",
        default=list(DEFAULT_IDS),
        help="要抓取的 CAN ID 列表（默认 201 202 203 204）。",
    )
    parser.add_argument("--output-dir", default=".", help="输出目录，默认当前目录。")
    return parser.parse_args()


def normalize_can_id(can_id: str) -> str:
    token = can_id.strip().upper()
    if token.startswith("0X"):
        token = token[2:]
    if not token:
        raise ValueError("空 CAN ID")
    if not all(ch in "0123456789ABCDEF" for ch in token):
        raise ValueError(f"非法 CAN ID: {can_id}")
    return token


def open_output_files(output_dir: pathlib.Path, can_ids: set[str]) -> Dict[str, TextIO]:
    output_dir.mkdir(parents=True, exist_ok=True)
    files: Dict[str, TextIO] = {}
    for can_id in sorted(can_ids):
        files[can_id] = (output_dir / f"{can_id}.txt").open("a", encoding="utf-8")
    return files


def parse_candump_line(line: str, expected_interface: str) -> str | None:
    """解析 candump 输出，返回 CAN ID（大写十六进制），失败返回 None。"""
    text = line.strip()
    if not text.startswith("("):
        return None
    right = text.find(")")
    if right < 0:
        return None
    rest = text[right + 1 :].strip()
    parts = rest.split()
    if len(parts) < 2 or parts[0] != expected_interface:
        return None
    token = parts[1].split("#", 1)[0].upper()
    if not token:
        return None
    if not all(ch in "0123456789ABCDEF" for ch in token):
        return None
    return token


def main() -> int:
    args = parse_args()
    can_ids = {normalize_can_id(can_id) for can_id in args.ids}
    out_dir = pathlib.Path(args.output_dir).resolve()
    out_files = open_output_files(out_dir, can_ids)
    hit_counter = {can_id: 0 for can_id in can_ids}
    last_print_ts = time.monotonic()

    process = subprocess.Popen(
        ["candump", "-ta", args.interface],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    def stop_handler(_: int, __) -> None:
        if process.poll() is None:
            process.terminate()

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    try:
        assert process.stdout is not None
        for line in process.stdout:
            can_id = parse_candump_line(line, args.interface)
            if can_id is None or can_id not in can_ids:
                continue
            out_files[can_id].write(line)
            out_files[can_id].flush()
            hit_counter[can_id] += 1

            now = time.monotonic()
            if now - last_print_ts >= 1.0:
                msg = " ".join(f"{key}:{hit_counter[key]}" for key in sorted(hit_counter))
                print(f"\r抓包计数 {msg}", end="", flush=True)
                last_print_ts = now
    finally:
        print()
        for file_obj in out_files.values():
            file_obj.close()
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=3)

    return process.returncode or 0


if __name__ == "__main__":
    sys.exit(main())
