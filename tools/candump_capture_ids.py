#!/usr/bin/env python3
"""从 candump 抓取指定 CAN ID，并分别保存到文本文件。"""

from __future__ import annotations

import argparse
import pathlib
import signal
import subprocess
import sys
import time
from typing import Dict, TextIO


DEFAULT_IDS = ("18F021F6", "18F022F6", "18F023F6")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="抓取 candump 中指定 CAN ID 的报文，并分别写入 txt 文件。"
    )
    parser.add_argument(
        "--interface",
        default="can2",
        help="CAN 接口名，默认 can2。",
    )
    parser.add_argument(
        "--ids",
        nargs="+",
        default=list(DEFAULT_IDS),
        help="要抓取的 CAN ID 列表（空格分隔）。",
    )
    parser.add_argument(
        "--output-dir",
        default=".",
        help="输出目录，默认当前目录。",
    )
    return parser.parse_args()


def open_output_files(output_dir: pathlib.Path, can_ids: set[str]) -> Dict[str, TextIO]:
    output_dir.mkdir(parents=True, exist_ok=True)
    files: Dict[str, TextIO] = {}
    for can_id in sorted(can_ids):
        file_path = output_dir / f"{can_id}.txt"
        files[can_id] = file_path.open("a", encoding="utf-8")
    return files


def parse_candump_line(line: str, expected_interface: str) -> str | None:
    """解析 candump 行，返回匹配到的 CAN ID（大写），失败返回 None。"""
    text = line.strip()
    if not text.startswith("("):
        return None
    right = text.find(")")
    if right < 0:
        return None

    rest = text[right + 1 :].strip()
    if not rest:
        return None
    parts = rest.split()
    if len(parts) < 2:
        return None
    if parts[0] != expected_interface:
        return None

    # 常见格式：
    # 1) can2 18F021F6 [8] ...
    # 2) can2 18F021F6#11223344
    # 3) can2 18F021F6##...
    id_token = parts[1].split("#", 1)[0]
    if not id_token:
        return None
    if not all(ch in "0123456789abcdefABCDEF" for ch in id_token):
        return None
    return id_token.upper()


def main() -> int:
    args = parse_args()
    ids = {can_id.upper() for can_id in args.ids}
    output_dir = pathlib.Path(args.output_dir).resolve()
    out_files = open_output_files(output_dir, ids)
    hit_counter = {can_id: 0 for can_id in ids}
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
            if can_id is None:
                continue
            if can_id not in ids:
                continue
            out_files[can_id].write(line)
            out_files[can_id].flush()
            hit_counter[can_id] += 1

            now = time.monotonic()
            if now - last_print_ts >= 1.0:
                msg = " ".join(f"{k}:{hit_counter[k]}" for k in sorted(hit_counter))
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
