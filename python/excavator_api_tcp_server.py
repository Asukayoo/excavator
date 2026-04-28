#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""TCP 服务端：可选键盘或手柄，发送 Servo v3 + Status v5。"""

from __future__ import annotations

import argparse
import os
import select
import socket
import struct
import sys
import termios
import time
import tty
from typing import List, Optional

SERV_MAGIC = 0x56524553
SERVO_VERSION = 3
STATUS_VERSION = 5
DEADZONE = 0.1


def clamp(v: float) -> float:
    return max(-1.0, min(1.0, float(v)))


def axis_from_stick(v: float) -> float:
    if abs(v) <= DEADZONE:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    u = (abs(v) - DEADZONE) / (1.0 - DEADZONE)
    return clamp(sign * u)


def pack_servo(joint_n: List[float], motor_n: float) -> bytes:
    return struct.pack("<II9d", SERV_MAGIC, SERVO_VERSION, *(clamp(x) for x in joint_n), clamp(motor_n))


def pack_status(motor_n: float, toggle_mask: int) -> bytes:
    return struct.pack("<IIdHH", SERV_MAGIC, STATUS_VERSION, clamp(motor_n), int(toggle_mask) & 0x07FF, 0)


def render_two_lines(status11: List[int], scalar8: List[float], first_draw: bool) -> bool:
    _ = (status11, first_draw)
    if not sys.stdout.isatty():
        return False
    line_scalar = "[SCALAR8] " + " ".join(f"{clamp(v):.3f}" for v in scalar8[:8])
    prev_line = getattr(render_two_lines, "_prev_line", None)
    if line_scalar == prev_line:
        return False
    setattr(render_two_lines, "_prev_line", line_scalar)
    sys.stdout.write("\r\033[2K" + line_scalar)
    sys.stdout.flush()
    return False


def run_keyboard(sock: socket.socket, hz: float, motor: float) -> None:
    step = 0.05
    joint = [0.0] * 8
    toggle_mask = 0
    status11 = [0] * 11
    first_draw = True
    key_axis = {
        "w": (0, +1), "s": (0, -1),
        "a": (1, +1), "d": (1, -1),
        "t": (2, +1), "g": (2, -1),
        "f": (3, +1), "h": (3, -1),
        "i": (4, +1), "k": (4, -1),
        "j": (5, +1), "l": (5, -1),
        "u": (6, +1), "o": (6, -1),
        "y": (7, +1), "p": (7, -1),
    }
    key_status = {str(i + 1): i for i in range(9)}
    key_status["0"] = 9
    key_status["-"] = 10

    # 启动时打印键位映射，便于现场核对。
    print("\n[键盘映射]")
    print("关节1: w(+), s(-)")
    print("关节2: a(+), d(-)")
    print("关节3: t(+), g(-)")
    print("关节4: f(+), h(-)")
    print("关节5: i(+), k(-)")
    print("关节6: j(+), l(-)")
    print("关节7: u(+), o(-)")
    print("关节8: y(+), p(-)")
    print("状态位切换: 1~9, 0, -")
    print("退出: q\n")

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    period = 1.0 / hz
    next_tick = time.monotonic()
    try:
        while True:
            next_tick += period
            r, _, _ = select.select([sys.stdin], [], [], 0.0)
            if r:
                ch = sys.stdin.read(1)
                if ch in ("q", "Q"):
                    return
                if ch in key_axis:
                    idx, sgn = key_axis[ch]
                    joint[idx] = clamp(joint[idx] + sgn * step)
                if ch in key_status:
                    sid = key_status[ch]
                    toggle_mask |= 1 << sid
                    if sid == 9:
                        status11[sid] = (status11[sid] + 1) & 0x3
                    else:
                        status11[sid] ^= 1
            try:
                sock.sendall(pack_servo(joint, motor))
                sock.sendall(pack_status(motor, toggle_mask))
            except (BrokenPipeError, ConnectionResetError):
                return
            first_draw = render_two_lines(status11, joint, first_draw)
            toggle_mask = 0
            sleep_t = next_tick - time.monotonic()
            if sleep_t > 0:
                time.sleep(sleep_t)
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, old)


def run_joystick(sock: socket.socket, hz: float, motor: float, motor_axis: int) -> None:
    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
    import pygame

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() < 2:
        raise RuntimeError(f"至少需要2个手柄，当前{pygame.joystick.get_count()}")
    js1 = pygame.joystick.Joystick(0)
    js2 = pygame.joystick.Joystick(1)
    js1.init()
    js2.init()
    print("\n[手柄映射]")
    print(f"手柄0: {js1.get_name()}")
    print(f"手柄1: {js2.get_name()}")
    print("轴映射(当前控制组):")
    print("  js1.axis0 -> 关节1/5, js1.axis1 -> 关节2/6")
    print("  js2.axis0 -> 关节3/7, js2.axis1 -> 关节4/8")
    print("控制组切换: button11 (前4关节 <-> 后4关节)")
    print("状态位切换: button0~button10 对应 status0~status10")
    print("  status9(电机档位)按一次 +1(mod 4)")
    print("  其他 status 为 0/1 翻转")
    if motor_axis >= 0:
        print(f"电机标量输入轴: js1.axis{motor_axis}")
    else:
        print(f"电机标量固定值: {clamp(motor):.3f}")
    print("退出: ESC 或关闭窗口\n")
    prev_btn = tuple(0 for _ in range(12))
    use_rear = False
    status11 = [0] * 11
    first_draw = True
    period = 1.0 / hz
    next_tick = time.monotonic()
    while True:
        next_tick += period
        pygame.event.pump()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                return
        ax = [
            axis_from_stick(js1.get_axis(0)),
            axis_from_stick(js1.get_axis(1)),
            axis_from_stick(js2.get_axis(0)),
            axis_from_stick(js2.get_axis(1)),
        ]
        jv = [0.0] * 8
        if use_rear:
            jv[4:8] = ax
        else:
            jv[0:4] = ax
        cur = tuple(1 if (i < js1.get_numbuttons() and js1.get_button(i)) else 0 for i in range(12))
        toggle_mask = 0
        for i in range(11):
            if prev_btn[i] == 0 and cur[i] == 1:
                toggle_mask |= 1 << i
                if i == 9:
                    status11[i] = (status11[i] + 1) & 0x3
                else:
                    status11[i] ^= 1
        if prev_btn[11] == 0 and cur[11] == 1:
            use_rear = not use_rear
        prev_btn = cur

        motor_n = clamp(motor)
        if motor_axis >= 0 and motor_axis < js1.get_numaxes():
            motor_n = axis_from_stick(js1.get_axis(motor_axis))
        try:
            sock.sendall(pack_servo(jv, motor_n))
            sock.sendall(pack_status(motor_n, toggle_mask))
        except (BrokenPipeError, ConnectionResetError):
            return
        first_draw = render_two_lines(status11, jv, first_draw)
        sleep_t = next_tick - time.monotonic()
        if sleep_t > 0:
            time.sleep(sleep_t)


def choose_input_mode(default_mode: Optional[str]) -> str:
    if default_mode in ("keyboard", "joystick"):
        return default_mode
    while True:
        print("请选择输入模式: [1] 键盘  [2] 手柄")
        choice = input("> ").strip().lower()
        if choice in ("1", "k", "key", "keyboard"):
            return "keyboard"
        if choice in ("2", "j", "joy", "joystick"):
            return "joystick"
        print("输入无效，请重新选择。")


def main() -> int:
    parser = argparse.ArgumentParser(description="TCP 服务端（键盘/手柄）")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=29753)
    parser.add_argument("--input", choices=("keyboard", "joystick"), default=None)
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument("--motor", type=float, default=0.0)
    parser.add_argument("--motor-axis", type=int, default=-1)
    args = parser.parse_args()
    if args.hz <= 0 or args.hz > 500:
        print("--hz 须在(0,500]", file=sys.stderr)
        return 1
    mode = choose_input_mode(args.input)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(1)
    conn, addr = srv.accept()
    _ = addr
    try:
        if mode == "keyboard":
            run_keyboard(conn, args.hz, args.motor)
        else:
            run_joystick(conn, args.hz, args.motor, args.motor_axis)
    except KeyboardInterrupt:
        pass
    finally:
        conn.close()
        srv.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
