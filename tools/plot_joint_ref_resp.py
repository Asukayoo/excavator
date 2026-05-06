#!/usr/bin/env python3
"""可视化前4关节的 ref/resp 状态与 rpm 规划。"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import ScalarFormatter


def load_matrix(file_path: Path) -> np.ndarray:
    """读取空格分隔矩阵，返回 shape=(N,8)。"""
    if not file_path.exists():
        raise FileNotFoundError(f"文件不存在: {file_path}")
    data = np.loadtxt(str(file_path), dtype=np.float64)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] != 8:
        raise ValueError(f"列数应为8，实际为{data.shape[1]}: {file_path}")
    return data


def load_time_axis(sample_count: int) -> tuple[np.ndarray, str]:
    """固定采样周期 20ms，横轴单位秒。"""
    frame_dt_sec = 0.02
    return np.arange(sample_count, dtype=np.float64) * frame_dt_sec, "time (s)"


def align_length(*arrays: np.ndarray) -> tuple[np.ndarray, ...]:
    """按最短长度对齐，避免日志行数不一致。"""
    min_len = min(arr.shape[0] for arr in arrays)
    return tuple(arr[:min_len] for arr in arrays)


def align_right_axis_zero(left_ax: plt.Axes, right_ax: plt.Axes, right_data: np.ndarray) -> None:
    """将右轴零点对齐到左轴零点。"""
    left_min, left_max = left_ax.get_ylim()
    left_span = left_max - left_min
    if left_span <= 1e-12:
        right_ax.set_ylim(-1.05, 1.05)
        return

    zero_ratio = (0.0 - left_min) / left_span
    zero_ratio = float(np.clip(zero_ratio, 1e-6, 1.0 - 1e-6))

    right_min_data = float(np.min(right_data))
    right_max_data = float(np.max(right_data))
    span = 2.1
    span = max(span, right_max_data / (1.0 - zero_ratio), -right_min_data / zero_ratio)
    right_min = -zero_ratio * span
    right_max = (1.0 - zero_ratio) * span
    right_ax.set_ylim(right_min, right_max)


def plot_joint_windows(log_dir: Path) -> None:
    ref_pos = load_matrix(log_dir / "ref" / "position.txt")
    ref_vel = load_matrix(log_dir / "ref" / "velocity.txt")
    ref_vel_scalar = load_matrix(log_dir / "ref" / "velocity_scalar.txt")
    ref_acc = load_matrix(log_dir / "ref" / "acceleration.txt")
    ref_plan_rpm = load_matrix(log_dir / "ref" / "plan_rpm.txt")
    ref_motor_rpm = load_matrix(log_dir / "ref" / "motor_rpm.txt")

    resp_pos = load_matrix(log_dir / "resp" / "position.txt")
    resp_vel = load_matrix(log_dir / "resp" / "velocity.txt")
    resp_acc = load_matrix(log_dir / "resp" / "acceleration.txt")

    (
        ref_pos,
        ref_vel,
        ref_vel_scalar,
        ref_acc,
        ref_plan_rpm,
        ref_motor_rpm,
        resp_pos,
        resp_vel,
        resp_acc,
    ) = align_length(
        ref_pos,
        ref_vel,
        ref_vel_scalar,
        ref_acc,
        ref_plan_rpm,
        ref_motor_rpm,
        resp_pos,
        resp_vel,
        resp_acc,
    )

    x, x_label = load_time_axis(ref_pos.shape[0])
    x = x[: ref_pos.shape[0]]
    for joint_idx in range(4):
        fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
        fig.canvas.manager.set_window_title(f"Joint {joint_idx + 1}")
        fig.suptitle(f"Joint {joint_idx + 1} 对比")

        axes[0].plot(x, ref_pos[:, joint_idx], label="ref.position", linewidth=1.2)
        axes[0].plot(x, resp_pos[:, joint_idx], label="resp.position", linewidth=1.2)
        axes[0].set_ylabel("position")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend(loc="upper right")

        axes[1].plot(x, ref_vel[:, joint_idx], label="ref.velocity", linewidth=1.2)
        axes[1].plot(x, resp_vel[:, joint_idx], label="resp.velocity", linewidth=1.2)
        axes[1].set_ylabel("velocity")
        axes[1].grid(True, alpha=0.3)
        ax1_right = axes[1].twinx()
        ax1_right.plot(
            x,
            ref_vel_scalar[:, joint_idx],
            label="ref.velocity_scalar",
            linewidth=1.0,
            linestyle="--",
            color="tab:green",
        )
        ax1_right.set_ylabel("velocity_scalar")
        align_right_axis_zero(axes[1], ax1_right, ref_vel_scalar[:, joint_idx])
        lines_left, labels_left = axes[1].get_legend_handles_labels()
        lines_right, labels_right = ax1_right.get_legend_handles_labels()
        axes[1].legend(lines_left + lines_right, labels_left + labels_right, loc="upper right")

        axes[2].plot(x, ref_acc[:, joint_idx], label="ref.acceleration", linewidth=1.2)
        axes[2].plot(x, resp_acc[:, joint_idx], label="resp.acceleration", linewidth=1.2)
        axes[2].set_ylabel("acceleration")
        axes[2].grid(True, alpha=0.3)
        axes[2].legend(loc="upper right")

        axes[3].plot(x, ref_plan_rpm[:, joint_idx], label="ref.plan_rpm", linewidth=1.2)
        axes[3].plot(x, ref_motor_rpm[:, joint_idx], label="ref.motor_rpm", linewidth=1.2)
        axes[3].set_ylabel("rpm")
        axes[3].set_xlabel(x_label)
        axes[3].grid(True, alpha=0.3)
        axes[3].legend(loc="upper right")
        # RPM 纵轴禁用科学计数法与 offset 缩写
        _rpm_yfmt = ScalarFormatter()
        _rpm_yfmt.set_scientific(False)
        _rpm_yfmt.set_useOffset(False)
        axes[3].yaxis.set_major_formatter(_rpm_yfmt)

        fig.tight_layout()

    plt.show()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="可视化 ref/resp 关节日志（前4关节）。")
    parser.add_argument(
        "--log-dir",
        default="log/20260503_155319",
        help="日志目录，需包含 ref/resp 子目录。",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    log_dir = Path(args.log_dir)
    plot_joint_windows(log_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
