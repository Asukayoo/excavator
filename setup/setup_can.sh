#!/usr/bin/env bash
set -euo pipefail

# 用法: ./setup/setup_can.sh can2
CAN_IF="${1:-can2}"
BITRATE="${2:-250000}"

# 仅允许 canx 形式，避免误操作其他网卡。
if [[ ! "${CAN_IF}" =~ ^can[0-9]+$ ]]; then
  echo "错误: 接口名需为 canx 形式，例如 can2/can1/can0" >&2
  exit 1
fi

sudo ip link set "${CAN_IF}" down
sudo ip link set "${CAN_IF}" type can bitrate "${BITRATE}"
sudo ip link set "${CAN_IF}" up

echo "已配置 ${CAN_IF} bitrate=${BITRATE}"
