#!/usr/bin/env bash
set -euo pipefail

# 远端目标机器与工作区参数（可通过环境变量覆盖）。
RDK_USER="${RDK_USER:-sunrise}"
RDK_HOST="${RDK_HOST:?Please set RDK_HOST, e.g. export RDK_HOST=192.168.127.10}"
RDK_PORT="${RDK_PORT:-22}"
REMOTE_WS="${REMOTE_WS:-/home/sunrise/rm_ws}"
# 本地工作区根目录（scripts 的上一级）。
LOCAL_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "Syncing autostart files to ${RDK_USER}@${RDK_HOST}:${REMOTE_WS}"

# 在远端创建脚本目录和 user 级 systemd 服务目录。
ssh -p "${RDK_PORT}" "${RDK_USER}@${RDK_HOST}" "mkdir -p '${REMOTE_WS}/src/scripts' ~/.config/systemd/user"

# 同步自启动相关脚本和 service 文件到远端。
rsync -avz \
  "${LOCAL_WS}/scripts/clean_build_and_start_on_rdk.sh" \
  "${LOCAL_WS}/scripts/start_autoaim_tmux.sh" \
  "${LOCAL_WS}/scripts/check_autoaim_topics.sh" \
  "${LOCAL_WS}/scripts/rm-autoaim.service" \
  "${RDK_USER}@${RDK_HOST}:${REMOTE_WS}/src/scripts/"

# 赋予脚本执行权限，安装 service 到 user 目录，并启用开机自启动。
ssh -p "${RDK_PORT}" "${RDK_USER}@${RDK_HOST}" "\
  chmod +x '${REMOTE_WS}/src/scripts/clean_build_and_start_on_rdk.sh' \
           '${REMOTE_WS}/src/scripts/start_autoaim_tmux.sh' \
           '${REMOTE_WS}/src/scripts/check_autoaim_topics.sh' && \
  cp '${REMOTE_WS}/src/scripts/rm-autoaim.service' ~/.config/systemd/user/rm-autoaim.service && \
  systemctl --user daemon-reload && \
  systemctl --user enable rm-autoaim.service"

echo "Autostart service installed."
