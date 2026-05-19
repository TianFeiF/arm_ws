#!/usr/bin/env bash
# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
#
# One-shot system bootstrap for armv7 on Ubuntu 22.04 + ROS 2 Humble.
#
# Idempotent: re-running is safe. Will not undo previous configuration.
# Requires sudo (will prompt). Does NOT touch ~/.bashrc — the caller must
# source the workspace install/setup.bash themselves.
#
# Usage:
#   bash scripts/install_deps.sh            # full install
#   bash scripts/install_deps.sh --check    # only verify, no install
#   bash scripts/install_deps.sh --no-rt    # skip realtime group setup

set -euo pipefail

# ─────────────── tty helpers ───────────────
color() { printf '\033[%sm%s\033[0m\n' "$1" "$2"; }
info()  { color "1;34" "[INFO]  $*"; }
ok()    { color "1;32" "[ OK ]  $*"; }
warn()  { color "1;33" "[WARN]  $*"; }
err()   { color "1;31" "[ERR ]  $*" >&2; }
hdr()   { echo; color "1;36" "─── $* ───"; }

CHECK_ONLY=0
SETUP_RT=1
for arg in "$@"; do
    case "$arg" in
        --check) CHECK_ONLY=1 ;;
        --no-rt) SETUP_RT=0 ;;
        -h|--help)
            sed -n '/^# Usage:/,/^$/p' "$0" | sed 's/^# \{0,1\}//'
            exit 0 ;;
        *) err "未知参数: $arg"; exit 1 ;;
    esac
done

# ─────────────── 0. 平台检查 ───────────────
hdr "0. 平台检查"
. /etc/os-release 2>/dev/null || true
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
    warn "宿主非 Ubuntu 22.04 (${PRETTY_NAME:-unknown}) — 脚本未在此平台测试,继续风险自负。"
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    err "未检测到 ROS 2 Humble。先按 https://docs.ros.org/en/humble/Installation.html 安装。"
    exit 1
fi
ok "Ubuntu 22.04 + ROS 2 Humble"

# ─────────────── 1. apt 软件包 ───────────────
APT_PACKAGES=(
    # MoveIt 2 完整栈
    ros-humble-moveit
    ros-humble-moveit-planners
    ros-humble-moveit-configs-utils
    ros-humble-moveit-resources
    ros-humble-moveit-visual-tools
    ros-humble-moveit-servo
    ros-humble-moveit-setup-assistant
    ros-humble-moveit-task-constructor-core
    ros-humble-srdfdom
    ros-humble-launch-param-builder

    # ros2_control
    ros-humble-ros2-control
    ros-humble-ros2-controllers
    ros-humble-controller-manager
    ros-humble-joint-state-broadcaster
    ros-humble-joint-trajectory-controller

    # rqt 诊断可视化(armv7_diagnostics 推荐使用)
    ros-humble-rqt-robot-monitor

    # IgH EtherCAT master
    ethercat-master
    libethercat-dev
    pkg-config

    # 构建 / 工具
    python3-colcon-common-extensions
    python3-rosdep
    git
)

hdr "1. APT 软件包"
if [[ $CHECK_ONLY -eq 1 ]]; then
    missing=()
    for p in "${APT_PACKAGES[@]}"; do
        dpkg -s "$p" &>/dev/null || missing+=("$p")
    done
    if [[ ${#missing[@]} -eq 0 ]]; then
        ok "全部 ${#APT_PACKAGES[@]} 个包均已安装"
    else
        warn "缺少 ${#missing[@]} 个包: ${missing[*]}"
    fi
else
    info "apt update + install (sudo 提示在下方)"
    sudo apt update
    sudo apt install -y "${APT_PACKAGES[@]}"
    ok "apt 安装完成"
fi

# ─────────────── 2. rosdep ───────────────
hdr "2. rosdep"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    if [[ $CHECK_ONLY -eq 1 ]]; then
        warn "rosdep 未 init"
    else
        info "首次 rosdep init"
        sudo rosdep init
    fi
fi
if [[ $CHECK_ONLY -eq 0 ]]; then
    rosdep update --rosdistro humble
    ok "rosdep 已更新"
fi

# ─────────────── 3. realtime 组 ───────────────
if [[ $SETUP_RT -eq 1 ]]; then
    hdr "3. realtime 组"
    if getent group realtime &>/dev/null; then
        ok "组 realtime 已存在"
    elif [[ $CHECK_ONLY -eq 1 ]]; then
        warn "组 realtime 不存在"
    else
        info "创建 realtime 组"
        sudo groupadd -f realtime
    fi

    if id -nG "$USER" | tr ' ' '\n' | grep -qx realtime; then
        ok "$USER 已在 realtime 组"
    elif [[ $CHECK_ONLY -eq 1 ]]; then
        warn "$USER 不在 realtime 组(注销重登后生效)"
    else
        info "将 $USER 加入 realtime 组"
        sudo usermod -aG realtime "$USER"
        warn "新组成员关系需要注销/重登才生效"
    fi

    LIMITS_FILE=/etc/security/limits.d/realtime.conf
    if [[ -f $LIMITS_FILE ]] && grep -q "@realtime.*rtprio" "$LIMITS_FILE"; then
        ok "$LIMITS_FILE 已配置"
    elif [[ $CHECK_ONLY -eq 1 ]]; then
        warn "$LIMITS_FILE 未配置"
    else
        info "写入 $LIMITS_FILE"
        sudo tee "$LIMITS_FILE" >/dev/null <<'EOF'
@realtime   -   rtprio       99
@realtime   -   memlock      unlimited
@realtime   -   nice         -20
EOF
        ok "$LIMITS_FILE 已写入"
    fi
fi

# ─────────────── 4. ethercat 组 + 设备 ───────────────
hdr "4. EtherCAT 设备 / 服务"

if id -nG "$USER" | tr ' ' '\n' | grep -qx ethercat; then
    ok "$USER 已在 ethercat 组"
elif [[ $CHECK_ONLY -eq 1 ]]; then
    warn "$USER 不在 ethercat 组"
else
    info "将 $USER 加入 ethercat 组"
    sudo usermod -aG ethercat "$USER"
    warn "新组成员关系需要注销/重登才生效"
fi

UDEV_RULE=/etc/udev/rules.d/99-ethercat.rules
if [[ -f $UDEV_RULE ]]; then
    ok "udev 规则 $UDEV_RULE 存在"
else
    if [[ $CHECK_ONLY -eq 1 ]]; then
        warn "udev 规则缺失(可能依赖 libethercat 默认规则也行)"
    else
        info "写入 udev 规则"
        sudo tee "$UDEV_RULE" >/dev/null <<'EOF'
# EtherCAT Master 设备权限规则
KERNEL=="EtherCAT[0-9]*", MODE="0664", GROUP="ethercat"
EOF
        sudo udevadm control --reload-rules
        ok "udev 规则已加载"
    fi
fi

if systemctl is-enabled ethercat &>/dev/null; then
    ok "ethercat 服务 enabled"
else
    if [[ $CHECK_ONLY -eq 1 ]]; then
        warn "ethercat 服务未 enable"
    else
        info "启用 ethercat 服务"
        sudo systemctl enable ethercat
    fi
fi

if systemctl is-active ethercat &>/dev/null; then
    ok "ethercat 服务 active"
else
    if [[ $CHECK_ONLY -eq 1 ]]; then
        warn "ethercat 服务未启动(需先在 /etc/ethercat.conf 配置 MASTER0_DEVICE)"
    else
        warn "未启动 ethercat 服务 — 请先确认 /etc/ethercat.conf 里 MASTER0_DEVICE 是您的网卡 MAC,然后:"
        echo "    sudo systemctl start ethercat"
    fi
fi

# ─────────────── 5. 验证 ───────────────
hdr "5. 自检"

# pkg-config 找到 libethercat
if pkg-config --exists libethercat 2>/dev/null; then
    PC_INCDIR=$(pkg-config --variable=includedir libethercat)
    ok "pkg-config: libethercat -> include=$PC_INCDIR"
else
    err "pkg-config 找不到 libethercat — libethercat-dev 安装失败?"
fi

# MoveIt 可执行
if [[ -x /opt/ros/humble/lib/moveit_ros_move_group/move_group ]]; then
    ok "MoveIt move_group 可执行"
else
    err "MoveIt move_group 未安装"
fi

# realtime ulimit(当前 shell)
if [[ "$(ulimit -r 2>/dev/null)" -eq 99 ]]; then
    ok "ulimit -r = 99(realtime 生效)"
elif [[ $SETUP_RT -eq 1 ]]; then
    warn "ulimit -r = $(ulimit -r) — 注销/重登后才会变成 99"
fi

# ─────────────── 6. 下一步 ───────────────
hdr "6. 下一步"
cat <<EOF

  1) 如果脚本提示了"注销/重登",现在去做。
  2) 回到该工作区:
       cd $(pwd)
       source /opt/ros/humble/setup.bash
       colcon build --symlink-install
       source install/setup.bash
  3) 干跑(不连真硬件):
       ros2 launch armv7_bringup arm.launch.py use_fake_hardware:=true use_rt:=false
  4) 上真硬件:
       sudo systemctl start ethercat       # 如未启动
       ros2 launch armv7_bringup arm.launch.py

  常见报错见 docs/troubleshooting.md。
EOF
