#!/bin/bash
# robot-controller 一键部署脚本
# 用法: bash deploy.sh
#
# 将 ROS2 功能包、OpenClaw agent 配置、skills、systemd 服务
# 一次性部署到目标机器上。
#
# 不包含用户画像和隐私文件（USER.md / MEMORY.md），
# 这些文件由用户在部署后自行创建。

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# ── 配置项（根据实际环境修改）─────────────────────────────
ROS2_WS="${ROS2_WS:-$HOME/wheeltec_ros2}"
OPENCLAW_HOME="${OPENCLAW_HOME:-$HOME/.openclaw}"
AGENT_NAME="${AGENT_NAME:-wheeltec_robot}"
# ──────────────────────────────────────────────────────────

echo ""
echo "${CYAN}========================================${NC}"
echo "${CYAN}   robot-controller 部署脚本${NC}"
echo "${CYAN}========================================${NC}"
echo ""

# 检查源目录
if [[ ! -d "$SCRIPT_DIR/robot_controller" ]]; then
    echo "${RED}错误: 找不到 robot_controller/ 目录${NC}"
    echo "${YELLOW}请在 robot-controller 项目根目录下运行此脚本${NC}"
    exit 1
fi

if [[ ! -d "$SCRIPT_DIR/openclaw" ]]; then
    echo "${RED}错误: 找不到 openclaw/ 目录${NC}"
    exit 1
fi

AGENT_DIR="$OPENCLAW_HOME/agents/$AGENT_NAME"

echo "${YELLOW}部署配置:${NC}"
echo "  ROS2 工作空间:  $ROS2_WS/src/"
echo "  OpenClaw Agent: $AGENT_DIR/"
echo "  Agent 名称:     $AGENT_NAME"
echo ""

# 定义部署项: 源 → 目标
DEPLOY_ITEMS=(
    # ROS2 功能包
    "$SCRIPT_DIR/robot_controller"        "$ROS2_WS/src/robot_controller"

    # OpenClaw agent 配置文件（不含 USER.md / MEMORY.md）
    "$SCRIPT_DIR/openclaw/SOUL.md"        "$AGENT_DIR/SOUL.md"
    "$SCRIPT_DIR/openclaw/AGENTS.md"      "$AGENT_DIR/AGENTS.md"
    "$SCRIPT_DIR/openclaw/IDENTITY.md"    "$AGENT_DIR/IDENTITY.md"
    "$SCRIPT_DIR/openclaw/TOOLS.md"       "$AGENT_DIR/TOOLS.md"
    "$SCRIPT_DIR/openclaw/HEARTBEAT.md"   "$AGENT_DIR/HEARTBEAT.md"

    # Skills
    "$SCRIPT_DIR/openclaw/skills"         "$AGENT_DIR/skills"
    "$SCRIPT_DIR/openclaw/SKILL.md"       "$AGENT_DIR/SKILL.md"

    # systemd 服务
    "$SCRIPT_DIR/scripts/fifo_watchdog.service" "$HOME/.config/systemd/user/fifo_watchdog.service"
    "$SCRIPT_DIR/scripts/fifo_watchdog.sh"      "$ROS2_WS/src/robot_controller/scripts/fifo_watchdog.sh"
)

# 确认
echo -n "${YELLOW}确认开始部署？(y/n): ${NC}"
read REPLY
if [[ "$REPLY" != "y" && "$REPLY" != "Y" ]]; then
    echo "${YELLOW}已取消${NC}"
    exit 0
fi

echo ""
echo "${GREEN}开始部署...${NC}"
echo ""

# 备份目录
BACKUP_DIR=""

success_count=0
fail_count=0
item_count=$((${#DEPLOY_ITEMS[@]} / 2))

for i in $(seq 1 $item_count); do
    src="${DEPLOY_ITEMS[$(( (i - 1) * 2 ))]}"
    dest="${DEPLOY_ITEMS[$(( (i - 1) * 2 + 1 ))]}"

    if [[ ! -e "$src" ]]; then
        echo "${RED}[$i/$item_count] 源文件不存在: $src${NC}"
        ((fail_count++))
        continue
    fi

    # 目标存在时备份
    if [[ -e "$dest" ]]; then
        if [[ -z "$BACKUP_DIR" ]]; then
            BACKUP_DIR="$HOME/robot_controller_backup.$(date +%Y%m%d_%H%M%S)"
            mkdir -p "$BACKUP_DIR"
        fi
        cp -r "$dest" "$BACKUP_DIR/$(basename "$dest")" 2>/dev/null
        rm -rf "$dest"
    fi

    # 创建目标目录
    mkdir -p "$(dirname "$dest")"

    echo "${CYAN}[$i/$item_count]${NC} $src"
    echo "    → $dest"

    if cp -r "$src" "$dest" 2>/dev/null; then
        echo "${GREEN}    ✓ 成功${NC}"
        ((success_count++))
    else
        echo "${RED}    ✗ 失败${NC}"
        ((fail_count++))
    fi
done

# 编译提示
echo ""
echo "${CYAN}========================================${NC}"
echo "${CYAN}   部署完成${NC}"
echo "${CYAN}========================================${NC}"
echo ""
echo "${GREEN}成功: $success_count${NC}  ${RED}失败: $fail_count${NC}"

if [[ -n "$BACKUP_DIR" ]]; then
    echo ""
    echo "${CYAN}备份目录: $BACKUP_DIR${NC}"
fi

echo ""
echo "${YELLOW}后续步骤:${NC}"
echo ""
echo "  1. 编译 ROS2 包:"
echo "     cd $ROS2_WS"
echo "     colcon build --packages-select interfaces"
echo "     source install/setup.bash"
echo "     colcon build --packages-select largemodel"
echo "     source install/setup.bash"
echo ""
echo "  2. 启用 fifo_watchdog 服务:"
echo "     chmod +x $ROS2_WS/src/robot_controller/scripts/fifo_watchdog.sh"
echo "     systemctl --user daemon-reload"
echo "     systemctl --user enable fifo_watchdog"
echo "     systemctl --user start fifo_watchdog"
echo ""
echo "  3. 创建用户画像文件（不含在部署包中）:"
echo "     # USER.md — 用户信息"
echo "     # MEMORY.md — 长期记忆"
echo "     # 在 $AGENT_DIR/ 下手动创建"
echo ""
echo "  4. 启动系统:"
echo "     # 终端 1: bridge"
echo "     ros2 launch largemodel robot_bridge.launch.py"
echo "     # 终端 2: OpenClaw"
echo "     openclaw gateway run"
echo ""
