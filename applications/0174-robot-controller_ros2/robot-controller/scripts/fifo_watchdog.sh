#!/usr/bin/env bash
# FIFO 通信进程管理 — 按行读取 FIFO，处理连接注册/注销/心跳/异常事件
# 自动注册/注销 bridge，无限重连
# 语音注入已改由 bridge 直接 POST OpenClaw API，不经过此脚本
#
# FIFO 事件：
#   ready          — bridge 初始就绪，需 POST /register
#   ping           — bridge 心跳（忽略）
#   obstacle_stop  — 避障停车
#   robot_stuck    — 机器人卡住
#   sensor_warning — 传感器异常
#   EOF            — bridge 下线，等待重连

FIFO=/tmp/robot_bridge_ready
BASE_URL="http://localhost:9090"
STATUS_FILE="/tmp/robot_bridge_status.json"

log() {
    echo "[fifo-watchdog] $(date '+%H:%M:%S') $*"
}

write_status() {
    local status="$1"
    local msg="$2"
    local ts
    ts=$(date +%s)
    printf '{"status":"%s","message":"%s","timestamp":%d}\n' "$status" "$msg" "$ts" > "$STATUS_FILE"
}

call_api() {
    local method="$1"
    local path="$2"
    local data="$3"
    if [ -n "$data" ]; then
        curl -sf -X "$method" "${BASE_URL}${path}" \
            -H "Content-Type: application/json" -d "$data" 2>/dev/null
    else
        curl -sf -X "$method" "${BASE_URL}${path}" 2>/dev/null
    fi
    return $?
}

register() {
    local result
    result=$(call_api POST /register '{"client_id":"openclaw-main"}')
    if [ $? -eq 0 ] && echo "$result" | grep -q '"ok"'; then
        log "✅ OpenClaw 已注册到 bridge"
        write_status "registered" "已连接，bridge 就绪"
        return 0
    else
        log "❌ 注册失败: $result"
        write_status "error" "注册失败"
        return 1
    fi
}

unregister() {
    call_api POST /unregister '' >/dev/null 2>&1
    log "已发送注销请求"
    write_status "disconnected" "已注销"
}

log "=== 启动 FIFO 事件监听（通信进程管理）==="
write_status "init" "watchdog 启动"

while true; do
    # 等待 FIFO 就绪管道创建
    while [ ! -p "$FIFO" ]; do
        sleep 2
    done

    log "检测到 FIFO 管道，打开读端..."
    write_status "connecting" "正在连接 bridge..."

    # 按行读取 FIFO — 阻塞等待 bridge 写入
    # 一旦读到 EOF（bridge 崩溃/关闭写端）则退出循环
    connected=false
    while IFS= read -r line || [ -n "$line" ]; do
        line=$(echo "$line" | tr -d '\r\n')

        case "$line" in
            ready)
                log "收到 ready 信号，注册..."
                register && connected=true
                ;;
            obstacle_stop)
                log "⚠️ 避障停车"
                ;;
            robot_stuck)
                log "⚠️ 机器人卡住"
                ;;
            sensor_warning)
                log "⚠️ 传感器异常"
                ;;
            wake)
                log "唤醒/打断信号"
                ;;
            ping)
                # bridge 心跳，忽略
                ;;
            *)
                if [ -n "$line" ]; then
                    log "未知 FIFO 事件: '$line'"
                fi
                ;;
        esac
    done < "$FIFO"

    # 读端关闭 — FIFO 结束（EOF）
    if [ "$connected" = true ]; then
        log "⚠️ FIFO 连接断开 (EOF) — bridge 可能已下线"
        unregister
        connected=false
    else
        log "FIFO 读端关闭（未连接过），等待重试..."
    fi

    write_status "waiting" "等待 bridge 重新启动..."
    sleep 2
done
