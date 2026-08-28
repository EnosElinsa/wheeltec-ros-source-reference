#!/bin/bash

# 配置参数
AP_NAME="Wi-Fi connection 1"
LOG_FILE="AP_start.log"
MAX_WAIT=30  # 最大等待时间（秒）
SLEEP_INTERVAL=1  # 检测间隔（秒）

# 初始化日志
echo "正在初始化..." > $LOG_FILE

# 函数：检测WiFi适配器是否可用
wait_for_wifi_adapter() {
    local count=0
    echo "正在检测WiFi适配器..." >> $LOG_FILE
    
    while [ $count -lt $MAX_WAIT ]; do
        # 获取WiFi接口
        WIFI_INTERFACE=$(nmcli device | awk '$2=="wifi" {print $1}')
        
        if [ -n "$WIFI_INTERFACE" ]; then
            echo "检测到WiFi适配器: $WIFI_INTERFACE" >> $LOG_FILE
            return 0
        fi
        
        sleep $SLEEP_INTERVAL
        ((count++))
    done
    
    echo "错误: 在${MAX_WAIT}秒内未检测到WiFi适配器" >> $LOG_FILE
    exit 1
}

# 函数：启用AP模式
enable_ap_mode() {
    echo "正在启用AP模式..." >> $LOG_FILE
    
    # 断开所有现有WiFi连接
    nmcli device disconnect "$WIFI_INTERFACE" >> $LOG_FILE 2>&1
    
    # 启用配置好的热点
    if nmcli connection up "$AP_NAME" >> $LOG_FILE 2>&1; then
        echo "AP模式已启用 - 热点名称: $AP_NAME" >> $LOG_FILE
    else
        echo "无法启用AP模式，请检查热点配置" >> $LOG_FILE
        exit 1
    fi
}

# 函数：初始化检查
initial_check() {
    # 检查是否有活跃的WiFi连接
    CURRENT_CONNECTION=$(nmcli -t -f active,ssid dev wifi | grep '^yes' | cut -d':' -f2)
    
    if [ -z "$CURRENT_CONNECTION" ]; then
        echo "未检测到活跃的WiFi连接，正在启用AP模式..." >> $LOG_FILE
        enable_ap_mode
    elif [ "$CURRENT_CONNECTION" != "$AP_NAME" ]; then
        echo "当前已连接到 $CURRENT_CONNECTION，正在强制切换到AP模式..." >> $LOG_FILE
        enable_ap_mode
    else
        echo "当前处于AP模式 (热点: $AP_NAME)" >> $LOG_FILE
    fi
}

# 主程序
wait_for_wifi_adapter
initial_check

echo "脚本执行完成" >> $LOG_FILE