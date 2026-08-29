#!/bin/bash

# 获取无线网络接口
WIFI_INTERFACE=$(nmcli device | awk '$2=="wifi" {print $1}')

# 配置参数
AP_NAME="WHEELTEC_RASPBERRY5"

# 函数：启用AP模式
enable_ap_mode() {
    echo "正在启用AP模式..." >> AP_start.log
    
    # 断开所有现有WiFi连接
    nmcli device disconnect "$WIFI_INTERFACE"
    
    # 启用配置好的热点
    if nmcli connection up "$AP_NAME"; then
        echo "AP模式已启用 - 热点名称: $AP_NAME" >> AP_start.log
    else
        echo "无法启用AP模式，请检查热点配置" >> AP_start.log
        exit 1
    fi
}

# 初始化检查
initial_check() {
    # 检查是否有活跃的WiFi连接
    CURRENT_CONNECTION=$(nmcli -t -f active,ssid dev wifi | grep '^yes' | cut -d':' -f2)
    
    if [ -z "$CURRENT_CONNECTION" ]; then
        echo "未检测到活跃的WiFi连接，正在启用AP模式..." >> AP_start.log
        enable_ap_mode
    elif [ "$CURRENT_CONNECTION" != "$AP_NAME" ]; then
        echo "当前已连接到 $CURRENT_CONNECTION，正在强制切换到AP模式..." >> AP_start.log
        enable_ap_mode
    else
        echo "当前处于AP模式 (热点: $AP_NAME)" >> AP_start.log
    fi
}

#__main__:
echo "正在初始化..." > AP_start.log
# 执行初始化检查
initial_check
#__main_end__
