#!/bin/bash

# 获取无线网络接口
WIFI_INTERFACE=$(nmcli device | awk '$2=="wifi" {print $1}')

# 配置参数
AP_NAME="Wi-Fi_connection_1"

# 检查是否以root运行
if [ "$(id -u)" -ne 0 ]; then
    echo "请使用root权限运行此脚本"
    exit 1
fi

# 函数：启用AP模式
enable_ap_mode() {
    echo "正在启用AP模式..."
    
    # 断开所有现有WiFi连接
    nmcli device disconnect "$WIFI_INTERFACE"
    
    # 启用配置好的热点
    if nmcli connection up "$AP_NAME"; then
        echo "AP模式已启用 - 热点名称: $AP_NAME"
    else
        echo "无法启用AP模式，请检查热点配置"
        exit 1
    fi
}

# 函数：扫描可用WiFi网络
scan_wifi() {
    echo "正在扫描可用WiFi网络..."
    nmcli radio wifi on
    nmcli connection down $AP_NAME
    nmcli -t -f SSID,BARS device wifi list --rescan yes
    nmcli connection up $AP_NAME
}

# 函数：连接到指定WiFi
connect_to_wifi() {
    read -p "请输入要连接的WiFi名称(SSID): " SSID
    read -p "请输入密码(如果没有密码请直接回车): " PASSWORD
    echo "---"
    echo "正在尝试连接到 $SSID..."
    echo "如果连接失败自动切换回AP热点模式, 长时间未响应则大概率连接成功"
    echo ""
    echo "查看主控新的IP地址的教程如下:"
    echo "让自己电脑与主控连接到同一个WIFI"
    echo "在自己电脑上输入ifconfig命令查看interface(例如wlan0或wlx90de803212a4)"
    echo "在自己电脑上输入以下命令查看主控新的IP地址。"
    echo "sudo arp-scan --interface=wlx90de803212a4 --localnet"

    MAC_ADDRESS=$(ifconfig "$WIFI_INTERFACE" | awk '/ether/{print $2}')
    if [ -z "$MAC_ADDRESS" ]; then
        echo "错误：无法获取接口 $WIFI_INTERFACE 的MAC地址"
        exit 1
    fi
    echo "主控WiFi接口: $WIFI_INTERFACE"
    echo "主控MAC地址: $MAC_ADDRESS"
    echo "---"

    nmcli device disconnect "$WIFI_INTERFACE"

    # 已有配置则更新密码，没有则新建
    EXISTING=$(nmcli -t -f NAME connection show | grep "^${SSID}$")
    if [ -z "$EXISTING" ]; then
        if [ -z "$PASSWORD" ]; then
            nmcli connection add type wifi con-name "$SSID" \
                ifname "$WIFI_INTERFACE" ssid "$SSID"
        else
            nmcli connection add type wifi con-name "$SSID" \
                ifname "$WIFI_INTERFACE" ssid "$SSID" \
                wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PASSWORD"
        fi
    else
        if [ -n "$PASSWORD" ]; then
            nmcli connection modify "$SSID" wifi-sec.psk "$PASSWORD"
        fi
    fi

    # 用配置连接，不依赖扫描缓存
    nmcli connection up "$SSID"
    sleep 5

    # 用实际状态验证，不依赖返回值
    STATE=$(nmcli -t -f GENERAL.STATE device show "$WIFI_INTERFACE" | cut -d':' -f2)
    if echo "$STATE" | grep -q "100 (connected)"; then
        echo "成功连接到 $SSID"
    else
        echo "连接失败，正在切换回AP模式..."
        enable_ap_mode
    fi
}
# 函数：显示当前连接状态
show_status() {
    echo "当前网络状态:"
    nmcli connection show --active
}

# 主菜单
main_menu() {
    while true; do
        echo ""
        echo "========== WiFi/AP 模式切换菜单 =========="
        echo "1. 启用AP模式 (热点: $AP_NAME)"
        echo "2. 扫描可用WiFi网络"
        echo "3. 连接到WiFi网络"
        echo "4. 显示当前连接状态"
        echo "5. 退出"
        echo "=========================================="
        read -p "请选择操作 [1-5]: " choice
        
        case $choice in
            1) enable_ap_mode ;;
            2) scan_wifi ;;
            3) connect_to_wifi ;;
            4) show_status ;;
            5) exit 0 ;;
            *) echo "无效选项，请重新输入" ;;
        esac
    done
}

# 初始化检查
initial_check() {
    # 检查是否有活跃的WiFi连接
    CURRENT_CONNECTION=$(nmcli -t -f active,ssid dev wifi | grep '^yes' | cut -d':' -f2)
    
    if [ -z "$CURRENT_CONNECTION" ]; then
        echo "未检测到活跃的WiFi连接，正在启用AP模式..."
        enable_ap_mode
    elif [ "$CURRENT_CONNECTION" != "$AP_NAME" ]; then
        echo "当前已连接到 $CURRENT_CONNECTION，正在强制切换到AP模式..."
        enable_ap_mode
    else
        echo "当前处于AP模式 (热点: $AP_NAME)"
    fi
}

#__main__:
echo "正在初始化..."
# 执行初始化检查
initial_check

# 显示主菜单
main_menu
#__main_end__
