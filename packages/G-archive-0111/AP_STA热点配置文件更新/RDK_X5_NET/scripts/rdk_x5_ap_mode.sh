#!/bin/bash

WIFI_INTERFACE=$(nmcli device | awk '$2=="wifi" {print $1}')
AP_IP=192.168.0.100
netmask=255.255.255.0

if [ "$(id -u)" -ne 0 ]; then
    echo "please run with sudo"
    #exec sudo "$0" "$@"
    exit 1
else
    echo "initializing ..."
fi

## check file
## /etc/hostapd.conf 
## /etc/default/isc-dhcp-server
## /etc/dhcp/dhcpd.conf

enable_ap_mode() {
    # stop wpa_supplicant
    sudo systemctl mask wpa_supplicant
    sudo systemctl stop wpa_supplicant

    # stop hostapd
    sudo killall -9 hostapd
    sleep 1

    # restart WIFI_INTERFACE:wlan0
    sudo ip addr flush dev ${WIFI_INTERFACE}
    sleep 0.5
    sudo ifconfig wlan0 down
    sleep 1
    sudo ifconfig wlan0 up
    
    # start hostapd
    sleep 0.5
    echo "[RUN] sudo hostapd -B /etc/hostapd.conf"
    sudo hostapd -B /etc/hostapd.conf
    echo "---"

    # 配置无线接口wlan0的IP和网段
    # 注意要跟/etc/dhcp/dhcpd.conf的配置保持一致
    sudo ifconfig wlan0 ${AP_IP} netmask ${netmask}
    sleep 0.5

    # 最后开启dhcp服务器
    sudo systemctl stop isc-dhcp-server
    sudo systemctl start isc-dhcp-server
}