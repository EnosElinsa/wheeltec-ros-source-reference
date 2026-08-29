#!/bin/bash

WIFI_INTERFACE=$(nmcli device | awk '$2=="wifi" {print $1}')
AP_IP=192.168.0.100
netmask=255.255.255.0
log_file="/home/wheeltec/RDK_X5_NET/scripts/AP_start.log"


if [ "$(id -u)" -ne 0 ]; then
    echo "tring to run with root"
    exec sudo "$0" "$@" >> ${log_file}
    echo "finish"
    exit 1
else
    echo "$(date +"%Y-%m-%d %H:%M:%S")" > ${log_file}
    echo "init" >> ${log_file}
fi
## sudo visudo
## <whoami> ALL=(ALL) NOPASSWD: <file_path>
## check file
## /etc/hostapd.conf 
## /etc/default/isc-dhcp-server
## /etc/dhcp/dhcpd.conf

# stop wpa_supplicant
sudo systemctl mask wpa_supplicant >> ${log_file}
sudo systemctl stop wpa_supplicant >> ${log_file}

# stop hostapd
sudo killall -9 hostapd
sleep 1

# restart WIFI_INTERFACE:wlan0
sudo ip addr flush dev ${WIFI_INTERFACE} >> ${log_file}
sleep 0.5
sudo ifconfig wlan0 down >> ${log_file} 
sleep 1
sudo ifconfig wlan0 up >> ${log_file}

# start hostapd
sleep 0.5
echo "[RUN] sudo hostapd -B /etc/hostapd.conf" >> ${log_file}
sudo hostapd -B /etc/hostapd.conf >> ${log_file}
echo "---" >> ${log_file}

# 配置无线接口wlan0的IP和网段
# 注意要跟/etc/dhcp/dhcpd.conf的配置保持一致
sudo ifconfig wlan0 ${AP_IP} netmask ${netmask} >> ${log_file}
sleep 0.5

# 最后开启dhcp服务器
sudo systemctl stop isc-dhcp-server >> ${log_file}
sudo systemctl start isc-dhcp-server >> ${log_file}
sudo systemctl enable isc-dhcp-server >> ${log_file}

