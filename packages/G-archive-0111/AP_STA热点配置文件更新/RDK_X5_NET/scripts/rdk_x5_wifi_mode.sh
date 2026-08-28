#!/bin/bash

# 停止 hostapd
sudo killall -9 hostapd

# 清除 wlan0 的地址
sudo ip addr flush dev wlan0
sleep 0.5
sudo ifconfig wlan0 down
sleep 1
sudo ifconfig wlan0 up

# 重启 wpa_supplicant
sudo systemctl unmask wpa_supplicant
sudo systemctl restart wpa_supplicant

#重装wifi驱动
sudo rmmod aic8800_fdrv 
sudo modprobe aic8800_fdrv

# 连接热点,，具体操作可以查看上一章节 “无线网络”
# wifi_connect "WiFi-Test" "12345678"

