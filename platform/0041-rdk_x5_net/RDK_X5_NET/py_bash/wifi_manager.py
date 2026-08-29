import subprocess
from typing import List, Dict, Optional
import time
class WifiManager:
    @staticmethod
    def enable_wifi_mode() -> bool:
        """启用WIFI客户端模式"""
        commands = [
            "sudo killall -9 hostapd",
            "sudo ip addr flush dev wlan0",
            #"sleep 0.5",
            "sudo ifconfig wlan0 down",
            #"sleep 1",
            "sudo ifconfig wlan0 up",
            "sudo systemctl unmask wpa_supplicant",
            "sudo systemctl restart wpa_supplicant",
            "sudo rmmod aic8800_fdrv",
            "sudo modprobe aic8800_fdrv"
        ]
        
        try:
            for cmd in commands:
                if cmd == "sudo ifconfig wlan0 down":
                    subprocess.run(cmd, shell=True, check=True,
                                   stdin=subprocess.DEVNULL) 
                    time.sleep(0.5)
                elif cmd == "sudo ifconfig wlan0 up":
                    subprocess.run(cmd, shell=True, check=True,
                                   stdin=subprocess.DEVNULL)
                    time.sleep(1.0)
                else:
                    subprocess.run(cmd, shell=True, check=True,
                                   stdin=subprocess.DEVNULL)
            return True
            
        except subprocess.CalledProcessError as e:
            print(f"启用WIFI模式失败: {e}")
            return False
    @staticmethod
    def scan_wifi() -> List[Dict[str, str]]:
        """扫描可用WIFI网络"""
        try:
            result = subprocess.run(
                ["sudo", "nmcli", "-t", "-f", "SSID,BARS", "device", "wifi", "list", "--rescan", "yes"],
                capture_output=True, text=True, check=True
            )
            
            networks = []
            for line in result.stdout.splitlines():
                if line:
                    ssid, strength = line.split(":")
                    networks.append({"ssid": ssid, "strength": strength})
            return networks
        except subprocess.CalledProcessError as e:
            print(f"扫描WIFI失败: {e}")
            return []

    @staticmethod
    def connect_to_wifi(ssid: str, password: str) -> bool:
        """连接指定WIFI网络"""
        try:
            # 检查是否存在对应SSID的网络配置
            check_result = subprocess.run(
                ["nmcli", "-t", "-f", "NAME", "connection", "show"],
                stdin=subprocess.DEVNULL,
                capture_output=True,
                text=True,
                check=False
            )

            # 检查输出中是否包含指定的SSID
            if ssid not in check_result.stdout:
                # 未找到对应配置，创建新的网络配置
                subprocess.run(
                    ["sudo", "nmcli", "connection", "add", "type", "wifi", "con-name", ssid, "ifname", "wlan0", "ssid", ssid, "wifi-sec.key-mgmt",  "wpa-psk",  "wifi-sec.psk", password],
                    capture_output=True, text=True, check=True
                )
            time.sleep(5.0)
            subprocess.run(
                ["sudo", "nmcli", "device", "wifi", "connect", ssid, "password", password],
                check=True
            )
            return True
        except subprocess.CalledProcessError as e:
            print(f"连接WIFI失败: {e}")
            return False
    @staticmethod
    def get_wlan0_state(text):
        lines = text.split('\n')
        wlan0_line = [line for line in lines if "wlan0" in line][0]
        state = wlan0_line.split()[2]
        return state
    
    @staticmethod
    def get_wlan0_mac():
        try:
            result = subprocess.run(
                "ifconfig wlan0 | awk '/ether/{print $2}'",
                shell=True,  # 启用 shell 解析管道
                capture_output=True,
                text=True,
                check=True
            )
            return result.stdout.strip()  # 去除多余的空格和换行
        except subprocess.CalledProcessError as e:
            print(f"获取MAC失败: {e}")
            return None

    @staticmethod
    def get_wlan0_ip():
        try:
            result = subprocess.run(
                "ip addr show wlan0 | awk '/inet /{print $2}' | cut -d/ -f1",
                shell=True,
                capture_output=True,
                text=True,
                check=True
            )
            return result.stdout.strip()
        except subprocess.CalledProcessError as e:
            print(f"获取IP失败: {e}")
            return None
        
    @staticmethod
    def get_current_mode():
        mode = ""
        try:
            result = subprocess.run(
                ["nmcli", "device", "status"],
                capture_output=True, text=True, check=True
            )
            if WifiManager.get_wlan0_state(result.stdout) == "unavailable":
                mode = "AP"
            else:
                mode = "WIFI"
            return mode
        except subprocess.CalledProcessError as e:
            print(f"获取MAC失败: {e}")

    @staticmethod
    def get_active_connections() -> List[Dict[str, str]]:
        """获取当前活动的网络连接"""
        try:
            result = subprocess.run(
                ["nmcli", "connection", "show", "--active"],
                capture_output=True, 
                text=True, 
                check=True
            )
            return result.stdout
        except subprocess.CalledProcessError as e:
            print(f"获取活动连接失败: {e}")
            return []
