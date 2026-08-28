import subprocess
from typing import Optional

class APManager:
    @staticmethod
    def get_wifi_interface() -> Optional[str]:
        """获取WIFI接口名称"""
        try:
            result = subprocess.run(
                ["nmcli", "device"],
                capture_output=True, text=True, check=True
            )
            for line in result.stdout.splitlines():
                parts = line.split()
                if len(parts) >= 2 and parts[1] == "wifi":
                    return parts[0]
            return None
        except subprocess.CalledProcessError as e:
            print(f"获取WIFI接口失败: {e}")
            return None

    @staticmethod
    def enable_ap_mode(ap_ip: str = "192.168.0.100", netmask: str = "255.255.255.0") -> bool:
        """启用AP热点模式"""
        # wifi_interface = APManager.get_wifi_interface()
        # if not wifi_interface:
        #     print("未找到可用的WIFI接口")
        #     return False
        wifi_interface = "wlan0"
        commands = [
            "sudo systemctl mask wpa_supplicant",
            "sudo systemctl stop wpa_supplicant",
            "sudo killall -9 hostapd",
            "sleep 1",
            f"sudo ip addr flush dev {wifi_interface}",
            "sleep 0.5",
            f"sudo ifconfig {wifi_interface} down",
            "sleep 1",
            f"sudo ifconfig {wifi_interface} up",
            "sleep 0.5",
            "sudo hostapd -B /etc/hostapd.conf",
            f"sudo ifconfig {wifi_interface} {ap_ip} netmask {netmask}",
            "sleep 0.5",
            "sudo systemctl stop isc-dhcp-server",
            "sudo systemctl start isc-dhcp-server"
        ]

        success = True
        try:
            for cmd in commands:
                try:
                    subprocess.run(cmd, shell=True, check=True)
                    # print(f"命令执行成功: {cmd}")
                except subprocess.CalledProcessError as e:
                    # print(f"命令执行失败(继续执行下一条): {cmd} - 错误: {e}")
                    success = False
            
            if success:
                print(f"AP热点模式已启用，IP: {ap_ip}")
            else:
                print(f"AP热点模式已启用，但部分命令执行失败，IP: {ap_ip}")
            return success
            
        except Exception as e:
            print(f"启用AP热点模式时发生意外错误: {e}")
            return False