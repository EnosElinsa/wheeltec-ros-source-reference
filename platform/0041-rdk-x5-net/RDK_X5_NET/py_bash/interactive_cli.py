import readline
from .command_executor import CommandExecutor
from .wifi_manager import WifiManager
from .ap_manager import APManager
import time

class InteractiveCLI:
    def __init__(self):
        readline.set_history_length(100)
        self.wifi_menu_active = False
    
    def show_wifi_menu(self):
        while True:
            """显示WIFI功能菜单"""
            print("\n--WIFI 功能菜单--")
            print("1. 查看网络连接状态")
            print("2. 扫描WIFI网络")
            print("3. 连接WIFI网络")
            print("4. 返回主菜单")
            choice = input("请选择WIFI操作 (1-3): ").strip()
            if choice == "1":
                res = WifiManager.get_active_connections()
                if len(res) <= 1:
                    print("无网络连接")
                else:
                    print(res)
                
            elif choice == "2":
                print("正在扫描, 连接将会断开一段时间并自动重连, 请耐心等待... ^_^")
                networks = []
                mode = WifiManager.get_current_mode()
                modified = False
                if mode == "AP":
                    WifiManager.enable_wifi_mode()
                    modified = True
                    time.sleep(1)
                networks = WifiManager.scan_wifi()
                if modified:
                    APManager.enable_ap_mode()
                print("\n可用WIFI网络:")
                for i, net in enumerate(networks, 1):
                    print(f"{i}. {net['ssid']} (信号强度: {net['strength']})")

            elif choice == "3":
                ssid = input("输入SSID: ").strip()
                password = input("输入密码: ").strip()
                mode = WifiManager.get_current_mode()
                print("主机输入以下命令, 对照MAC地址, 可查找主控新IP")
                print("sudo arp-scan --interface=wlan0 --localnet")
                print("MAC=" + WifiManager.get_wlan0_mac())
                print("IP="  + WifiManager.get_wlan0_ip())
                modified = False
                if mode == "AP":
                    WifiManager.enable_wifi_mode()
                    modified = True
                    time.sleep(2)
                if WifiManager.connect_to_wifi(ssid, password):
                    print(f"已尝试连接至 {ssid}")
                elif modified:
                    APManager.enable_ap_mode()
            elif choice == "4":
                break
            else:
                print("无效选择，请输入1-4")

    def show_ap_menu(self):
        while True:
            """显示AP热点菜单"""
            print("\n--AP热点功能菜单--")
            print("1. 启用AP热点模式")
            print("2. 返回主菜单")
            choice = input("请选择AP操作 (1-2): ").strip()
            if choice == "1":
                ap_ip = input(f"输入AP IP [直接回车使用默认值: 192.168.0.100]: ").strip() or "192.168.0.100"
                netmask = input(f"输入子网掩码 [直接回车使用默认值: 255.255.255.0]: ").strip() or "255.255.255.0"
                print("注:重启主控将恢复默认值")
                if ap_ip != "192.168.2.100":
                    print("IP地址已变化, 请重新连接, IP: {}".format(ap_ip))
                if APManager.enable_ap_mode(ap_ip, netmask):
                    print("AP热点已启动")
            elif choice == "2":
                break
            else:
                print("无效选择，请输入1-2")

    def start(self):
        """交互式命令行界面"""
        print("已进入交互模式(可运行bash指令)\n例如: \n>>> echo hello world\nhello world")
        print("←→方向键可移动光标, ↑↓方向键可切换历史命令")
        while True:
            print("--交互模式主菜单--")
            print("输入 'wifi' 进入WIFI功能菜单")
            print("输入 'ap' 进入AP热点菜单")
            print("输入 'exit' 或 'quit' 或 'Ctrl+D' 退出")
            try:
                user_input = input(">>> ").strip()
                if not user_input:
                    continue
                    
                if user_input.lower() in ('exit', 'quit'):
                    print("退出交互模式")
                    break
                elif user_input.lower() == 'wifi':
                    self.show_wifi_menu()
                elif user_input.lower() == 'ap':
                    self.show_ap_menu()
                else:
                    # 执行普通命令
                    args = user_input.split()
                    if result := CommandExecutor.execute(args):
                        CommandExecutor.print_result(args, result)
                        
            except KeyboardInterrupt:
                print("\n提示: 输入 'exit' 退出或继续输入命令")
            except EOFError:
                print("\n退出交互模式")
                break
            except Exception as e:
                print(f"错误: {e}")


