/*
 * IP串口主动发送器 
 * 编译: gcc -o ip_serial_sender ip_serial_sender.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <time.h>
#include <signal.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/wireless.h>
#include <sys/file.h>  

// 配置参数
#define SERIAL_PORT "/dev/wheeltec_controller"
#define BAUD_RATE B115200
#define FRAME_HEADER 0x7B
#define FRAME_TAIL 0x7D
#define IP_FRAME_ID 0xFF
#define SEND_COUNT 100           // 发送次数
#define SEND_FREQUENCY_HZ 50     // 发送频率50Hz
#define SEND_INTERVAL_US (1000000 / SEND_FREQUENCY_HZ)  // 50000us = 50ms
#define IP_CHECK_INTERVAL 10      // 每5秒检查一次IP变化
#define SERIAL_WAIT_TIMEOUT 300   // 等待串口释放的最大时间（秒）

// 全局变量
volatile sig_atomic_t keep_running = 1;

// 统一数据帧结构
typedef struct {
    unsigned char buffer[11];
} UnifiedFrame;

// 网卡优先级
typedef enum {
    PRIORITY_WIRELESS = 3,  // 无线网卡优先级最高
    PRIORITY_ETHERNET = 2,  // 有线网卡次之
    PRIORITY_OTHER = 1      // 其他接口
} InterfacePriority;

// 信号处理
void signal_handler(int signum) {
    printf("\n接收到退出信号，正在停止...\n");
    keep_running = 0;
}

// BCC校验函数
unsigned char calculate_bcc(const unsigned char *data, unsigned int length) {
    unsigned char bcc = 0;
    for (unsigned int i = 0; i < length; i++) {
        bcc ^= data[i];
    }
    return bcc;
}

// 检查IP是否有效
int is_valid_ip(const char *ip) {
    unsigned int a, b, c, d;
    if (sscanf(ip, "%u.%u.%u.%u", &a, &b, &c, &d) != 4) return 0;
    if (a == 127) return 0;
    if (a == 0 && b == 0 && c == 0 && d == 0) return 0;
    if (a == 169 && b == 254) return 0;
    if (a >= 224 && a <= 239) return 0;
    if (a >= 240) return 0;
    return 1;
}

// 检查是否为虚拟接口
int is_virtual_interface(const char *ifname) {
    const char *virtual_ifaces[] = {
        "lo", "docker", "veth", "br-", "virbr", 
        "tun", "tap", "vmnet", NULL
    };
    for (int i = 0; virtual_ifaces[i] != NULL; i++) {
        if (strncmp(ifname, virtual_ifaces[i], strlen(virtual_ifaces[i])) == 0) 
            return 1;
    }
    return 0;
}

// 检测是否为无线网卡
int is_wireless_interface(const char *ifname) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return 0;
    
    struct iwreq wreq;
    memset(&wreq, 0, sizeof(wreq));
    strncpy(wreq.ifr_name, ifname, IFNAMSIZ - 1);
    
    int ret = ioctl(sock, SIOCGIWNAME, &wreq);
    close(sock);
    
    return (ret >= 0);
}

// 获取网卡优先级
InterfacePriority get_interface_priority(const char *ifname) {
    if (is_wireless_interface(ifname)) {
        return PRIORITY_WIRELESS;
    }
    
    if (strncmp(ifname, "wlan", 4) == 0 ||
        strncmp(ifname, "wl", 2) == 0 ||
        strncmp(ifname, "wifi", 4) == 0) {
        return PRIORITY_WIRELESS;
    }
    
    if (strncmp(ifname, "eth", 3) == 0 ||
        strncmp(ifname, "en", 2) == 0 ||
        strncmp(ifname, "em", 2) == 0) {
        return PRIORITY_ETHERNET;
    }
    
    return PRIORITY_OTHER;
}

// 获取当前最优IP（优先无线网卡）
int get_current_ip(char *ip_str, size_t len, char *ifname_out, size_t ifname_len) {
    struct ifaddrs *ifaddr, *ifa;
    if (getifaddrs(&ifaddr) == -1) return -1;
    
    char best_ip[INET_ADDRSTRLEN] = {0};
    char best_ifname[32] = {0};
    InterfacePriority best_priority = PRIORITY_OTHER;
    
    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == NULL) continue;
        if (ifa->ifa_addr->sa_family != AF_INET) continue;
        
        if (is_virtual_interface(ifa->ifa_name)) continue;
        
        if (!(ifa->ifa_flags & IFF_UP)) continue;
        if (ifa->ifa_flags & IFF_LOOPBACK) continue;
        
        struct sockaddr_in *sa = (struct sockaddr_in *)ifa->ifa_addr;
        char ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &sa->sin_addr, ip, sizeof(ip));
        
        if (!is_valid_ip(ip)) continue;
        
        InterfacePriority priority = get_interface_priority(ifa->ifa_name);
        
        if (priority > best_priority || best_ip[0] == '\0') {
            strncpy(best_ip, ip, sizeof(best_ip) - 1);
            strncpy(best_ifname, ifa->ifa_name, sizeof(best_ifname) - 1);
            best_priority = priority;
        }
    }
    
    freeifaddrs(ifaddr);
    
    if (best_ip[0] != '\0') {
        strncpy(ip_str, best_ip, len - 1);
        ip_str[len - 1] = '\0';
        if (ifname_out) {
            strncpy(ifname_out, best_ifname, ifname_len - 1);
            ifname_out[ifname_len - 1] = '\0';
        }
        return 0;
    }
    
    return -1;
}

// 等待获取有效IP
void wait_for_valid_ip(char *ip_str, size_t len, char *ifname_out, size_t ifname_len) {
    printf(" 等待获取有效IP（优先无线网卡）...\n");
    int attempt = 0;
    
    while (keep_running) {
        if (get_current_ip(ip_str, len, ifname_out, ifname_len) == 0) {
            InterfacePriority priority = get_interface_priority(ifname_out);
            const char *type = (priority == PRIORITY_WIRELESS) ? "无线" : 
                              (priority == PRIORITY_ETHERNET) ? "有线" : "其他";
            printf("✓ 获取到有效IP: %s (%s网卡: %s, 优先级: %d)\n", 
                   ip_str, type, ifname_out, priority);
            return;
        }
        
        if (attempt % 10 == 0) {
            printf(".");
            fflush(stdout);
        }
        attempt++;
        sleep(1);
    }
}

// 将IP字符串转换为4字节数组
void ip_string_to_bytes(const char *ip_str, unsigned char *ip_bytes) {
    unsigned int parts[4];
    if (sscanf(ip_str, "%u.%u.%u.%u", &parts[0], &parts[1], &parts[2], &parts[3]) == 4) {
        ip_bytes[0] = (unsigned char)parts[0];
        ip_bytes[1] = (unsigned char)parts[1];
        ip_bytes[2] = (unsigned char)parts[2];
        ip_bytes[3] = (unsigned char)parts[3];
    } else {
        ip_bytes[0] = 192; 
        ip_bytes[1] = 168; 
        ip_bytes[2] = 1; 
        ip_bytes[3] = 100;
    }
}

// 构建统一格式的IP帧
void build_unified_frame(const unsigned char *ip_bytes, UnifiedFrame *frame) {
    memset(frame->buffer, 0, sizeof(frame->buffer));
    
    frame->buffer[0] = FRAME_HEADER;
    frame->buffer[10] = FRAME_TAIL;
    frame->buffer[1] = IP_FRAME_ID;
    frame->buffer[2] = 0x00;
    memcpy(&frame->buffer[3], ip_bytes, 4);
    frame->buffer[7] = 0x00;
    frame->buffer[8] = 0x00;
    frame->buffer[9] = calculate_bcc(frame->buffer, 9);
}

// 配置串口
int setup_serial(int fd) {
    struct termios options;
    
    if (tcgetattr(fd, &options) != 0) {
        perror("获取串口配置失败");
        return -1;
    }
    
    cfsetispeed(&options, BAUD_RATE);
    cfsetospeed(&options, BAUD_RATE);
    
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag |= CREAD | CLOCAL;
    
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;
    
    tcflush(fd, TCIOFLUSH);
    
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        perror("设置串口配置失败");
        return -1;
    }
    
    return 0;
}

// 发送统一格式的IP帧
int send_unified_frame(int fd, const UnifiedFrame *frame) {
    ssize_t bytes_written = write(fd, frame->buffer, sizeof(frame->buffer));
    if (bytes_written < 0) {
        perror("发送数据失败");
        return -1;
    }
    tcdrain(fd);
    return 0;
}

// ========== 新增：串口占用检测和等待 ==========

/**
 * 检查串口是否被占用
 * 返回: 1-被占用, 0-未占用, -1-检查失败
 */
int check_serial_busy(const char *port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        if (errno == EBUSY) {
            return 1;  // 设备忙
        } else if (errno == ENOENT) {
            fprintf(stderr, "✗ 串口设备不存在: %s\n", port);
            return -1;
        } else if (errno == EACCES) {
            fprintf(stderr, "✗ 没有权限访问串口: %s\n", port);
            return -1;
        } else {
            // 其他错误，尝试用flock检测
        }
    } else {
        // 尝试获取排他锁
        if (flock(fd, LOCK_EX | LOCK_NB) == -1) {
            close(fd);
            if (errno == EWOULDBLOCK) {
                return 1;  // 被其他进程锁定
            }
            return 0;  // 其他情况认为未占用
        } else {
            // 成功获取锁，说明未被占用
            flock(fd, LOCK_UN);
            close(fd);
            return 0;
        }
    }
    
    if (fd >= 0) close(fd);
    return 0;
}

/**
 * 等待串口释放
 * 返回: 0-成功, -1-超时或失败
 */
int wait_for_serial_available(const char *port) {
    int wait_count = 0;
    int max_wait = SERIAL_WAIT_TIMEOUT;
    
    printf(" 检查串口状态: %s\n", port);
    
    while (keep_running && wait_count < max_wait) {
        int status = check_serial_busy(port);
        
        if (status == -1) {
            // 检查失败（设备不存在或权限问题）
            return -1;
        } else if (status == 0) {
            // 串口可用
            printf("✓ 串口可用\n");
            return 0;
        } else {
            // 串口被占用
            if (wait_count == 0) {
                printf(" 串口被占用，等待释放...\n");
            }
            
            printf("\r   等待中: %d/%d 秒", wait_count + 1, max_wait);
            fflush(stdout);
            
            sleep(1);
            wait_count++;
        }
    }
    
    if (wait_count >= max_wait) {
        printf("\n✗ 等待超时（%d秒），串口仍被占用\n", max_wait);
        return -1;
    }
    
    return -1;
}

/**
 * 尝试打开串口（带重试机制）
 * 返回: 文件描述符(成功) 或 -1(失败)
 */
int open_serial_with_retry(const char *port, int max_retries) {
    int fd = -1;
    
    for (int retry = 0; retry < max_retries && keep_running; retry++) {
        // 等待串口可用
        if (wait_for_serial_available(port) == 0) {
            // 尝试打开
            fd = open(port, O_RDWR | O_NOCTTY);
            
            if (fd >= 0) {
                // 再次尝试获取锁，确保独占
                if (flock(fd, LOCK_EX | LOCK_NB) == 0) {
                    printf("✓ 成功打开并锁定串口\n");
                    return fd;
                } else {
                    printf(" 打开成功但无法锁定，可能被其他进程占用\n");
                    close(fd);
                    fd = -1;
                }
            } else {
                fprintf(stderr, "✗ 打开串口失败: %s\n", strerror(errno));
            }
        }
        
        if (fd < 0 && retry < max_retries - 1) {
            printf(" 第 %d/%d 次重试...\n", retry + 1, max_retries);
            sleep(2);
        }
    }
    
    return -1;
}

// ========== 修改后的主动发送函数 ==========

/**
 * 主动发送IP（带串口占用检测）
 */
void active_send_ip(const char *ip_str, const char *ifname) {
    int serial_fd = -1;
    unsigned char ip_bytes[4];
    UnifiedFrame frame;
    
    printf("\n" "========================================" "\n");
    printf(" 准备发送IP: %s (网卡: %s)\n", ip_str, ifname);
    printf("========================================" "\n");
    
    // 尝试打开串口（带占用检测和等待）
    serial_fd = open_serial_with_retry(SERIAL_PORT, 3);  // 最多重试3次
    
    if (serial_fd < 0) {
        fprintf(stderr, "✗ 无法打开串口，跳过本次发送\n");
        printf("========================================" "\n\n");
        return;
    }
    
    // 配置串口
    if (setup_serial(serial_fd) < 0) {
        flock(serial_fd, LOCK_UN);  // 释放锁
        close(serial_fd);
        fprintf(stderr, "✗ 配置串口失败\n");
        printf("========================================" "\n\n");
        return;
    }
    
    // 构建IP帧
    ip_string_to_bytes(ip_str, ip_bytes);
    build_unified_frame(ip_bytes, &frame);
    
    printf("\n发送配置:\n");
    printf("  • 发送次数: %d次\n", SEND_COUNT);
    printf("  • 发送频率: %dHz\n", SEND_FREQUENCY_HZ);
    printf("  • 预计耗时: %.1f秒\n\n", (float)SEND_COUNT / SEND_FREQUENCY_HZ);
    
    struct timespec start_time, current_time;
    clock_gettime(CLOCK_MONOTONIC, &start_time);
    
    // 发送100次
    int success_count = 0;
    for (int i = 0; i < SEND_COUNT && keep_running; i++) {
        if (send_unified_frame(serial_fd, &frame) == 0) {
            success_count++;
        }
        
        // 进度显示
        if ((i + 1) % 10 == 0 || i == SEND_COUNT - 1) {
            printf("\r  进度: [%3d/%d] %3d%% (成功: %d)", 
                   i + 1, SEND_COUNT, (i + 1) * 100 / SEND_COUNT, success_count);
            fflush(stdout);
        }
        
        // 等待下一个发送周期
        if (i < SEND_COUNT - 1) {
            usleep(SEND_INTERVAL_US);
        }
    }
    
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    double elapsed = (current_time.tv_sec - start_time.tv_sec) + 
                     (current_time.tv_nsec - start_time.tv_nsec) / 1e9;
    
    printf("\n\n✓ 发送完成！\n");
    printf("  • 实际耗时: %.2f秒\n", elapsed);
    printf("  • 成功次数: %d/%d\n", success_count, SEND_COUNT);
    printf("  • 成功率: %.1f%%\n", success_count * 100.0 / SEND_COUNT);
    
    // 释放锁并关闭串口
    flock(serial_fd, LOCK_UN);
    close(serial_fd);
    printf("✓ 串口已释放\n");
    printf("========================================" "\n\n");
}

// 主控制循环
void Control_Loop() {
    char current_ip[16] = {0};
    char last_ip[16] = {0};
    char current_ifname[32] = {0};
    char last_ifname[32] = {0};
    time_t last_check = time(NULL);
    
    // 获取初始IP并发送
    if (get_current_ip(current_ip, sizeof(current_ip), 
                       current_ifname, sizeof(current_ifname)) == 0) {
        strcpy(last_ip, current_ip);
        strcpy(last_ifname, current_ifname);
        active_send_ip(current_ip, current_ifname);
    }
    
    printf(" 进入监听模式，每%d秒检查IP变化...\n\n", IP_CHECK_INTERVAL);
    
    while (keep_running) {
        sleep(1);
        
        // 每隔IP_CHECK_INTERVAL秒检查一次IP变化
        if (difftime(time(NULL), last_check) >= IP_CHECK_INTERVAL) {
            if (get_current_ip(current_ip, sizeof(current_ip), 
                             current_ifname, sizeof(current_ifname)) == 0) {
                
                // 检测IP或网卡是否变化
                if (strcmp(current_ip, last_ip) != 0 || 
                    strcmp(current_ifname, last_ifname) != 0) {
                    
                    printf(" 检测到变化:\n");
                    printf("   旧: %s (%s)\n", last_ip, last_ifname);
                    printf("   新: %s (%s)\n\n", current_ip, current_ifname);
                    
                    // 重新发送IP
                    active_send_ip(current_ip, current_ifname);
                    
                    strcpy(last_ip, current_ip);
                    strcpy(last_ifname, current_ifname);
                } else {
                    time_t now = time(NULL);
                    struct tm *tm_info = localtime(&now);
                    char time_str[32];
                    strftime(time_str, sizeof(time_str), "%H:%M:%S", tm_info);
                    printf("⏱️  [%s] IP未变化: %s (%s)\n", 
                           time_str, current_ip, current_ifname);
                }
            }
            
            last_check = time(NULL);
        }
    }
}

int main() {
    char ip_str[16];
    char ifname[32];
    
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    printf("==================================================\n");
    printf("IP串口主动发送器\n");
    printf("==================================================\n");
    printf("串口: %s\n", SERIAL_PORT);
    printf("发送策略: 检测到IP后立即发送%d次@%dHz\n", SEND_COUNT, SEND_FREQUENCY_HZ);
    printf("IP优先级: 无线网卡 > 有线网卡\n");
    printf("串口占用: 自动检测并等待释放（最长%d秒）\n", SERIAL_WAIT_TIMEOUT);
    printf("按 Ctrl+C 停止程序\n");
    printf("==================================================\n\n");
    
    // 等待获取有效IP
    wait_for_valid_ip(ip_str, sizeof(ip_str), ifname, sizeof(ifname));
    
    // 进入主控制循环
    Control_Loop();
    
    printf("\n==================================================\n");
    printf("程序已停止\n");
    printf("==================================================\n");
    
    return 0;
}
