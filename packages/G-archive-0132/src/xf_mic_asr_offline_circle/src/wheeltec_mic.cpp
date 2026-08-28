#include "mic_com.h"

struct ProtocolParser {
    ParseState state = ParseState::WAIT_HEADER;
    std::vector<uint8_t> buffer;
    uint16_t expected_length = 0;
    uint16_t message_id = 0;
    uint8_t message_type = 0;

    void reset() {
        state = ParseState::WAIT_HEADER;
        buffer.reserve(MAX_FRAME_SIZE); 
        buffer.clear();
        expected_length = 0;
        message_id = 0;
        message_type = 0;
    }
};

bool parse_from_json(const std::string& json_str, int& angle) {
    Json::Value root;
    Json::CharReaderBuilder reader;
    std::unique_ptr<Json::CharReader> json_reader(reader.newCharReader());
    std::string errors;

    // 解析外层JSON
    if (!json_reader->parse(json_str.c_str(), json_str.c_str() + json_str.size(), &root, &errors)) {
        //ROS_WARN("Outer JSON parse error: %s", errors.c_str());
        return false;
    }

    // 检查外层字段
    if (!root.isMember("content") || !root["content"].isObject()) {
        //ROS_WARN("Missing 'content' object in outer JSON");
        return false;
    }

    const Json::Value& content = root["content"];

    // 提取info字段（字符串类型的嵌套JSON）
    if (!content.isMember("info") || !content["info"].isString()) {
        //ROS_WARN("Missing or invalid 'info' field");
        return false;
    }
    std::string inner_json_str = content["info"].asString();

    // 解析内层JSON
    Json::Value inner_root;
    if (!json_reader->parse(inner_json_str.c_str(), inner_json_str.c_str() + inner_json_str.size(), &inner_root, &errors)) {
        //ROS_WARN("Inner JSON parse error: %s", errors.c_str());
        return false;
    }

    // 提取ivw.angle字段
    if (!inner_root.isMember("ivw") || 
        !inner_root["ivw"].isObject() || 
        !inner_root["ivw"].isMember("angle") || 
        !inner_root["ivw"]["angle"].isNumeric()) {
        //ROS_WARN("Missing or invalid 'ivw.angle' field");
        return false;
    }

    angle = inner_root["ivw"]["angle"].asInt();
    return true;
}

uint16_t parse_little_endian(const uint8_t* data) {
    return (data[1] << 8) | data[0];
}

uint8_t calculate_checksum(const std::vector<uint8_t>& data) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < data.size() - 1; ++i) {
        checksum += data[i];
    }
    return static_cast<uint8_t>(~checksum + 1);
}

// 串口资源管理类
SerialPort::SerialPort(const char* port, int baud) : fd(-1) {
    const int max_retries = 10;  // 增加最大重试次数到10次
    int retry_count = 0;
    while (retry_count++ < max_retries) {
        fd = open(port, O_RDWR | O_NOCTTY);
        if (fd >= 0) {
            // 清除O_NONBLOCK标志，改为阻塞模式
            int flags = fcntl(fd, F_GETFL, 0);
            fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
            
            if (configure(baud)) {  // 配置成功
                printf(">>>>>成功打开麦克风设备\n");
                printf(">>>>>唤醒词:\"%s!\"\n", awake_words_str.c_str());
                for (int i = 0; i < 2; ++i){
                    std_msgs::Int8 voice_flag_msg;
                    voice_flag_msg.data = 1;
                    voice_flag_pub.publish(voice_flag_msg);
                    //printf(">>>>>voice_flag_msg:%d\n", voice_flag_msg.data);
                    sleep(1.0);
                }
                check_connection(); // 执行初始连接检查
                return;
            }
            close(fd);
        }
        
        // 打印重试信息
        printf("打开串口 %s 失败，重试中 (%d/%d)...\n", port_name.c_str(), retry_count, max_retries);
        printf(">>>>>无法打开麦克风设备，尝试重新连接进行测试\n");
        
        // 增加重试等待时间
        sleep(1); // 增加到1秒重试间隔
    }
    
    // 达到最大重试次数，抛出异常
    throw std::runtime_error("Open serial port failed after maximum retries");   
}

SerialPort::~SerialPort() {
    if (fd >= 0) {
        close(fd);
        ROS_INFO("Serial port closed");
    }
}

bool SerialPort::check_connection() {
        uint8_t dummy;
        int ret = read(fd, &dummy, 1);
        if (ret == -1 && errno == EAGAIN) {
            // 设备无数据但连接正常
            return true;
        }
        return ret >= 0;
    }

ssize_t SerialPort::read_nonblock(unsigned char* buf, size_t size) {
    struct pollfd fds = {fd, POLLIN, 0};
    int ret = poll(&fds, 1, 10); // 10ms超时
    if (ret > 0 && (fds.revents & POLLIN)) {
        return read(fd, buf, size);
    }
    return -1;
}

bool SerialPort::configure(int baud) {
    struct termios tio;
    tcgetattr(fd, &tio);

    cfmakeraw(&tio);
    cfsetspeed(&tio, baud);

    tio.c_cflag &= ~PARENB;   // 无奇偶校验
    tio.c_cflag &= ~CSTOPB;   // 1位停止位
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;       // 8位数据位

    tio.c_cc[VTIME] = 0;     // 非规范模式读取超时
    tio.c_cc[VMIN] = 0;

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        throw std::runtime_error("Failed to configure serial port");
    }
    return tcsetattr(fd, TCSANOW, &tio) == 0;
}

bool EnhancedSerial::reliable_write(const uint8_t* data, size_t len, int retries=3) {
    while(retries-- > 0) {
        ssize_t written = write(fd, data, len);
        if(written == static_cast<ssize_t>(len)) return true;
        usleep(100000); // 100ms重试间隔
    }
    return false;
}

MicProcessor::MicProcessor(ros::NodeHandle& nh) : nh(nh) {
    awake_flag_pub = nh.advertise<std_msgs::Int8>("/awake_flag", 1);
    voice_words_pub = nh.advertise<std_msgs::String>("/voice_words", 1);
    pub_awake_angle = nh.advertise<std_msgs::Int32>("/mic/awake/angle", 1);
}

void MicProcessor::process_awake_event(int angle) {
    std_msgs::Int32 angle_msg;
    angle_msg.data = angle;
    pub_awake_angle.publish(angle_msg);

    std_msgs::Int8 flag_msg;
    flag_msg.data = 1;
    awake_flag_pub.publish(flag_msg);

    std_msgs::String voice_msg;
    voice_msg.data = "小车唤醒";
    voice_words_pub.publish(voice_msg);
}

void process_valid_frame(const std::vector<uint8_t>& frame,
                        uint8_t message_type, 
                        uint16_t message_id,
                        MicProcessor& processor) {
    switch (message_type) {
        case 0x01: // 握手请求
            //ROS_INFO("Handshake request received, ID: %d", message_id);
            break;
        case 0x04:{ // 设备消息
            //ROS_INFO("Device message received, ID: %d", message_id);
            // 提取JSON数据
            uint16_t data_length = (frame[3] << 8) | frame[4];
            const uint8_t* json_start = frame.data() + 7;
            std::string json_str(json_start, json_start + data_length);

            // 解析JSON
            int angle;
            if (!parse_from_json(json_str, angle)) {
                //ROS_WARN("Failed to parse angle from JSON");
                break;
            }
            processor.process_awake_event(angle);
            ROS_INFO("Published angle: %d", angle);
            break;
        }
        case 0x05: // 主控消息
            ROS_INFO("Control message received, ID: %d", message_id);
            break;
        case 0xFF: // 确认消息
            ROS_INFO("Ack message received, ID: %d", message_id);
            break;
        default:
            ROS_WARN("Unknown message type: 0x%02X", message_type);
            break;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "wheeltec_mic");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(100);

    voice_flag_pub = nh.advertise<std_msgs::Int8>("/voice_flag", 1);

    ProtocolParser parser;
    MicProcessor mic_processor(nh);
    
    // 最外层循环，确保即使发生异常也能尝试重新连接
    while(ros::ok()) {
        try {
            nh.param("usart_port_name", port_name, std::string("/dev/wheeltec_mic"));
            
            ROS_INFO("尝试连接串口设备: %s", port_name.c_str());
            
            // 初始化硬件接口
            EnhancedSerial serial(port_name.c_str(), B115200);

            // 主循环控制参数
            constexpr size_t READ_BUFFER_SIZE = 512;
            std::array<uint8_t, READ_BUFFER_SIZE> read_buffer;
            auto last_stat_print = ros::Time::now();
            auto last_connection_check = ros::Time::now();

            while(ros::ok()) {
                // 定期检查连接状态
                auto now = ros::Time::now();
                if ((now - last_connection_check).toSec() >= 5.0) { // 每5秒检查一次连接
                    if (!serial.check_connection()) {
                        ROS_WARN("串口连接丢失，尝试重新连接...");
                        throw std::runtime_error("Connection lost");
                    }
                    last_connection_check = now;
                }

                ssize_t bytes_read = serial.read_nonblock(read_buffer.data(), read_buffer.size());
                if(bytes_read > 0) {
                    for(ssize_t i=0; i<bytes_read; ++i) {
                        const uint8_t byte = read_buffer[i];

                        switch (parser.state) {
                            case ParseState::WAIT_HEADER:
                                if (byte == FRAME_HEADER) {
                                    parser.reset();
                                    parser.buffer.push_back(byte);
                                    parser.state = ParseState::CHECK_USER_ID;
                                }
                                break;

                            case ParseState::CHECK_USER_ID:
                                if (byte == USER_ID) {
                                    parser.buffer.push_back(byte);
                                    parser.state = ParseState::PARSE_TYPE;
                                } else {
                                    parser.reset();
                                }
                                break;

                            case ParseState::PARSE_TYPE:
                                parser.buffer.push_back(byte);
                                parser.message_type = byte;
                                parser.state = ParseState::PARSE_LENGTH;
                                break;

                            case ParseState::PARSE_LENGTH:
                                parser.buffer.push_back(byte);
                                if (parser.buffer.size() == 5) {
                                    parser.expected_length = parse_little_endian(&parser.buffer[3]);
                                    parser.state = ParseState::PARSE_ID;
                                }
                                break;

                            case ParseState::PARSE_ID:
                                parser.buffer.push_back(byte);
                                if (parser.buffer.size() == 7) {
                                    parser.message_id = parse_little_endian(&parser.buffer[5]);
                                    parser.state = ParseState::COLLECT_DATA;
                                }
                                break;

                            case ParseState::COLLECT_DATA:
                                parser.buffer.push_back(byte);
                                if (parser.buffer.size() == 7 + parser.expected_length) {
                                    parser.state = ParseState::VERIFY_CHECKSUM;
                                }
                                break;

                            case ParseState::VERIFY_CHECKSUM:
                                parser.buffer.push_back(byte);
                                if (calculate_checksum(parser.buffer) == byte) {
                                    process_valid_frame(parser.buffer, parser.message_type, parser.message_id, mic_processor);
                                } else {
                                    //ROS_WARN("Checksum mismatch");
                                }
                                parser.reset();
                                break;
                        }
                    }
                }

                ros::spinOnce();
                loop_rate.sleep();
            }
        }
        catch(const std::system_error& e) {
            ROS_ERROR("系统错误: %s (代码 %d)，将在5秒后重试...", e.what(), e.code().value());
            sleep(5); // 等待5秒后重试
        }
        catch(const std::exception& e) {
            ROS_ERROR("未处理的异常: %s，将在5秒后重试...", e.what());
            sleep(5); // 等待5秒后重试
        }
    }
    
    return 0;
}