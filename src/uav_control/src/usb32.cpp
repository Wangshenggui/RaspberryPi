#include "usb32.h"
#include <boost/asio.hpp>  // 串口库
#include "json.h"

using namespace boost::asio;

static USB32_Structure USB32_Struct;

// 串口相关
io_service io;
serial_port serial(io);

// 存储串口读取的数据
std::string serial_data;

void usb32_init()
{
    // 初始化串口
    serial.open("/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_205B3976534B-if00");  // 修改成你的串口
    serial.set_option(serial_port_base::baud_rate(115200));
    serial.set_option(serial_port_base::character_size(8));
    serial.set_option(serial_port_base::parity(serial_port_base::parity::none));
    serial.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    serial.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
}

// 读取串口数据的线程函数
void usb32_thread()
{
    char c;
    // 初始化串口
    usb32_init();

    while (ros::ok())
    {
        boost::system::error_code error;
        read(serial, buffer(&c, 1), error);  // 逐字节读取

        if (!error)
        {
            if (c == '\n')  // 读取完整一行数据
            {
                try 
                {
                    // 清理数据
                    std::string json_clean = serial_data;
                    json_clean.erase(0, json_clean.find_first_not_of(" \t\r\n"));
                    json_clean.erase(json_clean.find_last_not_of(" \t\r\n") + 1);

                    // ROS_INFO("Parsing JSON: %s", json_clean.c_str());

                    JSON::Value root = JSON::parse(json_clean.c_str());

                    if (root.objectValue.find("SetPosition") != root.objectValue.end()) 
                    {
                        USB32_Struct.SetPosition = root.objectValue["SetPosition"].booleanValue;
                    }

                    if (root.objectValue.find("Altitude") != root.objectValue.end()) 
                    {
                        USB32_Struct.Altitude = root.objectValue["Altitude"].numberValue;
                    }
                } 
                catch (const std::exception& e) 
                {
                    ROS_ERROR("Error parsing JSON: %s", e.what());
                }


                serial_data.clear();
            }
            else
            {
                serial_data += c;
            }
        }
    }
}

USB32_Structure get_USB32_Status(void)
{
    return USB32_Struct;
}

void usb32_send_frame(int16_t x, int16_t y, int16_t z, int16_t yaw)
{
    if (!serial.is_open())
    {
        ROS_ERROR("Serial port is not open!");
        return;
    }

    // 检查 x/y 是否在 2 字节有符号整数范围内
    if (x < -32768 || x > 32767 || y < -32768 || y > 32767 || z < -32768 || z > 32767 || yaw < -32768 || yaw > 32767)
    {
        ROS_ERROR("x/y 超出 2 字节有符号整数范围 (-32768 ~ 32767)");
        return;
    }

    // 帧头
    uint8_t frame[
        /*帧头 2 字节*/2
        +/*x 2 字节*/2
        +/*y 2 字节*/2
        +/*z 2 字节*/2
        +/*yaw 2 字节*/2
        +/*校验 1 字节*/1
        +/*帧尾 2 字节*/2
        ];
    frame[0] = 0xAA;
    frame[1] = 0x55;

    // x, y 转换为大端序字节
    frame[2] = (x >> 8) & 0xFF;
    frame[3] = x & 0xFF;
    frame[4] = (y >> 8) & 0xFF;
    frame[5] = y & 0xFF;
    frame[6] = (z >> 8) & 0xFF;
    frame[7] = z & 0xFF;
    frame[8] = (yaw >> 8) & 0xFF;
    frame[9] = yaw & 0xFF;

    // 计算校验位（x 和 y 逐字节异或）
    frame[10] = frame[2] ^ frame[3] ^ frame[4] ^ frame[5] ^ frame[6] ^ frame[7] ^ frame[8] ^ frame[9];

    // 帧尾
    frame[11] = 0xEB;
    frame[12] = 0x90;

    // 发送数据
    boost::system::error_code error;
    write(serial, buffer(frame, sizeof(frame)), error);

    if (error)
    {
        ROS_ERROR("Failed to send frame: %s", error.message().c_str());
    }
}



