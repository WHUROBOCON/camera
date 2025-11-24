#pragma once

#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <cstring>
#include <termios.h>
#include <errno.h>
#include <fcntl.h>

#define COUT_RED_START      std::cout << "\033[1;31m";
#define COUT_GREEN_START    std::cout << "\033[1;32m";
#define COUT_YELLOW_START   std::cout << "\033[1;33m";
#define COUT_BLUE_START     std::cout << "\033[1;34m";
#define COUT_PURPLE_START   std::cout << "\033[1;35m";
#define COUT_CYAN_START     std::cout << "\033[1;36m";
#define COUT_WHITE_START    std::cout << "\033[1;37m";
#define COUT_COLOR_END      std::cout << "\033[0m";

namespace com{

class my_uart
{
    private:
        const char *dev_name = "/dev/ttyUSB0";  //串口设备名
        int fd = -1;                   //串口文件描述符
        int flow_ctrl = 1;       // 流控方式 (0=无流控, 1=硬件流控 RTS/CTS)
        int databits = 8;        //数据位 
        int parity = 0;           //校验位 (0=无校验, 1=奇校验, 2=偶校验)
        int stopbits = 1;        //停止位 
        int baudrate = B115200;         //波特率 
        bool flag = 1;            //串口是否打开标志
        bool send_flag = 0;     //发送状态标志

        struct termios configs = {0};   //串口配置结构体

        int baudrate_arr[7] = {
            B115200,
            B19200,
            B9600,
            B4800,
            B2400,
            B1200,
            B300,
        };
        int flow_ctrl_arr[2] = {0, 1};
        int databits_arr[4] = {5, 6, 7, 8};
        int parity_arr[4] = {0, 1, 2};
        int stopbits_arr[2] = {1, 2};

    public:
        void UART_INIT();   //初始化
        void UART_OPEN();    //打开
        void UART_CLOSE();  //关闭

        void UART_CONFIG();     //配置
        void UART_SET_COM_NAME(std::string in_name);    //设置串口设备名
        void UART_SET_BAUDRATE(int in_baudrate);    //设置波特率
        void UART_SET_FLOW_CTRL(int in_flow_ctrl);  //设置流控方式
        void UART_SET_DATABITS(int in_databits);    //设置数据位
        void UART_SET_STOPBITS(int in_stopbits);    //设置停止位
        void UART_SET_PARITY(int in_parity);        //设置校验位

        void UART_SEND(const uint8_t* buffer_written, size_t length);   //发送数据
        void UART_SEND_CLONE(const uint8_t* buffer_written, size_t length);

        void UART_RECEIVE();    //接收数据

        my_uart();  
        ~my_uart(); 

};
}; // namespace com