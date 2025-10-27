#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <cstring>
#include <cstdio>
#include "gpio.h"
#include "main.h"
#define UART_WATCHDOG_TIMEOUT_MS 5000
namespace uartdriver
{
    //这个是保存看门狗（一种重连机制）的数据接收时间，接收超时时长和flag的结构体
    typedef struct 
    {
        uint32_t last_receive_tick;
        uint32_t timeout_ms;
        uint8_t is_triggered;
    } UARTWatchdog;
    //这个是位域的解码结构体
    typedef struct 
    {
        uint16_t header : 8;      // 8 bits
        uint16_t channel_1 : 11;  // 11 bits
        uint16_t channel_2 : 11;  // 11 bits
        uint16_t channel_3 : 11;  // 11 bits
        uint16_t channel_4 : 11;  // 11 bits
        uint16_t channel_5 : 11;  // 11 bits
        uint16_t channel_6 : 11;  // 11 bits
        uint16_t channel_7 : 11;  // 11 bits
        uint16_t channel_8 : 11;  // 11 bits
        uint16_t channel_9 : 11;  // 11 bits
        uint16_t channel_10 : 11; // 11 bits
        uint16_t channel_11 : 11; // 11 bits
        uint16_t channel_12 : 11; // 11 bits
        uint16_t channel_13 : 11; // 11 bits
        uint16_t channel_14 : 11; // 11 bits
        uint16_t channel_15 : 11; // 11 bits
        uint16_t channel_16 : 11; // 11 bits
        uint16_t channel_17 : 1;  // 1 bit
        uint16_t channel_18 : 1;  // 1 bit
        uint16_t frame_lost : 1;  // 1 bit
        uint16_t fail_act : 5;    // 5 bits
        uint16_t footer : 8;      // 8 bits
    } __attribute__((packed, aligned(1))) ReceivedBitValue;
    //这个是数据收取结构体
    struct ReceivedValue 
    {
        uint16_t header;
        uint16_t channel_1;
        uint16_t channel_2;
        uint16_t channel_3;
        uint16_t channel_4;
        uint16_t channel_5;
        uint16_t channel_6;
        uint16_t channel_7;
        uint16_t channel_8;
        uint16_t channel_9;
        uint16_t channel_10;
        uint16_t channel_11;
        uint16_t channel_12;
        uint16_t channel_13;
        uint16_t channel_14;
        uint16_t channel_15;
        uint16_t channel_16;
        uint16_t channel_17;
        uint16_t channel_18;
        uint16_t frame_lost;
        uint16_t fail_act;
        uint16_t footer;
    };

    class uart
    {
    private:
        UARTWatchdog uart_watchdog;//结构体变量
        ReceivedBitValue received_bit_value;//结构体变量
        ReceivedValue received_value;//结构体变量
        uint8_t dma_resetting;//重置DMA的flag
        uint8_t invalid_data_count;//错误次数
        uint32_t last_time;//上次接收时间
        uint32_t time_diff;//接收时间差
        // 私有方法
        uint8_t validate_number();
        void TriggerWatchdog();
        void ResetDMA();

    public:
        uart();
        void Init();
        void FeedWatchdog();
        void CheckWatchdog();
        void RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
        // 静态回调桥接（HAL需要C风格回调）
        static void StaticRxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
        
        // 公共访问方法
        const ReceivedValue& GetReceivedValue() const { return received_value; }
        bool IsDataValid() const { return uart_watchdog.is_triggered == 0; }
    };
}