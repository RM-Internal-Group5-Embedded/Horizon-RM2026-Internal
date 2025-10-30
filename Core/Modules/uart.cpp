
//hi
#include "uart.hpp"
//huart的引脚和灯暂定

namespace uartdriver
{
    //记录看门狗
    Uart::Uart() : dma_resetting_(0), invalid_data_count_(0), last_time_(0), time_diff_(0) 
    {
        uart_heartbeat_.last_receive_tick = 0;
        uart_heartbeat_.timeout_ms = UART_HEARTBEAT_TIMEOUT_MS;
        uart_heartbeat_.is_triggered = 0;
    }

    void Uart::init() 
    {
        // 初始化心跳检测
        uart_heartbeat_.last_receive_tick = xTaskGetTickCount(); // 使用FreeRTOS tick
        uart_heartbeat_.is_triggered = 0;

        // 检查UART句柄是否有效
        if (huart3.Instance == NULL) {
            // UART未初始化，直接返回
            return;
        }


        // 注册回调
        HAL_UART_RegisterRxEventCallback(&huart3, StaticRxEventCallback);

        // 启动DMA接收 - 添加错误检查
        HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)&received_bit_value_, sizeof(received_bit_value_));
        if (status != HAL_OK) {
            // DMA启动失败，尝试重新启动
            HAL_UART_AbortReceive(&huart3);
            status = HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)&received_bit_value_, sizeof(received_bit_value_));
        }
        
        if (huart3.hdmarx != NULL) {
            __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
        }
    }

    void Uart::feedHeartbeat() //喂心跳
    {
        uart_heartbeat_.last_receive_tick = xTaskGetTickCount();
        uart_heartbeat_.is_triggered = 0;
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);  // 使用已定义的LED
    }

    void Uart::checkHeartbeat() //检测心跳，（这个需要被扔到loop里反复检测）
    {
        if (xTaskGetTickCount() - uart_heartbeat_.last_receive_tick >= pdMS_TO_TICKS(uart_heartbeat_.timeout_ms)) {
            if (!uart_heartbeat_.is_triggered)
            {
                uart_heartbeat_.is_triggered = 1;
                triggerHeartbeat();
                resetDma();
            }
        }
    }

    uint8_t Uart::validateNumber() //检测接收数据大小有效性
    {
        if (received_bit_value_.channel_1 >= 240 && received_bit_value_.channel_1 <= 1807 &&
            received_bit_value_.channel_2 >= 240 && received_bit_value_.channel_2 <= 1807 &&
            received_bit_value_.channel_3 >= 240 && received_bit_value_.channel_3 <= 1807 &&
            received_bit_value_.channel_4 >= 240 && received_bit_value_.channel_4 <= 1807 &&
            received_bit_value_.channel_5 >= 240 && received_bit_value_.channel_5 <= 1807 &&
            received_bit_value_.channel_6 >= 240 && received_bit_value_.channel_6 <= 1807 &&
            received_bit_value_.channel_7 >= 240 && received_bit_value_.channel_7 <= 1807 &&
            received_bit_value_.channel_8 >= 240 && received_bit_value_.channel_8 <= 1807 &&
            received_bit_value_.channel_9 >= 240 && received_bit_value_.channel_9 <= 1807 &&
            received_bit_value_.channel_10 >= 240 && received_bit_value_.channel_10 <= 1807 &&
            received_bit_value_.channel_11 >= 240 && received_bit_value_.channel_11 <= 1807 &&
            received_bit_value_.channel_12 >= 240 && received_bit_value_.channel_12 <= 1807 &&
            received_bit_value_.channel_13 >= 240 && received_bit_value_.channel_13 <= 1807 &&
            received_bit_value_.channel_14 >= 240 && received_bit_value_.channel_14 <= 1807 &&
            received_bit_value_.channel_15 >= 240 && received_bit_value_.channel_15 <= 1807 &&
            received_bit_value_.channel_16 >= 240 && received_bit_value_.channel_16 <= 1807) {
            return 1;
        }
        return 0;
    }

    void Uart::triggerHeartbeat() //触发心跳
    {
        received_value_.header = 15;
        received_value_.channel_1 = 1024;
        received_value_.channel_2 = 1024;
        received_value_.channel_3 = 1024;
        received_value_.channel_4 = 1024;
        received_value_.channel_5 = 1024;
        received_value_.channel_6 = 1024;
        received_value_.channel_7 = 240;
        received_value_.channel_8 = 240;
        received_value_.channel_9 = 240;
        received_value_.channel_10 = 240;
        received_value_.channel_11 = 1024;
        received_value_.channel_12 = 1024;
        received_value_.channel_13 = 1024;
        received_value_.channel_14 = 1024;
        received_value_.channel_15 = 1024;
        received_value_.channel_16 = 1024;
        received_value_.channel_17 = 0;
        received_value_.channel_18 = 0;
        received_value_.frame_lost = 0;
        received_value_.fail_act = 0;
        received_value_.footer = 0;

        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);  // 使用已定义的LED
    }

    void Uart::resetDma() //重置DMA
    {
        if (!dma_resetting_) {
            dma_resetting_ = 1;
            HAL_UART_AbortReceive(&huart3);
            HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)&received_bit_value_, sizeof(received_bit_value_));
            __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
            dma_resetting_ = 0;
        }
    }

    void Uart::rxEventCallback(UART_HandleTypeDef* _huart, uint16_t _size) //回调函数
    {
        if (_huart == &huart3) 
        {
            // 首先检查接收到的数据长度
            if (_size != sizeof(received_bit_value_)) 
            {
                // 重新启动接收
                if (huart3.hdmarx != NULL) {
                    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)&received_bit_value_, sizeof(received_bit_value_));
                    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
                }
                return;
            }
            
            if (received_bit_value_.header == 0x0F && received_bit_value_.footer == 0x00) 
            {
                if (validateNumber()) 
                {
                    feedHeartbeat();
                    invalid_data_count_ = 0;
                    if (received_bit_value_.fail_act == 1)
                    {
                        triggerHeartbeat();
                    } 
                    else 
                    {
                        received_value_.header = received_bit_value_.header;
                        received_value_.channel_1 = received_bit_value_.channel_1;
                        received_value_.channel_2 = received_bit_value_.channel_2;
                        received_value_.channel_3 = received_bit_value_.channel_3;
                        received_value_.channel_4 = received_bit_value_.channel_4;
                        received_value_.channel_5 = received_bit_value_.channel_5;
                        received_value_.channel_6 = received_bit_value_.channel_6;
                        received_value_.channel_7 = received_bit_value_.channel_7;
                        received_value_.channel_8 = received_bit_value_.channel_8;
                        received_value_.channel_9 = received_bit_value_.channel_9;
                        received_value_.channel_10 = received_bit_value_.channel_10;
                        received_value_.channel_11 = received_bit_value_.channel_11;
                        received_value_.channel_12 = received_bit_value_.channel_12;
                        received_value_.channel_13 = received_bit_value_.channel_13;
                        received_value_.channel_14 = received_bit_value_.channel_14;
                        received_value_.channel_15 = received_bit_value_.channel_15;
                        received_value_.channel_16 = received_bit_value_.channel_16;
                        received_value_.channel_17 = received_bit_value_.channel_17;
                        received_value_.channel_18 = received_bit_value_.channel_18;
                        received_value_.frame_lost = received_bit_value_.frame_lost;
                        received_value_.fail_act = received_bit_value_.fail_act;
                        received_value_.footer = received_bit_value_.footer;
                    }
                } 
                else 
                {
                    invalid_data_count_++;
                    if (invalid_data_count_ >= 3)//失败数据大于三次就重置DMA
                    {
                        resetDma();
                        invalid_data_count_ = 0;
                    }
                }

                uint32_t current_time = xTaskGetTickCount();
                time_diff_ = current_time - last_time_;
                last_time_ = current_time;
                if (huart3.hdmarx != NULL) {
                    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)&received_bit_value_, sizeof(received_bit_value_));
                    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
                }

            }
        }
    }
}

// 桥接函数定义（在命名空间外，C风格）
extern "C" void StaticRxEventCallback(UART_HandleTypeDef* huart, uint16_t size) 
{
    extern uartdriver::Uart g_uart;
    g_uart.rxEventCallback(huart, size);
}
