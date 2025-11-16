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

        // 启动DMA接收 - 使用较大缓冲，容忍帧非对齐
        HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&huart3, sbus_buf_, SBUS_DMA_BUF_LEN);
        if (status != HAL_OK) {
            // DMA启动失败，尝试重新启动
            HAL_UART_AbortReceive(&huart3);
            status = HAL_UARTEx_ReceiveToIdle_DMA(&huart3, sbus_buf_, SBUS_DMA_BUF_LEN);
        }
        
        if (huart3.hdmarx != NULL) {
            __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
        }
    }

    void Uart::feedHeartbeat() //喂心跳
    {
        uart_heartbeat_.last_receive_tick = xTaskGetTickCount();
        uart_heartbeat_.is_triggered = 0;
    }

    void Uart::checkHeartbeat() //检测心跳，（这个需要被扔到loop里反复检测）
    {
        if (xTaskGetTickCount() - uart_heartbeat_.last_receive_tick >= pdMS_TO_TICKS(uart_heartbeat_.timeout_ms)) {
            if (!uart_heartbeat_.is_triggered)
            {
                uart_heartbeat_.is_triggered = 1;
                triggerHeartbeat();
                resetDma();
            } else {
                // Already triggered - periodically retry DMA reset for reconnection
                static TickType_t last_retry_tick = 0;
                TickType_t current_tick = xTaskGetTickCount();
                
                // Retry every 500ms while disconnected
                if (current_tick - last_retry_tick >= pdMS_TO_TICKS(500)) {
                    resetDma();  // Keep trying to restart DMA for reconnection
                    last_retry_tick = current_tick;
                }
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
    }

    void Uart::resetDma() //重置DMA
    {
        if (!dma_resetting_) {
            dma_resetting_ = 1;
            
            // Fully reset the DMA and UART to recover from power loss
            HAL_UART_DMAStop(&huart3);  // Stop DMA completely
            HAL_UART_AbortReceive(&huart3);  // Abort any ongoing reception
            
            // Clear any error flags
            __HAL_UART_CLEAR_FLAG(&huart3, UART_CLEAR_PEF | UART_CLEAR_FEF | 
                                           UART_CLEAR_NEF | UART_CLEAR_OREF);
            
            // Restart DMA reception
            HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&huart3, sbus_buf_, SBUS_DMA_BUF_LEN);
            
            // If restart failed, try one more time
            // OPTIMIZATION: Use vTaskDelay instead of HAL_Delay (non-blocking for other tasks)
            if (status != HAL_OK) {
                vTaskDelay(pdMS_TO_TICKS(10));  // Brief delay - allows other tasks to run
                HAL_UARTEx_ReceiveToIdle_DMA(&huart3, sbus_buf_, SBUS_DMA_BUF_LEN);
            }
            
            if (huart3.hdmarx != NULL) {
                __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
            }
            
            dma_resetting_ = 0;
        }
    }

    void Uart::rxEventCallback(UART_HandleTypeDef* _huart, uint16_t _size) //回调函数
    {
        if (_huart == &huart3) 
        {
            // 有任何接收活动，先快速扫描完整帧
            bool parsed = false;
            if (_size >= SBUS_FRAME_LEN) {
                for (uint16_t i = 0; i + SBUS_FRAME_LEN <= _size; ++i) {
                    const uint8_t* f = &sbus_buf_[i];
                    const uint8_t* framePtr = nullptr;
                    uint8_t invBuf[SBUS_FRAME_LEN];
                    if (f[0] == 0x0F && f[24] == 0x00) {
                        framePtr = f;
                    } else if (f[0] == 0xF0 && f[24] == 0xFF) {
                        for (uint8_t k = 0; k < SBUS_FRAME_LEN; ++k) invBuf[k] = (uint8_t)~f[k];
                        if (invBuf[0] == 0x0F && invBuf[24] == 0x00) framePtr = invBuf;
                    }
                    if (framePtr) {
                        const uint8_t* b = &framePtr[1];
                        uint16_t ch[16];
                        ch[0]  = ((b[0]      | b[1]  << 8)                   ) & 0x07FF;
                        ch[1]  = ((b[1]>>3   | b[2]  << 5)                   ) & 0x07FF;
                        ch[2]  = ((b[2]>>6   | b[3]  << 2 | b[4] << 10)      ) & 0x07FF;
                        ch[3]  = ((b[4]>>1   | b[5]  << 7)                   ) & 0x07FF;
                        ch[4]  = ((b[5]>>4   | b[6]  << 4)                   ) & 0x07FF;
                        ch[5]  = ((b[6]>>7   | b[7]  << 1 | b[8] << 9)       ) & 0x07FF;
                        ch[6]  = ((b[8]>>2   | b[9]  << 6)                   ) & 0x07FF;
                        ch[7]  = ((b[9]>>5   | b[10] << 3)                   ) & 0x07FF;
                        ch[8]  = ((b[11]     | b[12] << 8)                   ) & 0x07FF;
                        ch[9]  = ((b[12]>>3  | b[13] << 5)                   ) & 0x07FF;
                        ch[10] = ((b[13]>>6  | b[14] << 2 | b[15] << 10)     ) & 0x07FF;
                        ch[11] = ((b[15]>>1  | b[16] << 7)                   ) & 0x07FF;
                        ch[12] = ((b[16]>>4  | b[17] << 4)                   ) & 0x07FF;
                        ch[13] = ((b[17]>>7  | b[18] << 1 | b[19] << 9)      ) & 0x07FF;
                        ch[14] = ((b[19]>>2  | b[20] << 6)                   ) & 0x07FF;
                        ch[15] = ((b[20]>>5  | b[21] << 3)                   ) & 0x07FF;
                        const uint8_t flags = framePtr[23];
                        received_bit_value_.header    = framePtr[0];
                        received_bit_value_.channel_1 = ch[0];
                        received_bit_value_.channel_2 = ch[1];
                        received_bit_value_.channel_3 = ch[2];
                        received_bit_value_.channel_4 = ch[3];
                        received_bit_value_.channel_5 = ch[4];
                        received_bit_value_.channel_6 = ch[5];
                        received_bit_value_.channel_7 = ch[6];
                        received_bit_value_.channel_8 = ch[7];
                        received_bit_value_.channel_9 = ch[8];
                        received_bit_value_.channel_10 = ch[9];
                        received_bit_value_.channel_11 = ch[10];
                        received_bit_value_.channel_12 = ch[11];
                        received_bit_value_.channel_13 = ch[12];
                        received_bit_value_.channel_14 = ch[13];
                        received_bit_value_.channel_15 = ch[14];
                        received_bit_value_.channel_16 = ch[15];
                        received_bit_value_.channel_17 = (flags & 0x01) ? 1 : 0;
                        received_bit_value_.channel_18 = (flags & 0x02) ? 1 : 0;
                        received_bit_value_.frame_lost = (flags & 0x04) ? 1 : 0;
                        received_bit_value_.fail_act   = (flags & 0x08) ? 1 : 0;
                        received_bit_value_.footer     = framePtr[24];
                        parsed = true;
                        break;
                    }
                }
            }

            if (parsed) 
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
            }
            // 始终重新启动 DMA 接收
            if (huart3.hdmarx != NULL) {
                HAL_UARTEx_ReceiveToIdle_DMA(&huart3, sbus_buf_, SBUS_DMA_BUF_LEN);
                __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
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
