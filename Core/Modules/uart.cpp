#include "uart.hpp"
//huart的引脚和灯暂定
extern "C"{
    namespace uartdriver
    {
        //记录看门狗
        uart::uart() : dma_resetting(0), invalid_data_count(0), last_time(0), time_diff(0) 
        {
            uart_watchdog.last_receive_tick = 0;
            uart_watchdog.timeout_ms = UART_WATCHDOG_TIMEOUT_MS;
            uart_watchdog.is_triggered = 0;
        }

        void uart::Init() 
        {
            
            uart_watchdog.last_receive_tick = xTaskGetTickCount(); // 使用FreeRTOS tick
            uart_watchdog.is_triggered = 0;

            // 注册回调
            HAL_UART_RegisterRxEventCallback(&huart1, StaticRxEventCallback);

            // 启动DMA接收
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)&received_bit_value, sizeof(received_bit_value));
            __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
        }

        void uart::FeedWatchdog() //喂狗
        {
            uart_watchdog.last_receive_tick = xTaskGetTickCount();
            uart_watchdog.is_triggered = 0;
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);  // 使用已定义的LED
        }

        void uart::CheckWatchdog() //检测狗，（这个需要被扔到loop里反复检测）
        {
            if (xTaskGetTickCount() - uart_watchdog.last_receive_tick >= pdMS_TO_TICKS(uart_watchdog.timeout_ms)) {
                if (!uart_watchdog.is_triggered)
                {
                    uart_watchdog.is_triggered = 1;
                    TriggerWatchdog();
                    ResetDMA();
                }
            }
        }

        uint8_t uart::validate_number() //检测接收数据大小有效性
        {
            if (received_bit_value.channel_1 >= 240 && received_bit_value.channel_1 <= 1807 &&
                received_bit_value.channel_2 >= 240 && received_bit_value.channel_2 <= 1807 &&
                received_bit_value.channel_3 >= 240 && received_bit_value.channel_3 <= 1807 &&
                received_bit_value.channel_4 >= 240 && received_bit_value.channel_4 <= 1807 &&
                received_bit_value.channel_5 >= 240 && received_bit_value.channel_5 <= 1807 &&
                received_bit_value.channel_6 >= 240 && received_bit_value.channel_6 <= 1807 &&
                received_bit_value.channel_7 >= 240 && received_bit_value.channel_7 <= 1807 &&
                received_bit_value.channel_8 >= 240 && received_bit_value.channel_8 <= 1807 &&
                received_bit_value.channel_9 >= 240 && received_bit_value.channel_9 <= 1807 &&
                received_bit_value.channel_10 >= 240 && received_bit_value.channel_10 <= 1807 &&
                received_bit_value.channel_11 >= 240 && received_bit_value.channel_11 <= 1807 &&
                received_bit_value.channel_12 >= 240 && received_bit_value.channel_12 <= 1807 &&
                received_bit_value.channel_13 >= 240 && received_bit_value.channel_13 <= 1807 &&
                received_bit_value.channel_14 >= 240 && received_bit_value.channel_14 <= 1807 &&
                received_bit_value.channel_15 >= 240 && received_bit_value.channel_15 <= 1807 &&
                received_bit_value.channel_16 >= 240 && received_bit_value.channel_16 <= 1807) {
                return 1;
            }
            return 0;
        }

        void uart::TriggerWatchdog() //触发看门狗
        {
            received_value.header = 15;
            received_value.channel_1 = 1024;
            received_value.channel_2 = 1024;
            received_value.channel_3 = 1024;
            received_value.channel_4 = 1024;
            received_value.channel_5 = 1024;
            received_value.channel_6 = 1024;
            received_value.channel_7 = 240;
            received_value.channel_8 = 240;
            received_value.channel_9 = 240;
            received_value.channel_10 = 240;
            received_value.channel_11 = 1024;
            received_value.channel_12 = 1024;
            received_value.channel_13 = 1024;
            received_value.channel_14 = 1024;
            received_value.channel_15 = 1024;
            received_value.channel_16 = 1024;
            received_value.channel_17 = 0;
            received_value.channel_18 = 0;
            received_value.frame_lost = 0;
            received_value.fail_act = 0;
            received_value.footer = 0;

            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);  // 使用已定义的LED
        }

        void uart::ResetDMA() //重置DMA
        {
            if (!dma_resetting) {
                dma_resetting = 1;
                HAL_UART_AbortReceive(&huart1);
                HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)&received_bit_value, sizeof(received_bit_value));
                __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
                dma_resetting = 0;
            }
        }

        void uart::RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) //回调函数
        {
            if (huart == &huart1) 
            {
                // 首先检查接收到的数据长度
                if (Size != sizeof(received_bit_value)) 
                {
                    // 重新启动接收
                    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)&received_bit_value, sizeof(received_bit_value));
                    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
                    return;
                }
                
                if (received_bit_value.header == 0x0F && received_bit_value.footer == 0x00) 
                {
                    if (validate_number()) 
                    {
                        FeedWatchdog();
                        invalid_data_count = 0;
                        if (received_bit_value.fail_act == 1)
                        {
                            TriggerWatchdog();
                        } 
                        else 
                        {
                            received_value.header = received_bit_value.header;
                            received_value.channel_1 = received_bit_value.channel_1;
                            received_value.channel_2 = received_bit_value.channel_2;
                            received_value.channel_3 = received_bit_value.channel_3;
                            received_value.channel_4 = received_bit_value.channel_4;
                            received_value.channel_5 = received_bit_value.channel_5;
                            received_value.channel_6 = received_bit_value.channel_6;
                            received_value.channel_7 = received_bit_value.channel_7;
                            received_value.channel_8 = received_bit_value.channel_8;
                            received_value.channel_9 = received_bit_value.channel_9;
                            received_value.channel_10 = received_bit_value.channel_10;
                            received_value.channel_11 = received_bit_value.channel_11;
                            received_value.channel_12 = received_bit_value.channel_12;
                            received_value.channel_13 = received_bit_value.channel_13;
                            received_value.channel_14 = received_bit_value.channel_14;
                            received_value.channel_15 = received_bit_value.channel_15;
                            received_value.channel_16 = received_bit_value.channel_16;
                            received_value.channel_17 = received_bit_value.channel_17;
                            received_value.channel_18 = received_bit_value.channel_18;
                            received_value.frame_lost = received_bit_value.frame_lost;
                            received_value.fail_act = received_bit_value.fail_act;
                            received_value.footer = received_bit_value.footer;
                        }
                    } 
                    else 
                    {
                        invalid_data_count++;
                        if (invalid_data_count >= 3)//失败数据大于三次就重置DMA
                        {
                            ResetDMA();
                            invalid_data_count = 0;
                        }
                    }

                    uint32_t current_time = xTaskGetTickCount();
                    time_diff = current_time - last_time;
                    last_time = current_time;
                    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)&received_bit_value, sizeof(received_bit_value));
                    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

                }
            }
        }
        
        void uart::StaticRxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) 
        {
            extern uartdriver::uart g_uart;
            g_uart.RxEventCallback(huart, Size);
        }
    }
}