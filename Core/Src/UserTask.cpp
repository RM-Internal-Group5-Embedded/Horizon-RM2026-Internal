/**
 * @file UserTask.cpp
 * @author JIANG Yicheng  RM2023 (EthenJ@outlook.sg)
 * @brief Create user tasks with cpp support
 * @version 0.1
 * @date 2022-08-20
 *
 * @copyright Copyright (c) 2022
 */
#include "AppConfig.h" // Include our customized configuration
#include "FreeRTOS.h"
#include "gpio.h"
#include "main.h"
#include "task.h"
#include "uart.hpp"  // 包含您的UART驱动头文件


// 全局UART实例
uartdriver::uart g_uart;

// 任务栈和控制块
// UART任务栈 - 需要处理复杂的数据结构和DMA操作，使用更大的栈
StackType_t uxUartTaskStack[configMINIMAL_STACK_SIZE * 4];  // 512字节栈
StaticTask_t xUartTaskTCB;

// 数据处理任务栈 - 用于处理接收到的数据
StackType_t uxDataProcessTaskStack[configMINIMAL_STACK_SIZE * 3];  // 384字节栈
StaticTask_t xDataProcessTaskTCB;
// UART任务函数 - 负责UART初始化和看门狗检查
void uartTask(void *pvPara) {
  // 初始化UART
  g_uart.Init();
  
  while (true) {
    // 检查看门狗（这个方法需要在循环中定期调用）
    g_uart.CheckWatchdog();
    
    // 延时，避免任务占用过多CPU
    vTaskDelay(pdMS_TO_TICKS(50));  // 50ms延时，看门狗检查不需要太频繁
  }
}

// 数据处理任务函数 - 负责显示接收状态
void dataProcessTask(void *pvPara) {
  while (true) {
    // 检查数据是否有效
    if (g_uart.IsDataValid()) {
      // 显示接收状态 - LED1亮表示数据有效
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    } else {
      // 数据无效时LED1熄灭
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    }
    
    // 延时，避免任务占用过多CPU
    vTaskDelay(pdMS_TO_TICKS(100));  // 100ms延时
  }
}

/**
 * @brief Intialize all the drivers and add task to the scheduler
 * @todo  Add your own task in this file
 */
void startUserTasks() {
  // 创建UART处理任务 - 高优先级，负责UART通信和看门狗检查
  xTaskCreateStatic(uartTask, "UART_Task", configMINIMAL_STACK_SIZE * 4, NULL, 3,
                    uxUartTaskStack, &xUartTaskTCB);
  
  // 创建数据处理任务 - 中优先级，显示接收状态
  xTaskCreateStatic(dataProcessTask, "DataProcess_Task", configMINIMAL_STACK_SIZE * 3, NULL, 2,
                    uxDataProcessTaskStack, &xDataProcessTaskTCB);
  
  /**
   * @todo Add your own task here
   */
}
