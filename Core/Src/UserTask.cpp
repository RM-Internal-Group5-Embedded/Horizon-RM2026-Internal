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
#include "jc24btran.hpp"  // JC24B数据收发模块



// JC24B 数据收发实例
jc24b::DataTransceiver g_transceiver;


// Transceiver任务栈 - 用于JC24B数据收发
StackType_t uxTransceiverTaskStack[configMINIMAL_STACK_SIZE * 4];  // 512字节栈
StaticTask_t xTransceiverTaskTCB;


// Transceiver任务函数 - 负责JC24B数据收发和连接管理
void transceiverTask(void *pvPara) {
  // 初始化 DataTransceiver（使用 USART2）
  g_transceiver.init(&huart2);
  
  for(int i = 0; i < 3; i++) {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(200));
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  
  bool positions[4] = {0};
  uint32_t loop_count = 0;
  
  while (true) {
    loop_count++;
    if (loop_count % 20 == 0) {  // 50ms * 20 = 1秒
     HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    }
    
    // 主循环处理（检查看门狗超时）
    g_transceiver.process();
    
    if (g_transceiver.getPositions(positions)) {
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
      
      if (positions[0] == 1) {
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
      } else {
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
      }
      
    } else {
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
      
      // 发送停止电机以确保安全
    }
    
    vTaskDelay(pdMS_TO_TICKS(50));  // 50ms延时
  }
}

/**
 * @brief Intialize all the drivers and add task to the scheduler
 * @todo  Add your own task in this file
 */
void startUserTasks() {
  
  // 创建Transceiver任务 - 中高优先级，负责JC24B数据收发和连接管理
  xTaskCreateStatic(transceiverTask, "Transceiver_Task", configMINIMAL_STACK_SIZE * 4, NULL, 4,
                    uxTransceiverTaskStack, &xTransceiverTaskTCB);
  
  /**
   * @todo Add your own task here
   */
}
