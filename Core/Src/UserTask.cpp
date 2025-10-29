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
#include "line_follower.hpp"  // Line follower module


// 全局UART实例
uartdriver::uart g_uart;

// 全局Line Follower实例（使用占位符引脚配置）
// TODO: 实际GPIO引脚配置待定，当前使用LED引脚作为占位符
LineFollower::SensorPins line_follower_pins = {
    GPIOB, GPIO_PIN_12,  // Slot sensor (placeholder: LED1)
    GPIOB, GPIO_PIN_13,  // Left sensor (placeholder: LED2)
    GPIOC, GPIO_PIN_13,  // Middle sensor (placeholder: BTN_0)
    GPIOC, GPIO_PIN_14   // Right sensor (placeholder: BTN_1)
};
LineFollower::LineFollowerController g_line_follower(line_follower_pins);

// 任务栈和控制块
// UART任务栈 - 需要处理复杂的数据结构和DMA操作，使用更大的栈
StackType_t uxUartTaskStack[configMINIMAL_STACK_SIZE * 4];  // 512字节栈
StaticTask_t xUartTaskTCB;

// 数据处理任务栈 - 用于处理接收到的数据
StackType_t uxDataProcessTaskStack[configMINIMAL_STACK_SIZE * 3];  // 384字节栈
StaticTask_t xDataProcessTaskTCB;

// Line Follower任务栈 - 处理传感器读取和控制逻辑
StackType_t uxLineFollowerTaskStack[configMINIMAL_STACK_SIZE * 4];  // 512字节栈
StaticTask_t xLineFollowerTaskTCB;
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

// Line Follower任务函数 - 负责线循迹控制
void lineFollowerTask(void *pvPara) {
  // 初始化Line Follower
  g_line_follower.init();
  
  while (true) {
    // 处理线循迹逻辑：读取传感器 -> 计算状态 -> 更新电机命令
    g_line_follower.processLineFollowing();
    
    // 获取电机命令（当前为占位符，实际电机控制待实现）
    LineFollower::MotorCommand cmd = g_line_follower.getMotorCommand();
    (void)cmd;  // Suppress unused variable warning until motor interface is implemented
    
    // TODO: 调用实际的电机控制接口
    // motor_controller.setSpeed(cmd.left_speed, cmd.right_speed);
    
    // 50Hz更新频率（20ms延时）
    vTaskDelay(pdMS_TO_TICKS(LineFollower::LINE_FOLLOWER_UPDATE_RATE_MS));
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
  
  // 创建Line Follower任务 - 中优先级，负责线循迹控制
  xTaskCreateStatic(lineFollowerTask, "LineFollower_Task", configMINIMAL_STACK_SIZE * 4, NULL, 2,
                    uxLineFollowerTaskStack, &xLineFollowerTaskTCB);
  
  /**
   * @todo Add your own task here
   */
}
