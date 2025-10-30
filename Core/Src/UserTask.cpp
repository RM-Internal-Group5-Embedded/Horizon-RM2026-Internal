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
uartdriver::Uart g_uart;

// 全局Line Follower实例（使用实际IR传感器引脚）
LineFollower::SensorPins line_follower_pins = {
    IR1_GPIO_Port, IR1_Pin,  // Slot sensor (S) - PA2
    IR2_GPIO_Port, IR2_Pin,  // Left sensor (L) - PB0
    IR3_GPIO_Port, IR3_Pin,  // Middle sensor (M) - PB2
    IR4_GPIO_Port, IR4_Pin   // Right sensor (R) - PC6
};
LineFollower::LineFollowerController g_line_follower(line_follower_pins);

// 任务栈和控制块
// UART任务栈 - 需要处理复杂的数据结构和DMA操作，使用更大的栈
StackType_t uxUartTaskStack[configMINIMAL_STACK_SIZE * 8];  // 1024字节栈
StaticTask_t xUartTaskTCB;

// 数据处理任务栈 - 用于处理接收到的数据
StackType_t uxDataProcessTaskStack[configMINIMAL_STACK_SIZE * 2];  // 256字节 (1024 bytes)
StaticTask_t xDataProcessTaskTCB;

// Line Follower任务栈 - 轻量级传感器读取和控制逻辑
StackType_t uxLineFollowerTaskStack[configMINIMAL_STACK_SIZE * 2];  // 256字节 (1024 bytes)
StaticTask_t xLineFollowerTaskTCB;
// UART任务函数 - 负责UART初始化和看门狗检查
void uartTask(void *pvPara) {
  // 初始化UART
  g_uart.init();
  
  while (true) {
    // 检查心跳（这个方法需要在循环中定期调用）
    g_uart.checkHeartbeat();
    
    // 延时，避免任务占用过多CPU
    vTaskDelay(pdMS_TO_TICKS(50));  // 50ms延时，心跳检查不需要太频繁
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
  // 创建UART处理任务 - 高优先级，负责UART通信和心跳检查
  xTaskCreateStatic(uartTask, "UART_Task", configMINIMAL_STACK_SIZE * 8, NULL, 3,
                    uxUartTaskStack, &xUartTaskTCB);
  */
  
  // 数据处理任务暂时禁用 - 依赖UART
  /*
  xTaskCreateStatic(dataProcessTask, "DataProcess_Task", configMINIMAL_STACK_SIZE * 2, NULL, 2,
                    uxDataProcessTaskStack, &xDataProcessTaskTCB);
  */
  
  // 创建Line Follower任务 - 中优先级，负责线循迹控制
  xTaskCreateStatic(lineFollowerTask, "LineFollower_Task", configMINIMAL_STACK_SIZE * 2, NULL, 2,
                    uxLineFollowerTaskStack, &xLineFollowerTaskTCB);
  
  /**
   * @todo Add your own task here
   */
}
