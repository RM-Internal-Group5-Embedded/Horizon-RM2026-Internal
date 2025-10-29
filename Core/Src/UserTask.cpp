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

//forward declaration of FDCAN callbacks
extern "C" void FDCAN1_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
extern "C" void FDCAN1_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs);

// Four Mecanum Wheel Motors
M3508Functions motor_l_f(1, FDCAN1_RxFifo0Callback, FDCAN1_ErrorStatusCallback, 0x201, 0x204); 
M3508Functions motor_r_f(2, FDCAN1_RxFifo0Callback, FDCAN1_ErrorStatusCallback, 0x202, 0x205); 
M3508Functions motor_l_b(3, FDCAN1_RxFifo0Callback, FDCAN1_ErrorStatusCallback, 0x203, 0x206); 
M3508Functions motor_r_b(4, FDCAN1_RxFifo0Callback, FDCAN1_ErrorStatusCallback, 0x207, 0x208); 

// Simple flag to ensure only one RX is processed at a time inside ISR context
static volatile uint8_t s_fdcanRxInProgress = 0;

extern "C" void FDCAN1_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  // FreeRTOS-safe: do minimal work, no blocking calls
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) {
    return;
  }

  if (s_fdcanRxInProgress) {
    return; // Drop/skip if already processing to avoid re-entrancy
  }
  s_fdcanRxInProgress = 1;
  FDCAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];
  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
    uint16_t can_id = rxHeader.Identifier;
    // Dispatch to corresponding motor instance by CAN ID
    switch (can_id) {
      case 0x201: motor_l_f.readMotorFeedback(rxData); break;
      case 0x202: motor_r_f.readMotorFeedback(rxData); break;
      case 0x203: motor_l_b.readMotorFeedback(rxData); break;
      case 0x204: motor_r_b.readMotorFeedback(rxData); break;
      default: break;
    }
  }
  s_fdcanRxInProgress = 0;
}

extern "C" void FDCAN1_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) {
  // Optionally record or toggle an LED; keep ISR short
  (void)hfdcan;
  (void)ErrorStatusITs;
}

void updateERTask(void *pvPara) {
  ERStatusControl er_status_control(
      motor_l_f, motor_r_f,
      motor_l_b, motor_r_b
  );

  while (true) {
    // 获取最新的UART接收数据
    const uartdriver::ReceivedValue& received_data = g_uart.getReceivedValue();

    // 根据接收到的数据切换状态
    er_status_control.switchState(received_data);

    // 执行当前状态对应的操作
    switch (er_status_control.current_state) {
      case ERStatusControl::IDLE:
        er_status_control.idleMode();
        break;
      case ERStatusControl::MANUAL:
        er_status_control.manualMode(received_data);
        break;
      case ERStatusControl::GOLD:
        er_status_control.goldMode();
        break;
      case ERStatusControl::MINING:
        er_status_control.miningMode(received_data);
        break;
      case ERStatusControl::DEPOSIT:
        // er_status_control.depositMode();
      case ERStatusControl::ERROR:
        er_status_control.idleMode();
        break;
        // Handle error state if needed
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz loop
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
  
  // 创建ER状态更新任务 - 负责根据UART数据更新ER状态
  xTaskCreateStatic(updateERTask, "update_ER_Task", configMINIMAL_STACK_SIZE * 3, NULL, 2,
                    uxDataProcessTaskStack, &xDataProcessTaskTCB);
  
  /**
   * @todo Add your own task here
   */
}
