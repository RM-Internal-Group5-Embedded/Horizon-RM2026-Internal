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
#include "EROperatingSystem.hpp"
#include "MotorFunctions.hpp"

extern "C" {
#include "fdcan.h"
}


// 全局UART实例
uartdriver::Uart g_uart;

// 任务栈和控制块
// UART任务栈 - 需要处理复杂的数据结构和DMA操作，使用更大的栈
StackType_t uxUartTaskStack[configMINIMAL_STACK_SIZE * 8];  // 1024字节栈
StaticTask_t xUartTaskTCB;
// ER任务栈和控制块
StackType_t uxERTaskStack[configMINIMAL_STACK_SIZE * 6];
StaticTask_t xERTaskTCB;
// UART任务函数 - 负责UART初始化和心跳检查
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

// //forward declaration of FDCAN callbacks
// extern "C" void FDCAN1_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
// extern "C" void FDCAN1_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs);

// Four Mecanum Wheel Motors (static storage)
M3508Functions * motor_l_f_p = nullptr;
M3508Functions * motor_r_f_p = nullptr;
M3508Functions * motor_l_b_p = nullptr;
M3508Functions * motor_r_b_p = nullptr;

// Store last received ID for debugging
static volatile uint16_t s_lastCanId = 0;

// Simple FDCAN callbacks
extern "C" void FDCAN1_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) return;
  
  FDCAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];
  
  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5); // Debug: CAN RX
    
    uint16_t id = (uint16_t)rxHeader.Identifier;
    s_lastCanId = id;  // Store for debugging
    
    bool matched = false;
    
    switch(id){
      case 0x201:
        motor_l_f_p->readMotorFeedback(rxData);
        matched = true;
        break;
      case 0x202:
        motor_r_f_p->readMotorFeedback(rxData);
        matched = true;
        break;
      case 0x203:
        motor_l_b_p->readMotorFeedback(rxData);
        matched = true;
        break;
      case 0x204:
        motor_r_b_p->readMotorFeedback(rxData);
        matched = true;
        break;
      default:
        break;
    }
      // motor_l_f_p->readMotorFeedback(rxData);
      // motor_r_f_p->readMotorFeedback(rxData);
      // motor_l_b_p->readMotorFeedback(rxData);
      // motor_r_b_p->readMotorFeedback(rxData);
      
    
  }
}

extern "C" void FDCAN1_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) {
  (void)hfdcan; (void)ErrorStatusITs;
}

ERStatusControl *er_status_control_p = nullptr;

void updateERTask(void *pvPara) {
  (void)pvPara;
  
  // Send zero current once to trigger motor feedback
  static bool sent_init = false;
  if (!sent_init) {
    motor_l_f_p->sendCurrent(0);
    motor_r_f_p->sendCurrent(0);
    motor_l_b_p->sendCurrent(0);
    motor_r_b_p->sendCurrent(0);
    sent_init = true;
  }
  
  while (true) {
    // 获取最新的UART接收数据
    const uartdriver::ReceivedValue& received_data = g_uart.getReceivedValue();

    // 根据接收到的数据切换状态
    er_status_control_p->switchState(received_data);

    // 执行当前状态对应的操作
    switch (er_status_control_p->current_state) {
      case ERStatusControl::IDLE:
        er_status_control_p->idleMode();
        break;
      case ERStatusControl::MANUAL:
        er_status_control_p->manualMode(received_data);
        break;
      case ERStatusControl::GOLD:
        er_status_control_p->goldMode();
        break;
      case ERStatusControl::MINING:
        er_status_control_p->miningMode(received_data);
        break;
      case ERStatusControl::DEPOSIT:
        // er_status_control_p->depositMode();
        break;
      case ERStatusControl::ERROR:
        er_status_control_p->idleMode();
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
  // Initialize four motors with static storage and assign global pointers
  static M3508Functions s_motor_l_f(1, FDCAN1_RxFifo0Callback, FDCAN1_ErrorStatusCallback, 0x201, 0x204); 
  static M3508Functions s_motor_r_f(2, FDCAN1_RxFifo0Callback, FDCAN1_ErrorStatusCallback, 0x201, 0x204); 
  static M3508Functions s_motor_l_b(3, FDCAN1_RxFifo0Callback, FDCAN1_ErrorStatusCallback, 0x201, 0x204); 
  static M3508Functions s_motor_r_b(4, FDCAN1_RxFifo0Callback, FDCAN1_ErrorStatusCallback, 0x201, 0x204); 
  motor_l_f_p = &s_motor_l_f;
  motor_r_f_p = &s_motor_r_f;
  motor_l_b_p = &s_motor_l_b;
  motor_r_b_p = &s_motor_r_b;

  static ERStatusControl s_er_status_control(
      *motor_l_f_p, *motor_r_f_p,
      *motor_l_b_p, *motor_r_b_p
  );
  er_status_control_p = &s_er_status_control;

  // Configure FDCAN filter, callbacks, and start
  extern FDCAN_HandleTypeDef hfdcan1;
  FDCAN_FilterTypeDef filter;
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0;
  filter.FilterType = FDCAN_FILTER_RANGE;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = 0x000;  // Accept ALL standard IDs
  filter.FilterID2 = 0x7FF;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
  
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, 
                              FDCAN_ACCEPT_IN_RX_FIFO0, 
                              FDCAN_ACCEPT_IN_RX_FIFO0,  // Accept all non-matching too
                              FDCAN_FILTER_REMOTE, 
                              FDCAN_FILTER_REMOTE);
  
  HAL_FDCAN_RegisterRxFifo0Callback(&hfdcan1, FDCAN1_RxFifo0Callback);
  HAL_FDCAN_RegisterErrorStatusCallback(&hfdcan1, FDCAN1_ErrorStatusCallback);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_Start(&hfdcan1);

  // Create UART task
  xTaskCreateStatic(uartTask, "UART_Task", configMINIMAL_STACK_SIZE * 8, NULL, 3,
                    uxUartTaskStack, &xUartTaskTCB);
  
  // Create ER update task
  xTaskCreateStatic(updateERTask, "update_ER_Task", configMINIMAL_STACK_SIZE * 6, NULL, 2,
                    uxERTaskStack, &xERTaskTCB);
  
  /**
   * @todo Add your own task here
   */
}
