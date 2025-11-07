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
// ER任务栈和控制块 (chassis control)
StackType_t uxERTaskStack[configMINIMAL_STACK_SIZE * 6];
StaticTask_t xERTaskTCB;
// Claw任务栈和控制块
StackType_t uxClawTaskStack[configMINIMAL_STACK_SIZE * 8];
StaticTask_t xClawTaskTCB;
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

// Claw Motors (static storage)
GM6020Functions   * gm6020_motor_p  = nullptr;  // Small claw (ID 7)
M3508Functions    * s_m3508_claw_p  = nullptr;  // Medium (ID 5)
DMJ4310Functions  * dmj4310_motor_p = nullptr;  // Base claw (ID 5)

// Store last received ID for debugging
static volatile uint16_t s_lastCanId = 0;
static volatile uint8_t s_m3508_feedback_count[4] = {0, 0, 0, 0}; // Count feedback from each M3508
static volatile uint16_t s_canIdHistory[16] = {0}; // Circular buffer of last 16 CAN IDs
static volatile uint8_t s_canIdIndex = 0;

// Simple FDCAN callbacks
extern "C" void FDCAN1_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) return;
  
  FDCAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];
  
  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
    uint16_t id = (uint16_t)rxHeader.Identifier;
    s_lastCanId = id;
    
    // Store in circular buffer
    s_canIdHistory[s_canIdIndex] = id;
    s_canIdIndex = (s_canIdIndex + 1) % 16;
    
    // Dispatch based on motor ID
    switch (id) {
      case 0x201:  // M3508 motor 1
        s_m3508_feedback_count[0]++;
        if (motor_l_f_p) motor_l_f_p->readMotorFeedback(rxData);
        break;
      case 0x202:  // M3508 motor 2
        s_m3508_feedback_count[1]++;
        if (motor_r_f_p) motor_r_f_p->readMotorFeedback(rxData);
        break;
      case 0x203:  // M3508 motor 3
        s_m3508_feedback_count[2]++;
        if (motor_l_b_p) motor_l_b_p->readMotorFeedback(rxData);
        break;
      case 0x204:  // M3508 motor 4
        s_m3508_feedback_count[3]++;
        if (motor_r_b_p) motor_r_b_p->readMotorFeedback(rxData);
        break;
      case 0x205:  // M3508 motor 4
        if (s_m3508_claw_p) s_m3508_claw_p->readMotorFeedback(rxData);
        break;
      
      // DMJ4310 feedback on Master ID (default 0)
      case 0x000:  // Master ID = 0 (FACTORY DEFAULT)
      case 0x001:  // Master ID = 1
      case 0x002:  // Master ID = 2
      case 0x003:  // Master ID = 3
      case 0x004:  // Master ID = 4
      case 0x005:  // Master ID = 5
      case 0x006:  // Master ID = 6
      case 0x007:  // Master ID = 7
        if (dmj4310_motor_p) {
          dmj4310_motor_p->readMotorFeedback(rxData);
        }
        break;
      
      // GM6020 feedback IDs: 0x205-0x20B for motor IDs 1-7
      
      case 0x20B:  // GM6020 motor ID 7 (small claw)
        {
          static volatile uint32_t s_gm6020_feedback_count = 0;
          s_gm6020_feedback_count++;  // Track feedback messages
          if (gm6020_motor_p) gm6020_motor_p->readMotorFeedback(rxData);
        }
        break;
      default:
        break; // Unhandled CAN ID
    }
  }
}

extern "C" void FDCAN1_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) {
  (void)hfdcan; (void)ErrorStatusITs;
}

ERStatusControl *er_status_control_p = nullptr;
ERClawControl *er_claw_control_p = nullptr;

//const uartdriver::ReceivedValue& received_data;

void updateERTask(void *pvPara) {
  (void)pvPara;
  
  // Initialize chassis motors once after scheduler starts
  static bool motors_initialized = false;
  if (!motors_initialized) {
    
    // Send initial zero commands to all M3508 chassis motors
    motor_l_f_p->sendCurrent(0);
    motor_r_f_p->sendCurrent(0);
    motor_l_b_p->sendCurrent(0);
    motor_r_b_p->sendCurrent(0);
    
    motors_initialized = true;
  }
  
  while (true) {
    // Debug: indicate state with LEDs
    // LED1 ON = data valid, OFF = invalid
    if (g_uart.isDataValid()) {
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    }

    // Update wheel state and motors (chassis only)
    er_status_control_p->switchState(g_uart.getReceivedValue());
    er_status_control_p->update(g_uart.getReceivedValue());
    
    vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz loop
  }
}

void updateClawTask(void *pvPara) {
  (void)pvPara;
  
  // Initialize claw motors once after scheduler starts
  static bool motors_initialized = false;
  if (!motors_initialized) {
    // Enable DMJ4310 motors
    dmj4310_motor_p->enableMotorBroadcast();
    
    
    // Send initial zero commands to GM6020 and M3508 claw motors
    gm6020_motor_p->sendCurrent(0);
    s_m3508_claw_p->sendCurrent(0);
    
    motors_initialized = true;
  }
  
  while (true) {
    // Update claw state and motors based on UART commands
    er_claw_control_p->switchState(g_uart.getReceivedValue());
    er_claw_control_p->update();
    
    vTaskDelay(pdMS_TO_TICKS(8)); // 50 Hz loop (slower to reduce CAN traffic)
  }
}

/**
 * @brief Intialize all the drivers and add task to the scheduler
 * @todo  Add your own task in this file
 */
void startUserTasks() {
  // Initialize four M3508 motors with static storage and assign global pointers
  static M3508Functions s_motor_l_f(1, FDCAN1_RxFifo0Callback, FDCAN1_ErrorStatusCallback, 0x201, 0x204); 
  static M3508Functions s_motor_r_f(2, FDCAN1_RxFifo0Callback, FDCAN1_ErrorStatusCallback, 0x201, 0x204); 
  static M3508Functions s_motor_l_b(3, FDCAN1_RxFifo0Callback, FDCAN1_ErrorStatusCallback, 0x201, 0x204); 
  static M3508Functions s_motor_r_b(4, FDCAN1_RxFifo0Callback, FDCAN1_ErrorStatusCallback, 0x201, 0x204); 
  
  motor_l_f_p = &s_motor_l_f;
  motor_r_f_p = &s_motor_r_f;
  motor_l_b_p = &s_motor_l_b;
  motor_r_b_p = &s_motor_r_b;

  // Initialize claw motors
  static DMJ4310Functions s_dmj4310_base(5, FDCAN1_RxFifo0Callback, FDCAN1_ErrorStatusCallback, 0x205, 0x20B);
  static GM6020Functions  s_gm6020_small(7, FDCAN1_RxFifo0Callback, FDCAN1_ErrorStatusCallback, 0x205, 0x20B);
  static M3508Functions   s_m3508_claw  (5, FDCAN1_RxFifo0Callback, FDCAN1_ErrorStatusCallback, 0x205, 0x20B);
  
  dmj4310_motor_p = &s_dmj4310_base;
  gm6020_motor_p = &s_gm6020_small;
  s_m3508_claw_p = &s_m3508_claw;

  // Initialize chassis control (4 M3508 motors)
  static ERStatusControl s_er_status_control(
      *motor_l_f_p, *motor_r_f_p,
      *motor_l_b_p, *motor_r_b_p, 
      3.0, 4.0  // dimensions of car: length * width
  );
  er_status_control_p = &s_er_status_control;
  
  // Initialize claw control (DMJ4310 + GM6020 + M3508)
  static ERClawControl s_er_claw_control(s_dmj4310_base, s_gm6020_small, s_m3508_claw);
  er_claw_control_p = &s_er_claw_control;

  // Configure FDCAN filter and initialize DJI motors
  extern FDCAN_HandleTypeDef hfdcan1;
  FDCAN_FilterTypeDef filter = Modules::DJIMotors::getFilter(0x7FF, 0x000);  // Accept all standard IDs
  
  // Configure filter
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
      Error_Handler();
  }
  
  // Configure global filter to ACCEPT ALL (not reject like DJIMotors::init does)
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, 
                              FDCAN_ACCEPT_IN_RX_FIFO0,  // Accept all to RX FIFO0
                              FDCAN_ACCEPT_IN_RX_FIFO0,  // Accept all non-matching too
                              FDCAN_FILTER_REMOTE, 
                              FDCAN_FILTER_REMOTE);
  
  // Activate notification
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  
  // Register callbacks
  HAL_FDCAN_RegisterRxFifo0Callback(&hfdcan1, FDCAN1_RxFifo0Callback);
  HAL_FDCAN_RegisterErrorStatusCallback(&hfdcan1, FDCAN1_ErrorStatusCallback);
  
  HAL_FDCAN_Start(&hfdcan1);
  
  // NOTE: Cannot call vTaskDelay here - scheduler not started yet!
  // Motor enable commands will be sent from the ER task instead

  // Create UART task (highest priority)
  xTaskCreateStatic(uartTask, "UART_Task", configMINIMAL_STACK_SIZE * 8, NULL, 3,
                    uxUartTaskStack, &xUartTaskTCB);
  
  // Create ER chassis update task (medium priority)
  xTaskCreateStatic(updateERTask, "Chassis_Task", configMINIMAL_STACK_SIZE * 6, NULL, 2,
                    uxERTaskStack, &xERTaskTCB);
  
  // Create claw update task (lower priority, runs at 50Hz)
  xTaskCreateStatic(updateClawTask, "Claw_Task", configMINIMAL_STACK_SIZE * 8, NULL, 1,
                    uxClawTaskStack, &xClawTaskTCB);
  
  /**
   * @todo Add your own task here
   */
}
