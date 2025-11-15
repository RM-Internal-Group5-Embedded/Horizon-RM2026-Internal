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
#include "ws2812.hpp"


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

extern "C" void FDCAN1_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) return;
  
  FDCAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];
  
  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
    uint16_t id = (uint16_t)rxHeader.Identifier;
    
    // Dispatch based on motor ID
    switch (id) {
      case 0x201:  // M3508 motor 1
        if (motor_l_f_p) motor_l_f_p->readMotorFeedback(rxData);
        break;
      case 0x202:  // M3508 motor 2
        if (motor_r_f_p) motor_r_f_p->readMotorFeedback(rxData);
        break;
      case 0x203:  // M3508 motor 3
        if (motor_l_b_p) motor_l_b_p->readMotorFeedback(rxData);
        break;
      case 0x204:  // M3508 motor 4
        if (motor_r_b_p) motor_r_b_p->readMotorFeedback(rxData);
        break;
      case 0x205:  // M3508 motor 4
        if (s_m3508_claw_p) s_m3508_claw_p->readMotorFeedback(rxData);
        break;
      
      // DMJ4310 feedback on Master ID 768 (0x300)
      case 0x300:  // Master ID = 768 (configured in motor)
        if (dmj4310_motor_p) dmj4310_motor_p->readMotorFeedback(rxData);
        break;
      // GM6020 feedback IDs: 0x205-0x20B for motor IDs 1-7
      
      case 0x20B:  // GM6020 motor ID 7 (small claw)
        if (gm6020_motor_p) gm6020_motor_p->readMotorFeedback(rxData);
        break;
      default:
        break; // Unhandled CAN ID
    }
  }
}

extern "C" void FDCAN1_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) {
  // Check for bus-off condition (critical error) and attempt recovery
  if (ErrorStatusITs & FDCAN_FLAG_BUS_OFF) {
    if (HAL_FDCAN_Stop(hfdcan) == HAL_OK) {
      HAL_Delay(10);
      HAL_FDCAN_Start(hfdcan);
    }
  }
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
    // Check for motor timeouts (no feedback for >100ms = disconnected)
    uint32_t current_time = HAL_GetTick();
    bool motor_timeout = false;
    
    if (motor_l_f_p && (current_time - motor_l_f_p->motor_feedback.last_update > 100)) {
      motor_timeout = true;
    }
    if (motor_r_f_p && (current_time - motor_r_f_p->motor_feedback.last_update > 100)) {
      motor_timeout = true;
    }
    if (motor_l_b_p && (current_time - motor_l_b_p->motor_feedback.last_update > 100)) {
      motor_timeout = true;
    }
    if (motor_r_b_p && (current_time - motor_r_b_p->motor_feedback.last_update > 100)) {
      motor_timeout = true;
    }
    
    // If any motor timed out, send zero commands for safety
    if (motor_timeout) {
      er_status_control_p->sendMotorCurrents(0, 0, 0, 0);
    } else {
      // Normal operation: Update wheel state and motors
      er_status_control_p->switchState(g_uart.getReceivedValue());
      er_status_control_p->update(g_uart.getReceivedValue());
    }
    
    vTaskDelay(pdMS_TO_TICKS(5)); 
  }
}

void updateClawTask(void *pvPara) {
  (void)pvPara;
  
  // Initialize claw motors once after scheduler starts
  static bool motors_initialized = false;
  
  if (!motors_initialized) {
    vTaskDelay(pdMS_TO_TICKS(200)); // Wait for CAN to stabilize
    
    // Enable DMJ4310 motor
    dmj4310_motor_p->enableMotor();
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for motor to enable (LED should turn green)
    
    // Set current DMJ4310 position as zero reference
    // Zero position command: 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFE
    uint8_t zero_cmd[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    extern FDCAN_HandleTypeDef hfdcan1;
    FDCAN_TxHeaderTypeDef zero_header;
    zero_header.Identifier = 1;  // DMJ4310 CAN ID
    zero_header.IdType = FDCAN_STANDARD_ID;
    zero_header.TxFrameType = FDCAN_DATA_FRAME;
    zero_header.DataLength = FDCAN_DLC_BYTES_8;
    zero_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    zero_header.BitRateSwitch = FDCAN_BRS_OFF;
    zero_header.FDFormat = FDCAN_CLASSIC_CAN;
    zero_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    zero_header.MessageMarker = 0;
    
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &zero_header, zero_cmd);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Reset internal tracking in motor object so position starts at zero
    dmj4310_motor_p->resetData();
    
    // Send initial zero commands to GM6020 and M3508 claw motors
    gm6020_motor_p->sendCurrent(0);
    s_m3508_claw_p->sendCurrent(0);
    
    motors_initialized = true;
  }
  
  while (true) {
    uint32_t current_time = HAL_GetTick();
    
    // Check for claw motor timeouts (no feedback for >100ms = disconnected)
    volatile static bool claw_timeout = false;
    if (gm6020_motor_p && (current_time - gm6020_motor_p->motor_feedback.last_update > 100)) 
      claw_timeout = true;
    
    if (s_m3508_claw_p && (current_time - s_m3508_claw_p->motor_feedback.last_update > 100)) 
      claw_timeout = true;
    
    if (dmj4310_motor_p && (current_time - dmj4310_motor_p->motor_feedback.last_update > 100)) 
      claw_timeout = true;
    // If any claw motor timed out, send zero commands for safety
    if (claw_timeout) {
      if (gm6020_motor_p) gm6020_motor_p->sendCurrent(0);
      if (s_m3508_claw_p) s_m3508_claw_p->sendCurrent(0);
      // DMJ4310 already gets zero command above
    } else {
      // Normal operation: Update claw state and motors
      er_claw_control_p->switchState(g_uart.getReceivedValue());
      er_claw_control_p->update(g_uart.getReceivedValue());
    }
    
    vTaskDelay(pdMS_TO_TICKS(5));  // 100Hz for smooth DMJ4310 feedback
  }
}

/**
 * @brief Intialize all the drivers and add task to the scheduler
 * @todo  Add your own task in this file
 */
void startUserTasks() {
  // Initialize WS2812 LED strip (visual indicator that system started)
  WS2812::init(8);  // Initialize with 8 LEDs
  WS2812::blink(0, 0, 255, 0);    // LED 0: Green (system started)
  WS2812::blink(1, 0, 0, 255);    // LED 1: Blue (ready)
  WS2812::blink(2, 255, 0, 0);    // LED 2: Red (standby)
  
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
  // DMJ4310: CAN ID = 1, Master ID = 768 (0x300), MIT mode
  static DMJ4310Functions s_dmj4310_base(1, FDCAN1_RxFifo0Callback, FDCAN1_ErrorStatusCallback, 0x205, 0x20B);
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
  xTaskCreateStatic(updateERTask, "Chassis_Task", configMINIMAL_STACK_SIZE * 6, NULL, 4,
                    uxERTaskStack, &xERTaskTCB);
  
  // Create claw update task (lower priority, runs at 50Hz)
  xTaskCreateStatic(updateClawTask, "Claw_Task", configMINIMAL_STACK_SIZE * 8, NULL, 5,
                    uxClawTaskStack, &xClawTaskTCB);

  /**
   * @todo Add your own task here
   */
}
