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
#include "spi.h"
#include "mpu6500.hpp"


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
// MPU任务栈和控制块
StackType_t uxMpuTaskStack[configMINIMAL_STACK_SIZE * 4];
StaticTask_t xMpuTaskTCB;
// UART任务函数 - 负责UART初始化和心跳检查
void uartTask(void *pvPara) {
  // 初始化UART
  g_uart.init();
  
  while (true) {
    // 检查心跳（这个方法需要在循环中定期调用）
    // This must run frequently to detect disconnection and trigger safe mode
    g_uart.checkHeartbeat();
    
    // Short delay - heartbeat timeout is 5 seconds, so checking every 10ms is safe
    // This ensures quick response to disconnection while not wasting CPU
    vTaskDelay(pdMS_TO_TICKS(10));  // 10ms delay for responsive heartbeat checking
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
DMJ4310Functions  * dmj4310_motor_p = nullptr;  // Base claw (ID 1)

// CAN ID history buffer for debugging (last 16 received IDs)
struct CANIdHistory {
  uint16_t ids[16];
  uint8_t write_index;
  uint8_t count;  // Number of valid entries (0-16)
};
volatile CANIdHistory can_id_history = {{0}, 0, 0};

// Debug counter for DMJ4310 feedback (0x300)
volatile uint32_t dmj4310_feedback_count = 0;

// Debug: Track channel_10 value to see why it's stuck in IDLE
volatile uint16_t debug_channel_10 = 0;

// CAN bus recovery flag - set by error callback, checked by tasks
volatile bool can_bus_off_recovery_needed = false;
volatile uint32_t can_bus_off_recovery_time = 0;

// Forward declaration of FDCAN handle (defined in fdcan.c)
extern FDCAN_HandleTypeDef hfdcan1;

// Shared helper functions for task optimization
namespace TaskHelpers {
  // Check and handle CAN bus recovery (non-blocking)
  inline void checkCANRecovery(uint32_t current_time) {
    if (can_bus_off_recovery_needed) {
      if (current_time - can_bus_off_recovery_time >= 10) {
        if (HAL_FDCAN_Start(&hfdcan1) == HAL_OK) {
          can_bus_off_recovery_needed = false;
        }
      }
    }
  }
  
  // Monitor CAN error counters periodically (shared by both tasks)
  inline void monitorCANErrors(uint32_t current_time) {
    static uint32_t last_error_check_time = 0;
    if (current_time - last_error_check_time >= 200) {
      FDCAN_ErrorCountersTypeDef errorCounters;
      if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &errorCounters) == HAL_OK) {
        if (errorCounters.TxErrorCnt > 64 || errorCounters.RxErrorCnt > 64) {
          // High error rate detected - CAN bus may be experiencing issues
          // The error callback will handle bus-off if it reaches 255
        }
      }
      last_error_check_time = current_time;
    }
  }
  
  // Check chassis motor timeouts (optimized with early exit)
  inline bool checkChassisMotorTimeout(uint32_t current_time) {
    const uint32_t timeout_ms = 100;
    if (motor_l_f_p && (current_time - motor_l_f_p->motor_feedback.last_update > timeout_ms)) return true;
    if (motor_r_f_p && (current_time - motor_r_f_p->motor_feedback.last_update > timeout_ms)) return true;
    if (motor_l_b_p && (current_time - motor_l_b_p->motor_feedback.last_update > timeout_ms)) return true;
    if (motor_r_b_p && (current_time - motor_r_b_p->motor_feedback.last_update > timeout_ms)) return true;
    return false;
  }
  
  // Check claw motor timeouts (optimized with early exit)
  inline bool checkClawMotorTimeout(uint32_t current_time) {
    const uint32_t timeout_ms = 100;
    if (gm6020_motor_p && (current_time - gm6020_motor_p->motor_feedback.last_update > timeout_ms)) return true;
    if (s_m3508_claw_p && (current_time - s_m3508_claw_p->motor_feedback.last_update > timeout_ms)) return true;
    if (dmj4310_motor_p && (current_time - dmj4310_motor_p->motor_feedback.last_update > timeout_ms)) return true;
    return false;
  }
}

extern "C" void FDCAN1_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) return;
  
  // Process ALL pending messages in FIFO0 to avoid missing frames and reduce interrupt frequency
  // Using while loop ensures we don't miss any messages and process them in batches
  // This reduces the number of callback invocations and prevents CAN bus clogging
  FDCAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];
  uint8_t messages_processed = 0;
  const uint8_t MAX_MESSAGES_PER_CALLBACK = 3;  // Process up to 7 messages (one per motor) per callback
  
  // Process messages in batches to avoid spending too much time in interrupt context
  while (messages_processed < MAX_MESSAGES_PER_CALLBACK) {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
      // No more messages in FIFO - exit loop
      break;
    }
    
    uint16_t id = (uint16_t)rxHeader.Identifier;
    messages_processed++;
    
    // Store CAN ID in history buffer (circular buffer)
    can_id_history.ids[can_id_history.write_index] = id;
    can_id_history.write_index = (can_id_history.write_index + 1) % 16;
    if (can_id_history.count < 16) {
      can_id_history.count++;
    }
    
    // Dispatch based on motor ID - process in order to ensure fairness
    switch (id) {
      case 0x201:  // M3508 motor 1 (left-front)
        if (motor_l_f_p) motor_l_f_p->readMotorFeedback(rxData);
        break;
      case 0x202:  // M3508 motor 2 (right-front)
        if (motor_r_f_p) motor_r_f_p->readMotorFeedback(rxData);
        break;
      case 0x203:  // M3508 motor 3 (left-back)
        if (motor_l_b_p) motor_l_b_p->readMotorFeedback(rxData);
        break;
      case 0x204:  // M3508 motor 4 (right-back)
        if (motor_r_b_p) motor_r_b_p->readMotorFeedback(rxData);
        break;
      case 0x205:  // M3508 claw motor (ID 5)
        if (s_m3508_claw_p) s_m3508_claw_p->readMotorFeedback(rxData);
        break;
      
      // DMJ4310 feedback on Master ID 768 (0x300)
      case 0x300:  // Master ID = 768 (configured in motor)
        if (dmj4310_motor_p) {
          dmj4310_feedback_count++;  // Debug: track how many times we receive 0x300
          dmj4310_motor_p->readMotorFeedback(rxData);
        }
        break;
      // GM6020 feedback IDs: 0x205-0x20B for motor IDs 1-7
      
      case 0x20B:  // GM6020 motor ID 7 (small claw)
        if (gm6020_motor_p) gm6020_motor_p->readMotorFeedback(rxData);
        break;
      default:
        break; // Unhandled CAN ID
    }
  }
  
  // If we processed the maximum, there might be more messages - callback will be triggered again
  // This ensures we don't spend too much time in interrupt context while still processing all messages
}

extern "C" void FDCAN1_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) {
  // Check for bus-off condition (critical error) and attempt recovery
  // OPTIMIZATION: Use non-blocking recovery - set flag for task to handle
  // HAL_Delay blocks entire system including other tasks - removed!
  if (ErrorStatusITs & FDCAN_FLAG_BUS_OFF) {
    // Stop CAN and set recovery flag
    // Recovery will be handled by task context (non-blocking)
    HAL_FDCAN_Stop(hfdcan);
    can_bus_off_recovery_needed = true;
    can_bus_off_recovery_time = HAL_GetTick();
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
    // OPTIMIZATION: Cache HAL_GetTick() result to avoid multiple calls
    uint32_t current_time = HAL_GetTick();
    
    // Shared CAN recovery and monitoring (optimized helper functions)
    TaskHelpers::checkCANRecovery(current_time);
    TaskHelpers::monitorCANErrors(current_time);
    
    // Check for motor timeouts (optimized helper function with early exit)
    if (TaskHelpers::checkChassisMotorTimeout(current_time)) {
      // Safety: Send zero commands if any motor timed out
      er_status_control_p->sendMotorCurrents(0, 0, 0, 0);
    } else {
      // Normal operation: Get UART data once (atomic read, disables interrupts briefly)
      // Then use it for both state switching and update to avoid multiple interrupt disables
      uartdriver::ReceivedValue received_data = g_uart.getReceivedValue();
      er_status_control_p->switchState(received_data);
      er_status_control_p->update(received_data);
    }
    
    vTaskDelay(pdMS_TO_TICKS(23));  
  }
}

void updateClawTask(void *pvPara) {
  (void)pvPara;
  
  // Initialize claw motors once after scheduler starts
  static bool motors_initialized = false;
  
  if (!motors_initialized) {
    vTaskDelay(pdMS_TO_TICKS(200)); // Wait for CAN to stabilize
    
    // Reset internal tracking in motor object so position starts at zero
    dmj4310_motor_p->resetData();
    // Enable DMJ4310 motor first (CAN ID = 1)
    dmj4310_motor_p->enableMotor();
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for motor to enable (LED should turn green)
    
    // Set current DMJ4310 position as zero reference
    dmj4310_motor_p->setZeroPosition();
    vTaskDelay(pdMS_TO_TICKS(50));
    
    
    // Send initial zero commands to GM6020 and M3508 claw motors
    gm6020_motor_p->sendCurrent(0);
    s_m3508_claw_p->sendCurrent(0);
    
    motors_initialized = true;
  }

  while (true) {
    // OPTIMIZATION: Cache HAL_GetTick() result to avoid multiple calls
    uint32_t current_time = HAL_GetTick();
    
    // Shared CAN recovery and monitoring (optimized helper functions)
    TaskHelpers::checkCANRecovery(current_time);
    TaskHelpers::monitorCANErrors(current_time);
    
    // ALWAYS process UART data and update state machine, regardless of timeout
    // Timeout only affects safety commands, not state machine logic
    // OPTIMIZATION: Read UART data once and reuse (getReceivedValue disables interrupts)
    uartdriver::ReceivedValue received_data = g_uart.getReceivedValue();
    er_claw_control_p->switchState(received_data);
    
    //Check for claw motor timeouts (optimized helper function with early exit)
    if (TaskHelpers::checkClawMotorTimeout(current_time)) {
      // Safety: Send zero commands if any motor timed out (override normal commands)
      // BUT: Still send keep-alive to DMJ4310 with proper Kp/Kd to keep it enabled
      if (gm6020_motor_p) gm6020_motor_p->sendCurrent(0);
      if (s_m3508_claw_p) s_m3508_claw_p->sendCurrent(0);
      // DMJ4310: Send keep-alive with proper gains (not all zeros) to maintain enabled state
      // Also send enable command if feedback is lost
      if (dmj4310_motor_p) {
        // If no feedback for >50ms, send enable command first
        if (current_time - dmj4310_motor_p->motor_feedback.last_update > 50) {
          dmj4310_motor_p->enableMotor();
        }
        // Always send keep-alive MIT command
        dmj4310_motor_p->sendMITCommand(0.0f, 0.0f, 
                                        dmj4310_motor_p->RPM_KP, 
                                        dmj4310_motor_p->RPM_KD, 
                                        0.0f);
      }
    } else {
      //Normal operation: Update state machine and send normal commands
      
      er_claw_control_p->update(received_data);
    }
    
    vTaskDelay(pdMS_TO_TICKS(13));  // 100Hz update rate  
  }
}

mpu6500::MPU6500 mpu;
bool g_mpuReady = false;

// Function to get MPU6500 yaw for use in ER tasks
// This function is accessible from EROperatingSystem via forward declaration
// Returns current yaw angle in degrees, or 0.0f if MPU not ready
float getMPUYaw() {
    if (g_mpuReady) {
        return mpu.getYaw();
    }
    return 0.0f;  // Return 0 if MPU not ready
}

// Function to reset MPU6500 yaw to 0
// This function is accessible from EROperatingSystem via forward declaration
void resetMPUYaw() {
    if (g_mpuReady) {
        mpu.resetYaw();
    }
}

  // MPU6500陀螺仪任务函数
void mpuTask(void *pvPara) {
  // 初始化MPU6500
  if (!mpu.init()) {
    // 初始化失败
    while(1) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
  
  // 等待1秒让传感器稳定
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  //校准陀螺仪零点（AR必须静止！）
  mpu.calibrateGyro(1000);
  
  // Set Mahony filter gains optimized for fast robot with sustained movement
  // Kp=1.0: Faster response to accelerometer corrections
  // Ki=0.01: Integral term reduces long-term drift (critical for sustained movement)
  // Limit=1.0: Allows more integral correction for better accuracy
  mpu.setMadgwickBeta(0.1f);
  mpu.resetAttitude();
  g_mpuReady = true;
  
  // 传感器数据
  static mpu6500::SensorData data;
  
  // 更新周期: 1ms (1kHz) for fast robot with sustained movement
  // Higher frequency = better accuracy, less drift, faster response
  const uint32_t UPDATE_PERIOD_MS = 1;  
  const float dt = UPDATE_PERIOD_MS / 1000.0f;  // 0.001s = 1ms
  
  while (true) {
    // 更新传感器数据和倾角
    mpu.update(data, dt);
    
    // Run at 1kHz (1ms) for optimal accuracy and minimal drift
    vTaskDelay(pdMS_TO_TICKS(1));
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
  
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
      Error_Handler();
  }
  FDCAN_FilterTypeDef dmj_filter = Modules::DJIMotors::getFilter(0x300, 0x300);
  dmj_filter.FilterIndex = 1;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &dmj_filter) != HAL_OK) {
      Error_Handler();
  }
  
  // Accept all other frames into FIFO0
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, 
                              FDCAN_ACCEPT_IN_RX_FIFO0,
                              FDCAN_ACCEPT_IN_RX_FIFO0,
                              FDCAN_REJECT, 
                              FDCAN_REJECT);
  
  // Activate notification
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  
  // Register callbacks
  HAL_FDCAN_RegisterRxFifo0Callback(&hfdcan1, FDCAN1_RxFifo0Callback);
  HAL_FDCAN_RegisterErrorStatusCallback(&hfdcan1, FDCAN1_ErrorStatusCallback);
  
  HAL_FDCAN_Start(&hfdcan1);
  
  // NOTE: Cannot call vTaskDelay here - scheduler not started yet!
  // Motor enable commands will be sent from the ER task instead

  // Create UART task (HIGHEST priority - critical for safety)
  // FreeRTOS: Higher number = Higher priority
  xTaskCreateStatic(uartTask, "UART_Task", configMINIMAL_STACK_SIZE * 8, NULL, 8,
                    uxUartTaskStack, &xUartTaskTCB);
  
  // Create ER chassis update task (high priority, runs at 50Hz)
  xTaskCreateStatic(updateERTask, "Chassis_Task", configMINIMAL_STACK_SIZE * 6, NULL, 6,
                    uxERTaskStack, &xERTaskTCB);
  
  // Create claw update task (higher priority, runs at 100Hz)
  xTaskCreateStatic(updateClawTask, "Claw_Task", configMINIMAL_STACK_SIZE * 8, NULL, 9,
                    uxClawTaskStack, &xClawTaskTCB);
  
  //mpu6500 (medium priority, runs at 1kHz for fast robot with minimal drift)
  // Higher priority (3) ensures consistent 1ms updates for accuracy
  // xTaskCreateStatic(mpuTask, "MPU_Task", configMINIMAL_STACK_SIZE * 4, NULL, 3,
  //                  uxMpuTaskStack, &xMpuTaskTCB);
  /**
   * @todo Add your own task here
   */
}
