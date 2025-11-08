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
#include "jga25-370.hpp"  // 包含电机驱动头文件
#include "jga25-370-encoder.hpp"  // 包含编码器驱动头文件
#include "jga25-370pid.hpp"  // 包含AR PID控制头文件
#include "mpu6500.hpp"  // 包含MPU6500陀螺仪驱动头文件
#include "tim.h"  // 包含定时器头文件
#include "comp.h"  // 包含比较器头文件
#include "spi.h"  // 包含SPI头文件

// ========== 手动PWM控制（已禁用，如需测试可取消注释） ==========
// uint16_t g_manual_pwm = 0;          // PWM值 (0-1000)
// uint8_t g_direction = 0;            // 方向 (0=停止, 1=正转, 2=反转)
// float g_current_speed = 0.0f;       // 当前速度(mm/s)，用于观察

// 全局左电机实例
// 使用 TIM15_CH1 (PB14/jga_left_speed) 作为PWM输出
jgamotor::Jga25370 motor_left(&htim15, TIM_CHANNEL_1);

// 全局编码器实例
// 使用 TIM3 作为编码器输入（4倍频模式）
// 参数：定时器句柄, 轮子直径(mm), 编码器线数, 减速比
// 
// 当前配置：PC6(A相)→TIM3_CH1, PA3→COMP2→TIM3_CH2（4倍频）
jgaencoder::Jga25370Encoder encoder_left(&htim3, 84.0f, 11, 34.0f);

// 全局MPU6500实例（陀螺仪+加速度计）
mpu6500::MPU6500 mpu;

// 全局AR控制器实例
arpid::AR ar(&motor_left, &encoder_left, &mpu);

// 任务栈和控制块
// // AR控制任务栈
StackType_t uxARTaskStack[configMINIMAL_STACK_SIZE * 3];  // 384字节栈
StaticTask_t xARTaskTCB;

// 编码器读取任务栈
StackType_t uxEncoderTaskStack[configMINIMAL_STACK_SIZE * 3];  // 384字节栈
StaticTask_t xEncoderTaskTCB;

// MPU6500任务栈
StackType_t uxMpuTaskStack[configMINIMAL_STACK_SIZE * 4];  // 512字节栈
StaticTask_t xMpuTaskTCB;

// PWM控制任务栈（已禁用，如需测试可取消注释）
StackType_t uxPWMTaskStack[configMINIMAL_STACK_SIZE * 2];  // 256字节栈
StaticTask_t xPWMTaskTCB;

// // AR控制任务函数
void arTask(void *pvPara) {
  // 初始化电机
  motor_left.init();
  
  // 初始化AR控制器
  ar.init();
  
  // 等待3秒（等待陀螺仪校准完成）
  vTaskDelay(pdMS_TO_TICKS(3000));
  
  // 自动启动AR控制
  ar.enable();
  ar.setTargetVelocity(0.0f);  // 原地平衡
  
  // 控制周期
  const uint32_t UPDATE_PERIOD_MS = 5;  // 5ms = 200Hz
  const float dt = UPDATE_PERIOD_MS / 1000.0f;
  
  while (true) {
    // AR控制更新
    ar.update(dt);
    
    // 延时
    vTaskDelay(pdMS_TO_TICKS(UPDATE_PERIOD_MS));
  }
}

// 编码器读取任务函数
void encoderTask(void *pvPara) {
  // 启动COMP2（方案B需要启动比较器）
  HAL_COMP_Start(&hcomp2);
  
  // 初始化编码器
  encoder_left.init();
  
  // 更新周期（毫秒）
  const uint32_t UPDATE_PERIOD_MS = 20;  // 20ms = 50Hz
  
  while (true) {
    // 更新编码器数据
    encoder_left.update(UPDATE_PERIOD_MS);
    
    // 延时
    vTaskDelay(pdMS_TO_TICKS(UPDATE_PERIOD_MS));
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
  
  // 校准陀螺仪零点（AR必须静止！）
  mpu.calibrateGyro(1000);
  
  // 传感器数据
  mpu6500::SensorData data;
  
  // 更新周期
  const uint32_t UPDATE_PERIOD_MS = 5;  // 5ms = 200Hz（平衡车推荐）
  const float dt = UPDATE_PERIOD_MS / 1000.0f;
  
  while (true) {
    // 更新传感器数据和倾角
    mpu.update(data, dt);
    
    vTaskDelay(pdMS_TO_TICKS(UPDATE_PERIOD_MS));
  }
}

// 手动PWM控制任务函数（已禁用，如需测试可取消注释）
// void pwmTask(void *pvPara) {
//   // 初始化电机
//   motor_left.init();
  
//   // 更新周期
//   const uint32_t UPDATE_PERIOD_MS = 20;  // 20ms
  
//   while (true) {
//     // 更新当前速度（用于观察）
//     g_current_speed = encoder_left.getLinearVelocity();
    
//     // 根据方向控制电机
//     if (g_direction == 0) {
//       // 停止
//       motor_left.stop();
//     } else if (g_direction == 1) {
//       // 正转
//       uint16_t pwm = g_manual_pwm;
//       if (pwm > 1000) pwm = 1000;
//       motor_left.forward(pwm);
//     } else if (g_direction == 2) {
//       // 反转
//       uint16_t pwm = g_manual_pwm;
//       if (pwm > 1000) pwm = 1000;
//       motor_left.backward(pwm);
//     } else {
//       // 无效值，停止
//       motor_left.stop();
//     }
    
//     vTaskDelay(pdMS_TO_TICKS(UPDATE_PERIOD_MS));
//   }
// }

/**
 * @brief Intialize all the drivers and add task to the scheduler
 * @todo  Add your own task in this file
 */
void startUserTasks() {
  // 创建编码器读取任务 - 优先级2
  xTaskCreateStatic(encoderTask, "Encoder_Task", configMINIMAL_STACK_SIZE * 3, NULL, 2,
                    uxEncoderTaskStack, &xEncoderTaskTCB);
  
  // // 创建AR控制任务 - 优先级4
  xTaskCreateStatic(arTask, "AR_Task", configMINIMAL_STACK_SIZE * 3, NULL, 4,
                    uxARTaskStack, &xARTaskTCB);
  
  // 创建MPU6500任务 - 优先级5
  xTaskCreateStatic(mpuTask, "MPU_Task", configMINIMAL_STACK_SIZE * 4, NULL, 5,
                    uxMpuTaskStack, &xMpuTaskTCB);
  
  // 如果需要手动PWM控制（用于测试），取消注释下面的任务
  // xTaskCreateStatic(pwmTask, "PWM_Task", configMINIMAL_STACK_SIZE * 2, NULL, 3,
  //                   uxPWMTaskStack, &xPWMTaskTCB);
  
  /**
   * @todo Add your own task here
   */
}
