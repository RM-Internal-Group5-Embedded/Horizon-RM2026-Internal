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
#include "spi.h"  // 包含SPI头文件
#include "mg945.hpp"
// ========== 手动PWM控制（已禁用，如需测试可取消注释） ==========
// uint16_t g_manual_pwm = 0;          // PWM值 (0-1000)
// uint8_t g_direction = 0;            // 方向 (0=停止, 1=正转, 2=反转)
// float g_current_speed = 0.0f;       // 当前速度(mm/s)，用于观察

// ========== 左电机系统 ==========
// 左电机实例
// 使用 TIM15_CH1 (PB14/jga_left_speed) 作为PWM输出
// GPIO: PB12(正转), PB10(反转)
jgamotor::Jga25370 motor_left(&htim15, TIM_CHANNEL_1,
                               jga_left_forward_GPIO_Port, jga_left_forward_Pin,
                               jga_left_backward_GPIO_Port, jga_left_backward_Pin);

// 左编码器实例
// 使用 TIM4 作为编码器输入（4倍频模式）
// 参数：定时器句柄, 轮子直径(mm), 编码器线数, 减速比
// 当前配置：PA11(A相)→TIM4_CH1, PA12(B相)→TIM4_CH2（4倍频）
jgaencoder::Jga25370Encoder encoder_left(&htim4, 84.0f, 11, 34.0f);

// ========== 右电机系统 ==========
// 右电机实例
// 使用 TIM15_CH2 (PB15/jga_right_speed) 作为PWM输出
// GPIO: PB13(正转), PB11(反转)
jgamotor::Jga25370 motor_right(&htim15, TIM_CHANNEL_2,
                                jga_right_forward_GPIO_Port, jga_right_forward_Pin,
                                jga_right_backward_GPIO_Port, jga_right_backward_Pin);

// 右编码器实例
// 使用 TIM3 作为编码器输入（4倍频模式）
// 当前配置：PB4(A相)→TIM3_CH1, PB5(B相)→TIM3_CH2（4倍频）
jgaencoder::Jga25370Encoder encoder_right(&htim3, 84.0f, 11, 34.0f);

// ========== 共享传感器 ==========
// 全局MPU6500实例（陀螺仪+加速度计）
mpu6500::MPU6500 mpu;
static volatile bool g_mpuReady = false;

// ========== AR控制器 ==========
// 左电机AR控制器实例
arpid::AR ar_left(&motor_left, &encoder_left, &mpu);

// 右电机AR控制器实例
arpid::AR ar_right(&motor_right, &encoder_right, &mpu);
// ========== mg控制器 ==========
mg::mg945 servo(&htim2, TIM_CHANNEL_1, 0, 45, 90);


#include "jc24btran.hpp"  // JC24B数据收发模块
// JC24B 数据收发实例
jc24b::DataTransceiver g_transceiver;

// 任务栈和控制块
// AR控制任务栈
StackType_t uxARTaskStack[configMINIMAL_STACK_SIZE * 3];  // 384字节栈
StaticTask_t xARTaskTCB;

// 编码器读取任务栈
StackType_t uxEncoderTaskStack[configMINIMAL_STACK_SIZE * 3];  // 384字节栈
StaticTask_t xEncoderTaskTCB;

// MPU6500任务栈
StackType_t uxMpuTaskStack[configMINIMAL_STACK_SIZE * 4];  // 512字节栈
StaticTask_t xMpuTaskTCB;

StackType_t  uxMgTaskStack[configMINIMAL_STACK_SIZE * 3]; // 384字节栈
StaticTask_t xMgTaskTCB;

StackType_t  uxMoveTaskStack[configMINIMAL_STACK_SIZE * 4]; // 512字节栈
StaticTask_t xMoveTaskTCB;

// Transceiver任务栈 - 用于JC24B数据收发
StackType_t uxTransceiverTaskStack[configMINIMAL_STACK_SIZE * 8];  // 512字节栈
StaticTask_t xTransceiverTaskTCB;

// PWM控制任务栈
// StackType_t uxPWMTaskStack[configMINIMAL_STACK_SIZE * 2];  // 256字节栈
// StaticTask_t xPWMTaskTCB;

// AR控制任务函数（双电机）
void arTask(void *pvPara) {
  // 初始化电机
  motor_left.init();
  motor_right.init();
  
  // 初始化AR控制器
  ar_left.init();
  ar_right.init();
  
  // 等待陀螺仪校准完成
  while (!g_mpuReady) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  // 自动启动AR控制
  // ar_left.setAnglePID(700 * 0.6, 0, 7 * 0.6);
  // ar_right.setAnglePID(700 * 0.6, 0, 7 * 0.6);
  ar_left.enable();
  ar_right.enable();
  static volatile float p;
  static volatile float i;
  static volatile float d;
  static volatile float velocity1;
  static volatile float velocity2;
  ar_left.setTargetAngleOffset(-2.0f);
  ar_right.setTargetAngleOffset(-2.0f);
  // 配置偏航PID（在电机PID内部执行差速），以及符号（左+1，右-1）
  // ar_left.setYawPID(0.0f, 0.0f, 0.0f);
  // ar_right.setYawPID(0.0f, 0.0f, 0.0f);
  ar_left.setYawSign(+1);
  ar_right.setYawSign(-1);
  ar_left.setYawTarget(0.0f);
  ar_right.setYawTarget(0.0f);
  ar_left.setVelocityPID(0.05, 0.1, 0.00001, 1);
  ar_right.setVelocityPID(0.05, 0.1, 0.00001, -1);
  // 控制周期
  const uint32_t UPDATE_PERIOD_MS = 1;  // 1ms = 1kHz（尽可能快）
  const float dt = UPDATE_PERIOD_MS / 1000.0f;
  while (true) {
    // AR控制更新（双电机）
    ar_left.update(dt);
    ar_right.update(dt);
    // ar_left.setVelocityPID(p, i, d, -1);
    // ar_right.setVelocityPID(p, i, d, 1);
    ar_left.setTargetVelocity(velocity1);   // 基础速度由 yaw 修正叠加
    ar_right.setTargetVelocity(velocity2);

   
    // 延时
    vTaskDelay(pdMS_TO_TICKS(UPDATE_PERIOD_MS));
  }
}
// 编码器读取任务函数（双编码器，使用卡尔曼滤波融合陀螺仪数据）
void encoderTask(void *pvPara) {
  // 初始化编码器
  encoder_left.init();
  encoder_right.init();
  
  // 更新周期（毫秒）
  const uint32_t UPDATE_PERIOD_MS = 5;  
  
  while (true) {
    // 获取加速度计X轴数据（m/s²）
    // float accel_x = mpu.getAccelX();
    
    // 使用卡尔曼滤波更新编码器数据（融合加速度计X轴数据）
    encoder_left.update(UPDATE_PERIOD_MS);
    encoder_right.update(UPDATE_PERIOD_MS);
    
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
  
  //校准陀螺仪零点（AR必须静止！）
  mpu.calibrateGyro(1000);
  //MPU6500通用Mahony参数：Kp=0.8（中等响应），Ki=0.01（轻微积分补偿），积分限幅=0.2
  mpu.setMahonyGains(0.8f, 0.01f, 0.2f);
  mpu.resetAttitude();
  g_mpuReady = true;
  
  // 传感器数据
  static mpu6500::SensorData data;
  
  // 更新周期
  const uint32_t UPDATE_PERIOD_MS = 1;  
  const float dt = UPDATE_PERIOD_MS / 1000.0f;
  
  while (true) {
    // 更新传感器数据和倾角
    mpu.update(data, dt);
    
    vTaskDelay(pdMS_TO_TICKS(UPDATE_PERIOD_MS));
  }
}
void mgTask(void *pvPara){
  servo.init();

  const uint32_t ROTATE_TIME_MS = 500;  

  while(1){
    servo.first_angle();
    vTaskDelay(pdMS_TO_TICKS(ROTATE_TIME_MS));
    servo.second_angle();
    vTaskDelay(pdMS_TO_TICKS(ROTATE_TIME_MS));
    servo.third_angle();
    vTaskDelay(pdMS_TO_TICKS(ROTATE_TIME_MS));
  }
}
void MoveTask(void *pvPara)
{
  bool position[4] = {0};
  bool receivestatus;
  int8_t i = -1;
  int8_t angle_flag = 0;
  int sensor = 0;//侧面传感器值
  int startposition = 0; //起始点信号
  servo.first_angle();
  while(1)
  {
    receivestatus =  g_transceiver.getPositions(position);
    if (startposition)
    {
      ar_left.setTargetVelocity(50);
      ar_right.setTargetVelocity(50);
    }
    // TODO:: 在这里加巡线逻辑，然后把侧边sensor优先级加到最大，其次才是巡线
      if (sensor && !startposition)
      {
        ar_left.setTargetVelocity(0);
        ar_right.setTargetVelocity(0);
        vTaskDelay(pdMS_TO_TICKS(500));
        if (sensor && !startposition)
        {
          i++;
          if (position[i])
          {
            if(angle_flag == 0)
            {
              servo.second_angle();
              vTaskDelay(pdMS_TO_TICKS(2000));
              angle_flag = 1;
            }
            if(angle_flag == 1)
            {
              servo.third_angle();
              vTaskDelay(pdMS_TO_TICKS(500));
              servo.first_angle();
              vTaskDelay(pdMS_TO_TICKS(2000));
              angle_flag = 2;
            }
            if (angle_flag == 2)
            {
              ar_left.setTargetVelocity(-50);
              ar_right.setTargetVelocity(-50);
              angle_flag = 0;
            }

          }
        }
        ar_left.setTargetVelocity(50);
        ar_right.setTargetVelocity(50);
      }
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

// Transceiver任务函数 - 负责JC24B数据收发和连接管理
void transceiverTask(void *pvPara) {
  // 初始化 DataTransceiver（使用 USART2）
  g_transceiver.init(&huart2);
  
  bool positions[4] = {0};
  uint32_t loop_count = 0;
  
  while (true) {
    loop_count++;
    
    // 主循环处理（检查看门狗超时）
    g_transceiver.process();
    
    // 获取位置数据（如果需要可以在这里处理）
    // if (g_transceiver.getPositions(positions)) {
    //   // 处理位置数据
    // }
    
    vTaskDelay(pdMS_TO_TICKS(50));  // 50ms延时
  }
}

/**
 * @brief Intialize all the drivers and add task to the scheduler
 * @todo  Add your own task in this file
 */
void startUserTasks() {
  // 创建MPU6500任务 - 优先级12（最高，姿态数据最关键）
  xTaskCreateStatic(mpuTask, "MPU_Task", configMINIMAL_STACK_SIZE * 4, NULL, 12,
                    uxMpuTaskStack, &xMpuTaskTCB);
  
  // 创建AR控制任务 - 优先级10（次高，平衡控制）
  xTaskCreateStatic(arTask, "AR_Task", configMINIMAL_STACK_SIZE * 3, NULL, 12,
                    uxARTaskStack, &xARTaskTCB);
  
  // 创建编码器读取任务 - 优先级8（较高，速度反馈）
  xTaskCreateStatic(encoderTask, "Encoder_Task", configMINIMAL_STACK_SIZE * 3, NULL, 8,
                    uxEncoderTaskStack, &xEncoderTaskTCB);
  
  xTaskCreateStatic(mgTask, "MG_Task", configMINIMAL_STACK_SIZE * 3, NULL, 11,
                    uxMgTaskStack, &xMgTaskTCB);
  // xTaskCreateStatic(mgTask, "MoveTask", configMINIMAL_STACK_SIZE * 3, NULL, 11,
  //                   uxMoveTaskStack, &xMoveTaskTCB);
  // 如果需要手动PWM控制（用于测试），取消注释下面的任务
  // xTaskCreateStatic(pwmTask, "PWM_Task", configMINIMAL_STACK_SIZE * 2, NULL, 3,
  //                   uxPWMTaskStack, &xPWMTaskTCB);
  
  // 创建Transceiver任务 - 中高优先级，负责JC24B数据收发和连接管理
  xTaskCreateStatic(transceiverTask, "Transceiver_Task", configMINIMAL_STACK_SIZE * 4, NULL, 8,
                    uxTransceiverTaskStack, &xTransceiverTaskTCB);
}
