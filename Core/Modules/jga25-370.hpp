#ifndef JGA25_370_HPP
#define JGA25_370_HPP

#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include <cstring>
#include <cstdio>
namespace jgamotor
{
    class Jga25370
    {
    public:
        // 构造函数：传入PWM定时器和通道
        Jga25370(TIM_HandleTypeDef* _pwm_timer, uint32_t _pwm_channel);
        
        // 初始化电机
        void init();
        
        // 正转（speed: 0-1000）
        void forward(uint16_t _speed);
        
        // 反转（speed: 0-1000）
        void backward(uint16_t _speed);
        
        // 停止
        void stop();
        
    private:
        TIM_HandleTypeDef* pwm_timer_;
        uint32_t pwm_channel_;
        
        void setPwm(uint16_t _duty);  // 设置PWM占空比
    };
}

#endif // JGA25_370_HPP
