#include "jga25-370.hpp"

namespace jgamotor
{
    Jga25370::Jga25370(TIM_HandleTypeDef* _pwm_timer, uint32_t _pwm_channel,
                       GPIO_TypeDef* _forward_port, uint16_t _forward_pin,
                       GPIO_TypeDef* _backward_port, uint16_t _backward_pin)
        : pwm_timer_(_pwm_timer), pwm_channel_(_pwm_channel)
        , forward_port_(_forward_port), forward_pin_(_forward_pin)
        , backward_port_(_backward_port), backward_pin_(_backward_pin)
    {
    }
    
    void Jga25370::init()
    {
        // 启动PWM
        HAL_TIM_PWM_Start(pwm_timer_, pwm_channel_);
        
        // 初始停止状态
        stop();
    }
    
    void Jga25370::forward(uint16_t _speed)
    {
        // 限制速度范围
        if (_speed > 1000) _speed = 1000;
        
        // 设置方向：正转
        HAL_GPIO_WritePin(forward_port_, forward_pin_, GPIO_PIN_SET);
        HAL_GPIO_WritePin(backward_port_, backward_pin_, GPIO_PIN_RESET);
        
        // 设置PWM速度
        setPwm(_speed);
    }
    
    void Jga25370::backward(uint16_t _speed)
    {
        // 限制速度范围
        if (_speed > 1000) _speed = 1000;
        
        // 设置方向：反转
        HAL_GPIO_WritePin(forward_port_, forward_pin_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(backward_port_, backward_pin_, GPIO_PIN_SET);
        
        // 设置PWM速度
        setPwm(_speed);
    }
    
    void Jga25370::stop()
    {
        // 两个方向引脚都设为低
        HAL_GPIO_WritePin(forward_port_, forward_pin_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(backward_port_, backward_pin_, GPIO_PIN_RESET);
        // PWM设为0
        setPwm(0);
    }
    
    void Jga25370::setPwm(uint16_t _duty)
    {
        // PWM范围 0-1000 映射到定时器ARR
        // 注意：PWM周期 = ARR + 1（从0数到ARR）
        uint32_t arr = __HAL_TIM_GET_AUTORELOAD(pwm_timer_);
        uint32_t ccr = ((arr + 1) * _duty) / 1000 - 1;
        
        __HAL_TIM_SET_COMPARE(pwm_timer_, pwm_channel_, ccr);
    }
}
