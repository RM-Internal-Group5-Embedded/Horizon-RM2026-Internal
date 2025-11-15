#include "mg945.hpp"
namespace mg
{
    mg945::mg945(TIM_HandleTypeDef* _pwm_timer,
                 uint32_t _pwm_channel,
                 uint32_t _first_angle,
                 uint32_t _second_angle,
                 uint32_t _third_angle)
        : pwm_timer_(_pwm_timer)
        , pwm_channel_(_pwm_channel)
        , first_angle_(_first_angle)
        , second_angle_(_second_angle)
        , third_angle_(_third_angle)
    {
    }

    void mg945::init()
    {
        HAL_TIM_PWM_Start(pwm_timer_, pwm_channel_);
        set_angle(0);
    }

    void mg945::set_angle(uint32_t angle)
    {
        if (angle < 0) angle = 0;
        if (angle > 180) angle = 180;

        uint32_t ccr = angle_to_ccr(angle);
        __HAL_TIM_SET_COMPARE(pwm_timer_, pwm_channel_, ccr);
    }

    void mg945::first_angle()
    {
        set_angle(first_angle_);
    }

    void mg945::second_angle()
    {
        set_angle(second_angle_);
    }

    void mg945::third_angle()
    {
        set_angle(third_angle_);
    }

    // 角度 → CCR 值 (ARR=19999, 20ms 周期)
    uint32_t mg945::angle_to_ccr(uint32_t angle)
    {
        // 限制角度
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    // 0° → 0.5ms = 500us
    // 180°→ 2.5ms = 2500us
    float pulse_us = 500.0f + (angle / 180.0f) * 2000.0f;
    return pulse_us;
    }

} // namespace mg
