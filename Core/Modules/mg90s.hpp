#ifndef MG90S_HPP
#define MG90S_HPP

#include "main.h"
#include "tim.h"
#include "gpio.h"
#include <cstdint>

namespace mg
{
    class mg90s
    {
    public:
        mg90s(TIM_HandleTypeDef* pwm_timer,
              uint32_t pwm_channel,
              uint32_t first_angle,
              uint32_t second_angle,
              uint32_t third_angle);

        void init();
        void first_position();
        void second_position();
        void third_position();

    private:
        void set_angle(float angle);
        uint32_t angle_to_ccr(float angle);

        TIM_HandleTypeDef* pwm_timer_;
        uint32_t pwm_channel_;
        uint32_t first_angle_;
        uint32_t second_angle_;
        uint32_t third_angle_;
    };
}

#endif // MG90S_HPP
