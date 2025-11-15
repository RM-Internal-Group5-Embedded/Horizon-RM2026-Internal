#ifndef MG945_HPP
#define MG945_HPP

#include "main.h"
#include "tim.h"
#include "gpio.h"
#include <cstdint>

namespace mg
{
    class mg945
    {
    public:
        mg945(TIM_HandleTypeDef* pwm_timer,
              uint32_t pwm_channel,
              uint32_t first_angle,
              uint32_t second_angle,
              uint32_t third_angle);

        void init();
        void first_angle();
        void second_angle();
        void third_angle();

    private:
        void set_angle(uint32_t angle);
        uint32_t angle_to_ccr(uint32_t angle);

        TIM_HandleTypeDef* pwm_timer_;
        uint32_t pwm_channel_;
        uint32_t first_angle_;
        uint32_t second_angle_;
        uint32_t third_angle_;
    };
}

#endif // MG945_HPP
