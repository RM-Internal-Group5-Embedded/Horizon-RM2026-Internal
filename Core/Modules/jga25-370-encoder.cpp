#include "jga25-370-encoder.hpp"
#include <cmath>

namespace jgaencoder
{
    Jga25370Encoder::Jga25370Encoder(TIM_HandleTypeDef* _encoder_timer,
                                     float _wheel_diameter_mm,
                                     uint16_t _encoder_ppr,
                                     float _gear_ratio)
        : encoder_timer_(_encoder_timer)
        , wheel_diameter_mm_(_wheel_diameter_mm)
        , encoder_ppr_(_encoder_ppr)
        , gear_ratio_(_gear_ratio)
        , last_count_(0)
        , total_pulses_(0)
        , speed_pps_(0.0f)
        , linear_velocity_mmps_(0.0f)
        , angular_velocity_radps_(0.0f)
    {
        // 计算输出轴每转的脉冲数
        // 固定使用4倍频模式（TI12）：编码器线数 × 减速比 × 4
        pulses_per_revolution_ = encoder_ppr_ * gear_ratio_ * 4.0f;
    }
    
    void Jga25370Encoder::init()
    {
        // 启动编码器模式
        HAL_TIM_Encoder_Start(encoder_timer_, TIM_CHANNEL_ALL);
        
        // 重置计数器到中间值，避免启动时的跳变
        __HAL_TIM_SET_COUNTER(encoder_timer_, 32768);
        last_count_ = 32768;
        
        // 重置累计值
        reset();
    }
    
    void Jga25370Encoder::update(uint32_t _dt_ms)
    {
        if (_dt_ms == 0) return;

        // 读取当前计数值
        int32_t current_count = __HAL_TIM_GET_COUNTER(encoder_timer_);
        
        // 处理溢出并计算增量
        int32_t delta = handleOverflow(current_count);
        
        // 累计总脉冲数
        total_pulses_ += delta;

        // 计算速度（脉冲/秒）
        float dt_s = _dt_ms / 1000.0f;
        speed_pps_ = delta / dt_s;
        
        // 计算线速度（mm/s）
        // 公式：线速度 = (脉冲数 / 每转脉冲数) × π × 直径
        float wheel_circumference = M_PI * wheel_diameter_mm_;  // 轮子周长
        linear_velocity_mmps_ = (speed_pps_ / pulses_per_revolution_) * wheel_circumference;
        float filtered = 0.1 * linear_velocity_mmps_ + (1.0f - 0.1) * last_linear_velocity_mmps_;
        linear_velocity_mmps_ = filtered;
        last_linear_velocity_mmps_ =  linear_velocity_mmps_ ;
        // 计算角速度（rad/s）
        // 公式：角速度 = 线速度 / 半径
        float wheel_radius = wheel_diameter_mm_ / 2.0f;
        angular_velocity_radps_ = linear_velocity_mmps_ / wheel_radius;
        
        // 更新上次计数
        last_count_ = current_count;
    }
    
    int32_t Jga25370Encoder::getCount() const
    {
        return __HAL_TIM_GET_COUNTER(encoder_timer_);
    }
    
    int64_t Jga25370Encoder::getPosition() const
    {
        return total_pulses_;
    }
    
    float Jga25370Encoder::getSpeed() const
    {
        return speed_pps_;
    }
    
    float Jga25370Encoder::getLinearVelocity() const
    {
        return linear_velocity_mmps_;
    }
    
    float Jga25370Encoder::getAngularVelocity() const
    {
        return angular_velocity_radps_;
    }
    
    float Jga25370Encoder::getDistance() const
    {
        // 计算累计行驶距离（mm）
        float wheel_circumference = M_PI * wheel_diameter_mm_;
        return (total_pulses_ / pulses_per_revolution_) * wheel_circumference;
    }
    
    float Jga25370Encoder::getMotorRevolutions() const
    {
        // 返回电机累计转数（不是输出轴转数）
        return total_pulses_ / (encoder_ppr_ * 4.0f);
    }
    
    void Jga25370Encoder::reset()
    {
        total_pulses_ = 0;
        speed_pps_ = 0.0f;
        linear_velocity_mmps_ = 0.0f;
        angular_velocity_radps_ = 0.0f;  
        // 重置计数器到中间值
        __HAL_TIM_SET_COUNTER(encoder_timer_, 32768);
        last_count_ = 32768;
        }
    
    int32_t Jga25370Encoder::handleOverflow(int32_t _current_count)
        {
        // 计算增量
        int32_t delta = _current_count - last_count_;
        
        // 处理16位定时器溢出（Period = 65535）
        // 如果增量超过半个周期，说明发生了溢出
        const int32_t HALF_PERIOD = 32768;
        
        if (delta > HALF_PERIOD)
        {
            // 向下溢出（从0绕回65535）
            delta -= 65536;
    }
        else if (delta < -HALF_PERIOD)
        {
            // 向上溢出（从65535绕回0）
            delta += 65536;
        }
        
        return delta;
    }
}

