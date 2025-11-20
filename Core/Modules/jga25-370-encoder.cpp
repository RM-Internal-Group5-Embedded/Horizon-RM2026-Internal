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
        , kalman_linear_velocity_(0.0f)
        , kalman_p_(5.0f)
        , kalman_q_(0.006f)      // 过程噪声（线速度变化的不确定性）
        , kalman_r_(120.0f)       // 测量噪声（编码器测量误差）
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
    
    // void Jga25370Encoder::updateWithKalman(uint32_t _dt_ms, float _accel_x)
    // {
    //     if (_dt_ms == 0) return;

    //     // 先执行常规更新
    //     update(_dt_ms);
        
    //     // 使用卡尔曼滤波融合编码器线速度和加速度计X轴数据
    //     float dt_s = _dt_ms / 1000.0f;
        
    //     // 编码器线速度作为测量值（mm/s）
    //     float measurement = linear_velocity_mmps_;
        
    //     // 使用加速度计X轴数据改进预测模型
    //     // 加速度计X轴数据单位是m/s²，转换为mm/s²
    //     float accel_mmps2 = _accel_x * 1000.0f;  // m/s² -> mm/s²
        
    //     // 使用加速度预测线速度变化：v = v0 + a * dt
    //     float predicted_velocity = kalman_linear_velocity_ + accel_mmps2 * dt_s * 0.5f;
        
    //     // 根据加速度大小自适应调整过程噪声
    //     // 如果加速度大，说明运动剧烈，增加过程噪声
    //     float accel_change = fabsf(_accel_x);
    //     float adaptive_q = kalman_q_ * (1.0f + accel_change * 0.05f);
        
    //     // 执行卡尔曼滤波更新（使用自适应过程噪声）
    //     kalmanUpdateWithPrediction(measurement, dt_s, adaptive_q, kalman_r_, predicted_velocity);
    // }
    
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
    
    // float Jga25370Encoder::getFilteredLinearVelocity() const
    // {
    //     const float alpha = 0.1f;
    //     static float v = 0.0f;
    //     v = v * (1.0f - alpha) + kalman_linear_velocity_ * alpha;
    //     return v;
    // }
    
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
        kalman_linear_velocity_ = 0.0f;
        kalman_p_ = 1.0f;
        // 重置计数器到中间值
        __HAL_TIM_SET_COUNTER(encoder_timer_, 32768);
        last_count_ = 32768;
    }
    
    // void Jga25370Encoder::kalmanUpdate(float measurement, float dt)
    // {
    //     kalmanUpdate(measurement, dt, kalman_q_);
    // }
    
    // void Jga25370Encoder::kalmanUpdate(float measurement, float dt, float q)
    // {
    //     kalmanUpdateWithPrediction(measurement, dt, q, kalman_r_, kalman_linear_velocity_);
    // }
    
    // void Jga25370Encoder::kalmanUpdateWithPrediction(float measurement, float dt, float q, float r, float predicted)
    // {
    //     // 卡尔曼滤波算法
    //     // 状态：线速度 (mm/s)
    //     // 测量：编码器线速度
        
    //     // 预测步骤（Prediction）
    //     // x_k|k-1 = predicted (使用外部预测值，如加速度计数据)
    //     float x_pred = predicted;
        
    //     // P_k|k-1 = P_k-1|k-1 + Q (增加不确定性)
    //     kalman_p_ = kalman_p_ + q;
        
    //     // 更新步骤（Update）
    //     // 计算卡尔曼增益
    //     float k = kalman_p_ / (kalman_p_ + r);
        
    //     // 更新状态估计
    //     kalman_linear_velocity_ = x_pred + k * (measurement - x_pred);
        
    //     // 将滤波后的线速度赋值给linear_velocity_mmps_，这样getLinearVelocity()返回的就是滤波后的值
    //     linear_velocity_mmps_ = kalman_linear_velocity_;
        
    //     // 更新协方差
    //     kalman_p_ = (1.0f - k) * kalman_p_;
        
    //     // 防止协方差过小或过大
    //     if (kalman_p_ < 0.001f) kalman_p_ = 0.001f;
    //     if (kalman_p_ > 10.0f) kalman_p_ = 10.0f;
    // }
    
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

