#ifndef JGA25_370_ENCODER_HPP
#define JGA25_370_ENCODER_HPP

#include "main.h"
#include "tim.h"
#include <cstdint>

namespace jgaencoder
{
    class Jga25370Encoder
    {
    public:
        // 构造函数
        Jga25370Encoder(TIM_HandleTypeDef* _encoder_timer, 
                       float _wheel_diameter_mm,
                       uint16_t _encoder_ppr,
                       float _gear_ratio);
        
        // 初始化编码器
        void init();
        
        // 更新编码器数据（需要周期调用，_dt_ms为距离上次调用的时间间隔，单位ms）
        void update(uint32_t _dt_ms);
        
        // 使用卡尔曼滤波更新编码器数据（融合加速度计数据）
        // _dt_ms: 时间间隔(ms), _accel_x: 加速度计X轴数据(m/s²)
        void updateWithKalman(uint32_t _dt_ms, float _accel_x);
        
        // 获取当前计数值
        int32_t getCount() const;
        
        // 获取累计位置（总脉冲数）
        int64_t getPosition() const;
        
        // 获取速度（脉冲/秒）
        float getSpeed() const;
        
        // 获取线速度（mm/s）
        float getLinearVelocity() const;
        
        // 获取角速度（rad/s）- 原始值
        float getAngularVelocity() const;
        
        // 获取滤波后的线速度（mm/s）- 卡尔曼滤波结果
        float getFilteredLinearVelocity() const;
        
        // 获取累计行驶距离（mm）
        float getDistance() const;
        
        // 获取电机累计转数
        float getMotorRevolutions() const;
        
        // 重置所有累计值
        void reset();

    private:
        TIM_HandleTypeDef* encoder_timer_;      // 编码器定时器
        float wheel_diameter_mm_;               // 轮子直径（mm）
        uint16_t encoder_ppr_;                  // 编码器每转脉冲数（线数）
        float gear_ratio_;                      // 减速比
        float pulses_per_revolution_;           // 输出轴每转的脉冲数（考虑减速比和4倍频）
        
        int32_t last_count_;                    // 上次计数值
        int64_t total_pulses_;                  // 累计总脉冲数
        float speed_pps_;                       // 速度（脉冲/秒）
        float linear_velocity_mmps_;            // 线速度（mm/s）
        float angular_velocity_radps_;          // 角速度（rad/s）
        float last_linear_velocity_mmps_;
        
        // 卡尔曼滤波状态
        float kalman_linear_velocity_;          // 滤波后的线速度估计（mm/s）
        float kalman_p_;                        // 估计误差协方差
        float kalman_q_;                        // 过程噪声协方差
        float kalman_r_;                        // 测量噪声协方差
        
        // 处理计数器溢出
        int32_t handleOverflow(int32_t _current_count);
        
        // 卡尔曼滤波更新
        void kalmanUpdate(float measurement, float dt);
        void kalmanUpdate(float measurement, float dt, float q);  // 带自适应过程噪声的版本
        void kalmanUpdateWithPrediction(float measurement, float dt, float q, float r, float predicted);  // 带预测和自适应噪声的版本
    };
}

#endif // JGA25_370_ENCODER_HPP

