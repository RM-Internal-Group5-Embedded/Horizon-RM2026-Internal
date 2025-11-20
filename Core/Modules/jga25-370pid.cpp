#include "jga25-370pid.hpp"
// 声明全局编码器（因为它们本来就是全局的）
extern jgaencoder::Jga25370Encoder encoder_left;
extern jgaencoder::Jga25370Encoder encoder_right;
namespace arpid
{
    // ==================== PID控制器实现 ====================
    PID::PID(float kp, float ki, float kd)
        : kp_(kp), ki_(ki), kd_(kd), integral_(0.0f), last_error_(0.0f), out_min_(-1000.0f), out_max_(1000.0f), integral_min_(-500.0f), integral_max_(500.0f)
    {
    }

    void PID::setParams(float kp, float ki, float kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    void PID::setOutputLimits(float min, float max)
    {
        out_min_ = min;
        out_max_ = max;
    }

    void PID::setIntegralLimits(float min, float max)
    {
        integral_min_ = min;
        integral_max_ = max;
    }

    float PID::compute(float setpoint, float measurement, float dt)
    {
        error = setpoint - measurement;

        // P项
        float p_term = kp_ * error;

        // I项
        integral_ += error * dt;
        if (integral_ > integral_max_)
            integral_ = integral_max_;
        if (integral_ < integral_min_)
            integral_ = integral_min_;
        float i_term = ki_ * integral_;
        float derivative = (error - last_error_) / dt;
        const float alpha = 0.3f;
        derivative = alpha * derivative + (1.0f - alpha) * last_derivative_;
        float d_term = kd_ * derivative;

        last_error_ = error;
        last_derivative_ = derivative;

        // 总输出
        output = p_term + i_term + d_term;

        // 输出限幅
        if (output > out_max_)
            output = out_max_;
        if (output < out_min_)
            output = out_min_;

        return output;
    }

    void PID::reset()
    {
        integral_ = 0.0f;
        last_error_ = 0.0f;
    }

    // ==================== AR控制器实现 ====================

    AR::AR(jgamotor::Jga25370 *motor,
           jgaencoder::Jga25370Encoder *encoder,
           mpu6500::MPU6500 *mpu)
        : motor_(motor)
        , encoder_(encoder)
        , mpu_(mpu)
        , angle_pid_(0.0f, 0.0f, 0.0f)
        , velocity_pid_(0.0f, 0.0f, 0.0f)
        , yaw_pid_(0.0f, 0.0f, 0.0f)
        , enabled_(false)
        , target_velocity_(0.0f)
        , current_angle_(0.0f)
        , current_velocity_(0.0f)
        , current_yaw(0.0f)
        , target_angle_offset_(0.0f)
        , angle_feedforward_gain_(0.0f)
        , velocity_feedforward_gain_(0.0f)
        , yaw_target_deg_(0.0f)
        , yaw_sign_(1)
        , yaw_correction_(0)
        , velocity_cmd_(0)
        , polarity_(1)
    {
        angle_pid_.setOutputLimits(-1000.0f, 1000.0f); // 输出限幅
        angle_pid_.setIntegralLimits(-50.0f, 50.0f);
        yaw_pid_.setIntegralLimits(-50.0f, 50.0f);
        velocity_pid_.setOutputLimits(-10.0f, 10.0f); // 速度环输出限制为角度偏移量
        velocity_pid_.setIntegralLimits(-50.0f, 50.0f);
        yaw_pid_.setOutputLimits(-150.0f, 150.0f); 
    }

    void AR::init()
    {
        enabled_ = false;

    }

    void AR::update(float dt)
    {
        if (!enabled_)
        {
            motor_->stop();
            return;
        }

        // 读取传感器
        current_angle_ = -mpu_->getPitch();
        current_velocity_ = (encoder_left.getLinearVelocity() - encoder_right.getLinearVelocity()) / 2;
        // current_yaw = mpu_->getYaw();
        
        // //转向环（外外环）
        // yaw_correction_ = yaw_pid_.compute(yaw_target_deg_, current_yaw, dt);
        // velocity_cmd_ = target_velocity_- yaw_sign_ * yaw_correction_;

        // 速度环（外环）
        target_angle = velocity_pid_.compute(target_velocity_, current_velocity_, dt);

        // 角度环（内环)
        motor_output = angle_pid_.compute(target_angle - target_angle_offset_, current_angle_, dt);
        // // 倾角保护
        // if (current_angle_ > 45.0f || current_angle_ < -45.0f) {
        //     motor_->stop();
        //     enabled_ = false;
        //     return;
        // }

        // 控制电机
        int16_t pwm = limitMotorOutput(motor_output);
        {
            if (pwm > 0)
            {
                motor_->forward(pwm);
            }
            else if (pwm < 0)
            {
                motor_->backward(-pwm);
            }
            else
            {
                motor_->stop();
            }
        }
        
    }

    void AR::setTargetVelocity(float velocity)
    {
        target_velocity_ = velocity;
    }
    void AR::enable()
    {
        angle_pid_.reset();
        velocity_pid_.reset();
        enabled_ = true;
    }

    void AR::disable()
    {
        enabled_ = false;
        motor_->stop();
    }

    void AR::setAnglePID(float kp, float ki, float kd)
    {
        angle_pid_.setParams(kp, ki, kd);
    }

    void AR::setVelocityPID(float kp, float ki, float kd, int8_t polarity)
    {
        velocity_pid_.setParams(kp, ki, kd);
        polarity_ = polarity;
    }

    void AR::setYawPID(float kp, float ki, float kd)
    {
        yaw_pid_.setParams(kp, ki, kd);
    }
    void AR::setYawTarget(float yaw_deg)
    {
        yaw_target_deg_ = yaw_deg;
    }
    void AR::setYawSign(int8_t sign)
    {
        yaw_sign_ = (sign >= 0) ? 1 : -1;
    } // 左轮+1，右轮-1（或相反，按接线调整）
    void AR::setTargetAngleOffset(float angle)
    {
        target_angle_offset_ = angle;
    }

    int16_t AR::limitMotorOutput(float output)
    {
        const float MIN_PWM = 80.0f;   // 最小启动PWM（测试得出）
        const float MAX_PWM = 1000.0f; // 最大PWM
        // const float DEAD_ZONE = 70.0f;  // 死区阈值（略小于最小启动PWM）

        // 死区处理
        // if (output > -DEAD_ZONE && output < DEAD_ZONE) {
        //     return 0;
        // }

        // PWM映射：将PID输出映射到实际可用范围
        // PID输出范围：[-1000, 1000]
        // 实际PWM范围：[80, 1000]（去除死区）
        float mapped_pwm = 0;

        if (output > 0)
        {
            // 正转：映射到 [MIN_PWM, MAX_PWM]
            // 线性映射：DEAD_ZONE -> MIN_PWM, MAX_PWM -> MAX_PWM
            mapped_pwm = MIN_PWM + output * (MAX_PWM - MIN_PWM) / MAX_PWM;
        }
        else if (output < 0)
        {
            // 反转：映射到 [-MAX_PWM, -MIN_PWM]
            mapped_pwm = -MIN_PWM + output * (MAX_PWM - MIN_PWM) / MAX_PWM;
        }

        // 限幅
        if (mapped_pwm > MAX_PWM)
            mapped_pwm = MAX_PWM;
        if (mapped_pwm < -MAX_PWM)
            mapped_pwm = -MAX_PWM;

        return (int16_t)mapped_pwm;
    }
}
