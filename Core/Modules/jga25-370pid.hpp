#ifndef JGA25_370PID_HPP
#define JGA25_370PID_HPP

#include "jga25-370.hpp"
#include "jga25-370-encoder.hpp"
#include "mpu6500.hpp"
#include <cstdint>

namespace arpid
{
    // PID控制器
    class PID
    {
    public:
        PID(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f);
        
        void setParams(float kp, float ki, float kd);
        void setOutputLimits(float min, float max);
        void setIntegralLimits(float min, float max);
        
        float compute(float setpoint, float measurement, float dt);
        void reset();
        
    private:
        float kp_, ki_, kd_;
        float integral_;
        float last_error_;
        float out_min_, out_max_;
        float integral_min_, integral_max_;
    };
    
    // AR控制器
    class AR
    {
    public:
        AR(jgamotor::Jga25370* motor, 
           jgaencoder::Jga25370Encoder* encoder,
           mpu6500::MPU6500* mpu);
        
        void init();
        void update(float dt);
        
        void setTargetVelocity(float velocity);
        void enable();
        void disable();
        
        void setAnglePID(float kp, float ki, float kd);
        void setVelocityPID(float kp, float ki, float kd);
        
        float getCurrentAngle() const { return current_angle_; }
        float getCurrentVelocity() const { return current_velocity_; }
        
    private:
        jgamotor::Jga25370* motor_;
        jgaencoder::Jga25370Encoder* encoder_;
        mpu6500::MPU6500* mpu_;
        
        PID angle_pid_;
        PID velocity_pid_;
        
        bool enabled_;
        float target_velocity_;
        float current_angle_;
        float current_velocity_;
        
        int16_t limitMotorOutput(float output);
    };
}

#endif // JGA25_370PID_HPP


