#ifndef MPU6500_HPP
#define MPU6500_HPP

#include "main.h"
#include "spi.h"
#include <cstdint>

namespace mpu6500
{
    // 传感器数据结构
    struct SensorData {
        // 陀螺仪（角速度，单位：度/秒）
        float gyro_x;
        float gyro_y;
        float gyro_z;
        
        // 加速度计（单位：m/s²）
        float accel_x;
        float accel_y;
        float accel_z;
        
        // 温度（单位：°C）
        float temp;
        
        // 计算出的倾角（单位：度）
        float pitch;  // 俯仰角（前后倾斜）
        float roll;   // 横滚角（左右倾斜）
        float yaw;    // 偏航角（水平转向）
    };
    
    class MPU6500
    {
    public:
        // 构造函数（SPI句柄会自动从main.h获取）
        MPU6500();
        
        // 初始化MPU6500
        bool init();
        
        // 读取所有传感器数据并计算倾角
        bool update(SensorData& data, float dt);
        
        // 只读取陀螺仪（最常用，平衡车核心）
        void readGyro(float& gyro_x, float& gyro_y, float& gyro_z);
        
        // 只读取加速度计
        void readAccel(float& accel_x, float& accel_y, float& accel_z);
        
        // 校准陀螺仪零点（平衡车启动时静止状态调用）
        void calibrateGyro(uint16_t samples = 1000);

        // Mahony滤波参数与姿态管理
        void setMahonyGains(float kp, float ki, float integral_limit = 0.5f);
        void resetAttitude(float pitch_deg = 0.0f,
                           float roll_deg = 0.0f,
                           float yaw_deg = 0.0f);
        
        // 获取当前倾角
        float getPitch() const { return pitch_; }
        float getRoll() const { return roll_; }
        float getYaw() const { return yaw_; }
        
        // 重置yaw角（用于消除累积漂移）
        void resetYaw() { yaw_ = 0.0f; }
        
    private:
        // 陀螺仪零点偏移
        float gyro_offset_x_;
        float gyro_offset_y_;
        float gyro_offset_z_;
        
        // 倾角（互补滤波结果）
        float pitch_;
        float roll_;
        float yaw_;
        
        // 转换系数
        float gyro_scale_;
        float accel_scale_;
        
        // Mahony滤波器状态
        float q0_, q1_, q2_, q3_;
        float ex_int_, ey_int_, ez_int_;
        float rotation_matrix_[3][3];
        float kp_;
        float ki_;
        float last_dt_;
        float integral_limit_;
        
        void updateRotationMatrix();
        void clampDt(float& dt);
        void normalizeQuaternion();
        
        // SPI读写
        uint8_t readRegister(uint8_t reg);
        void writeRegister(uint8_t reg, uint8_t value);
        void readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length);
    };
}

#endif // MPU6500_HPP


