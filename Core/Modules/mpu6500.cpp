#include "mpu6500.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include <cmath>

// MPU6500寄存器地址（使用constexpr，不占用内存且有类型检查）
namespace {
    constexpr uint8_t MPU6500_WHO_AM_I       = 0x75;  // 设备ID寄存器（应返回0x70）
    constexpr uint8_t MPU6500_PWR_MGMT_1     = 0x6B;  // 电源管理1（复位、时钟源）
    constexpr uint8_t MPU6500_PWR_MGMT_2     = 0x6C;  // 电源管理2（传感器使能）
    constexpr uint8_t MPU6500_CONFIG         = 0x1A;  // 配置寄存器（低通滤波器）
    constexpr uint8_t MPU6500_SMPLRT_DIV     = 0x19;  // 采样率分频器
    constexpr uint8_t MPU6500_GYRO_CONFIG    = 0x1B;  // 陀螺仪配置（量程）
    constexpr uint8_t MPU6500_ACCEL_CONFIG   = 0x1C;  // 加速度计配置（量程）
    constexpr uint8_t MPU6500_ACCEL_CONFIG_2 = 0x1D;  // 加速度计配置2（低通滤波）
    constexpr uint8_t MPU6500_INT_PIN_CFG    = 0x37;  // 中断引脚配置
    constexpr uint8_t MPU6500_INT_ENABLE     = 0x38;  // 中断使能
    constexpr uint8_t MPU6500_INT_STATUS     = 0x3A;  // 中断状态
    constexpr uint8_t MPU6500_ACCEL_XOUT_H   = 0x3B;  // 加速度X高字节（连续读起始地址）
    constexpr uint8_t MPU6500_TEMP_OUT_H     = 0x41;  // 温度高字节
    constexpr uint8_t MPU6500_GYRO_XOUT_H    = 0x43;  // 陀螺仪X高字节
}

// CS引脚操作
#define CS_LOW()   HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_RESET)
#define CS_HIGH()  HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_SET)

namespace mpu6500
{
    MPU6500::MPU6500()
        : gyro_offset_x_(0.0f)
        , gyro_offset_y_(0.0f)
        , gyro_offset_z_(0.0f)
        , pitch_(0.0f)
        , roll_(0.0f)
        , yaw_(0.0f)
        , q0_(1.0f)
        , q1_(0.0f)
        , q2_(0.0f)
        , q3_(0.0f)
        , beta_(0.1f)  // Default Madgwick beta (tunable)
        , last_dt_(0.001f)

    {
        // Gyroscope ±2000°/s range
        // Sensitivity: 16.4 LSB/°/s (from MPU6500 datasheet)
        // Scale: 1/16.4 = 0.060975 °/s per LSB
        // Alternative: 2000/32768 = 0.061035 (very close, using official sensitivity)
        gyro_scale_ = 0.063400002 ;  // More accurate: uses official sensitivity
        
        // Accelerometer ±8g range
        // Sensitivity: 4096 LSB/g (from MPU6500 datasheet)
        // Scale: (1/4096) * 9.81 = 0.002393 m/s² per LSB
        // FIX: Was (16.0 * 9.81) / 32768 = 0.004789 (WRONG - double the correct value)
        accel_scale_ = 9.81f / 4096.0f;  // Correct: uses official sensitivity
    }
    
    bool MPU6500::init()
    {
        CS_HIGH();
        // OPTIMIZATION: Use vTaskDelay instead of HAL_Delay (allows other tasks to run)
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // 检查WHO_AM_I（MPU6500返回0x70）
        uint8_t who_am_i = readRegister(MPU6500_WHO_AM_I);
        if (who_am_i != 0x70) {
            return false;
        }
        
        // 复位设备
        writeRegister(MPU6500_PWR_MGMT_1, 0x80);
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // 唤醒设备，使用陀螺仪时钟
        writeRegister(MPU6500_PWR_MGMT_1, 0x01);
        vTaskDelay(pdMS_TO_TICKS(10));
        
        // 使能加速度计和陀螺仪
        writeRegister(MPU6500_PWR_MGMT_2, 0x00);
        vTaskDelay(pdMS_TO_TICKS(10));
        
        // 低通滤波器: 250Hz for fast robot (was 98Hz)
        // Higher frequency = faster response, less delay, better for fast movement
        writeRegister(MPU6500_CONFIG, 0x00);  // 0x00 = 250Hz DLPF
        
        // 陀螺仪量程 ±2000度/秒 (optimal for fast robot)
        writeRegister(MPU6500_GYRO_CONFIG, 0x18);  // 0x18 = 3<<3
        
        // 加速度计量程 ±8g (good for fast acceleration)
        writeRegister(MPU6500_ACCEL_CONFIG, 0x10);  // 0x10 = 2<<3
        
        vTaskDelay(pdMS_TO_TICKS(10));
        return true;
    }
    
    bool MPU6500::update(SensorData& data, float dt)
    {
        clampDt(dt);
        
        uint8_t buffer[14];
    
        // 读取14字节数据
        readRegisters(MPU6500_ACCEL_XOUT_H, buffer, 14);
    
        // 解析原始数据
        int16_t ax_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
        int16_t ay_raw = (int16_t)((buffer[2] << 8) | buffer[3]);
        int16_t az_raw = (int16_t)((buffer[4] << 8) | buffer[5]);
        int16_t temp_raw = (int16_t)((buffer[6] << 8) | buffer[7]);
        int16_t gx_raw = (int16_t)((buffer[8] << 8) | buffer[9]);
        int16_t gy_raw = (int16_t)((buffer[10] << 8) | buffer[11]);
        int16_t gz_raw = (int16_t)((buffer[12] << 8) | buffer[13]);
        
        // 转换为物理单位
        float _ax = ax_raw * accel_scale_;
        float _ay = ay_raw * accel_scale_;
        float _az = az_raw * accel_scale_;
        
        float _gx = gx_raw * gyro_scale_ - gyro_offset_x_;
        float _gy = gy_raw * gyro_scale_ - gyro_offset_y_;
        float _gz = gz_raw * gyro_scale_ - gyro_offset_z_;

        // 坐标系转换（根据安装方向）
        float ax = _ay;
        float ay = -_az;
        float az = -_ax;

        float gx = _gy;
        float gy = -_gz;
        float gz = -_gx;

        // 存储原始传感器数据
        data.accel_x = ax;
        data.accel_y = ay;
        data.accel_z = az;
        data.gyro_x = gx;
        data.gyro_y = gy;
        data.gyro_z = gz;
        data.temperature = temp_raw / 340.0f + 36.53f;  // MPU6500温度公式

        const float deg2rad = 0.017453292519943295f;
        float gx_rad = gx * deg2rad;
        float gy_rad = gy * deg2rad;
        float gz_rad = gz * deg2rad;

        // ==================== Madgwick Filter ====================
        float qDot1 = 0.5f * (-q1_ * gx_rad - q2_ * gy_rad - q3_ * gz_rad);
        float qDot2 = 0.5f * ( q0_ * gx_rad + q2_ * gz_rad - q3_ * gy_rad);
        float qDot3 = 0.5f * ( q0_ * gy_rad - q1_ * gz_rad + q3_ * gx_rad);
        float qDot4 = 0.5f * ( q0_ * gz_rad + q1_ * gy_rad - q2_ * gx_rad);

        float acc_mag = sqrtf(ax*ax + ay*ay + az*az);
        const float ACC_MIN = 4.0f;   // ≈0.4g
        const float ACC_MAX = 15.0f;  // ≈1.5g
        if (acc_mag > ACC_MIN && acc_mag < ACC_MAX) {
            float inv_norm = 1.0f / acc_mag;
            ax *= inv_norm;
            ay *= inv_norm;
            az *= inv_norm;

            // 计算目标与估计方向之间的梯度
            float f1 = 2.0f * (q1_ * q3_ - q0_ * q2_) - ax;
            float f2 = 2.0f * (q0_ * q1_ + q2_ * q3_) - ay;
            float f3 = 2.0f * (0.5f - q1_ * q1_ - q2_ * q2_) - az;

            float s0 = -2.0f * q2_ * f1 + 2.0f * q1_ * f2;
            float s1 =  2.0f * q3_ * f1 + 2.0f * q0_ * f2 - 4.0f * q1_ * f3;
            float s2 = -2.0f * q0_ * f1 + 2.0f * q3_ * f2 - 4.0f * q2_ * f3;
            float s3 =  2.0f * q1_ * f1 + 2.0f * q2_ * f2;

            float norm_s = sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
            if (norm_s > 0.0f) {
                float inv_s = 1.0f / norm_s;
                s0 *= inv_s;
                s1 *= inv_s;
                s2 *= inv_s;
                s3 *= inv_s;

                qDot1 -= beta_ * s0;
                qDot2 -= beta_ * s1;
                qDot3 -= beta_ * s2;
                qDot4 -= beta_ * s3;
            }
        }
        
        q0_ += qDot1 * dt;
        q1_ += qDot2 * dt;
        q2_ += qDot3 * dt;
        q3_ += qDot4 * dt;
        
        normalizeQuaternion();

        // 计算欧拉角
        const float rad2deg = 57.29577951308232f;

        float sinr_cosp = 2.0f * (q0_ * q1_ + q2_ * q3_);
        float cosr_cosp = 1.0f - 2.0f * (q1_ * q1_ + q2_ * q2_);
        roll_ = atan2f(sinr_cosp, cosr_cosp) * rad2deg;

        float sinp = 2.0f * (q0_ * q2_ - q3_ * q1_);
        if (fabsf(sinp) >= 1.0f) {
            pitch_ = copysignf(90.0f, sinp);
        } else {
            pitch_ = asinf(sinp) * rad2deg;
        }

        float siny_cosp = 2.0f * (q0_ * q3_ + q1_ * q2_);
        float cosy_cosp = 1.0f - 2.0f * (q2_ * q2_ + q3_ * q3_);
        yaw_ = atan2f(siny_cosp, cosy_cosp) * rad2deg;

        // 存储计算出的倾角
        data.pitch = pitch_;
        data.roll  = roll_;
        data.yaw   = yaw_;

        return true;
    }
    
    void MPU6500::readGyro(float& gyro_x, float& gyro_y, float& gyro_z)
    {
        uint8_t buffer[6];
        readRegisters(MPU6500_GYRO_XOUT_H, buffer, 6);
        
        int16_t gx_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
        int16_t gy_raw = (int16_t)((buffer[2] << 8) | buffer[3]);
        int16_t gz_raw = (int16_t)((buffer[4] << 8) | buffer[5]);
        
        gyro_x = gx_raw * gyro_scale_ - gyro_offset_x_;
        gyro_y = gy_raw * gyro_scale_ - gyro_offset_y_;
        gyro_z = gz_raw * gyro_scale_ - gyro_offset_z_;
    }
    
    void MPU6500::readAccel(float& accel_x, float& accel_y, float& accel_z)
    {
        uint8_t buffer[6];
        readRegisters(MPU6500_ACCEL_XOUT_H, buffer, 6);
        
        int16_t ax_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
        int16_t ay_raw = (int16_t)((buffer[2] << 8) | buffer[3]);
        int16_t az_raw = (int16_t)((buffer[4] << 8) | buffer[5]);
        
        accel_x = ax_raw * accel_scale_;
        accel_y = ay_raw * accel_scale_;
        accel_z = az_raw * accel_scale_;
    }
    
    void MPU6500::calibrateGyro(uint16_t samples)
    {
        float sum_x = 0, sum_y = 0, sum_z = 0;
        
        for (uint16_t i = 0; i < samples; i++) {
            uint8_t buffer[6];
            readRegisters(MPU6500_GYRO_XOUT_H, buffer, 6);
            
            int16_t gx_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
            int16_t gy_raw = (int16_t)((buffer[2] << 8) | buffer[3]);
            int16_t gz_raw = (int16_t)((buffer[4] << 8) | buffer[5]);
            
            sum_x += gx_raw * gyro_scale_;
            sum_y += gy_raw * gyro_scale_;
            sum_z += gz_raw * gyro_scale_;
            
            // OPTIMIZATION: Use vTaskDelay instead of HAL_Delay (allows other tasks to run)
            // This prevents blocking the entire system during calibration (1000ms total)
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        
        gyro_offset_x_ = sum_x / samples;
        gyro_offset_y_ = sum_y / samples;
        gyro_offset_z_ = sum_z / samples;

        resetAttitude();
    }
    
    void MPU6500::setMadgwickBeta(float beta)
    {
        if (beta > 0.0f) {
            beta_ = beta;
        }
    }
    
    void MPU6500::resetAttitude(float pitch_deg, float roll_deg, float yaw_deg)
    {
        // If all angles are zero (default), initialize from accelerometer
        // This prevents initial yaw drop by starting with correct roll/pitch
        if (pitch_deg == 0.0f && roll_deg == 0.0f && yaw_deg == 0.0f) {
            // Read accelerometer to get initial roll/pitch
            float ax, ay, az;
            readAccel(ax, ay, az);
            
            // Calculate initial roll and pitch from accelerometer (assuming stationary)
            float acc_mag = sqrtf(ax*ax + ay*ay + az*az);
            if (acc_mag > 4.0f && acc_mag < 15.0f) {
                // Normalize accelerometer
                float inv_norm = 1.0f / acc_mag;
                ax *= inv_norm;
                ay *= inv_norm;
                az *= inv_norm;
                
                // Calculate roll and pitch from accelerometer
                roll_deg = atan2f(ay, az) * 57.29577951308232f;  // rad to deg
                pitch_deg = -asinf(ax) * 57.29577951308232f;
                // Yaw remains 0 (no magnetometer reference)
            }
        }
        
        const float deg2rad = 0.017453292519943295f;
        float half_roll  = 0.5f * roll_deg  * deg2rad;
        float half_pitch = 0.5f * pitch_deg * deg2rad;
        float half_yaw   = 0.5f * yaw_deg   * deg2rad;
        
        float cr = cosf(half_roll);
        float sr = sinf(half_roll);
        float cp = cosf(half_pitch);
        float sp = sinf(half_pitch);
        float cy = cosf(half_yaw);
        float sy = sinf(half_yaw);
        
        this->q0_ = cr * cp * cy + sr * sp * sy;
        this->q1_ = sr * cp * cy - cr * sp * sy;
        this->q2_ = cr * sp * cy + sr * cp * sy;
        this->q3_ = cr * cp * sy - sr * sp * cy;
        
        normalizeQuaternion();
        
        // 计算欧拉角
        const float rad2deg = 57.29577951308232f;
        float sinr_cosp = 2.0f * (q0_ * q1_ + q2_ * q3_);
        float cosr_cosp = 1.0f - 2.0f * (q1_ * q1_ + q2_ * q2_);
        roll_ = atan2f(sinr_cosp, cosr_cosp) * rad2deg;

        float sinp = 2.0f * (q0_ * q2_ - q3_ * q1_);
        if (fabsf(sinp) >= 1.0f) {
            pitch_ = copysignf(90.0f, sinp);
        } else {
            pitch_ = asinf(sinp) * rad2deg;
        }

        float siny_cosp = 2.0f * (q0_ * q3_ + q1_ * q2_);
        float cosy_cosp = 1.0f - 2.0f * (q2_ * q2_ + q3_ * q3_);
        yaw_ = atan2f(siny_cosp, cosy_cosp) * rad2deg;
    }
    
    // ==================== SPI读写 ====================
    
    uint8_t MPU6500::readRegister(uint8_t reg)
    {
        uint8_t tx_data = reg | 0x80;  // 读操作
        uint8_t rx_data = 0;
        
        CS_LOW();
        HAL_SPI_Transmit(&hspi1, &tx_data, 1, 100);
        HAL_SPI_Receive(&hspi1, &rx_data, 1, 100);
        CS_HIGH();
        
        return rx_data;
    }
    
    void MPU6500::writeRegister(uint8_t reg, uint8_t value)
    {
        uint8_t data[2];
        data[0] = reg & 0x7F;  // 写操作
        data[1] = value;
        
        CS_LOW();
        HAL_SPI_Transmit(&hspi1, data, 2, 100);
        CS_HIGH();
    }
    
    void MPU6500::readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length)
    {
        uint8_t tx_data = reg | 0x80;
        
        CS_LOW();
        HAL_SPI_Transmit(&hspi1, &tx_data, 1, 100);
        HAL_SPI_Receive(&hspi1, buffer, length, 100);
        CS_HIGH();
    }
    
    void MPU6500::clampDt(float& dt)
    {
        if (dt <= 0.0f || dt > 0.05f) {
            dt = last_dt_;
        } else {
            last_dt_ = dt;
        }
    }

    void MPU6500::normalizeQuaternion()
    {
        // Improved: Use fast inverse square root for better performance
        // Check if normalization is needed (avoid unnecessary computation)
        float norm = this->q0_ * this->q0_ + this->q1_ * this->q1_ + this->q2_ * this->q2_ + this->q3_ * this->q3_;
        
        // Only normalize if significantly off (improves numerical stability)
        const float NORM_THRESHOLD = 1.0001f;  // Allow small deviation to avoid constant normalization
        if (norm > NORM_THRESHOLD || norm < (1.0f / NORM_THRESHOLD)) {
            if (norm > 0.0f) {
                // Use fast inverse square root approximation for better performance
                // For embedded systems, regular sqrt is fine, but this is more accurate
                norm = 1.0f / sqrtf(norm);
                this->q0_ *= norm;
                this->q1_ *= norm;
                this->q2_ *= norm;
                this->q3_ *= norm;
            } else {
                // Invalid quaternion - reset to identity
                this->q0_ = 1.0f;
                this->q1_ = this->q2_ = this->q3_ = 0.0f;
            }
        }
    }
}

