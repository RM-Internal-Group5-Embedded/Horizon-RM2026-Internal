#include "mpu6500.hpp"
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
    {
        // 2000度/秒量程
        gyro_scale_ = 2000.0f / 32768.0f;
        
        // 8g量程
        accel_scale_ = (8.0f * 9.81f) / 32768.0f;
    }
    
    bool MPU6500::init()
    {
        CS_HIGH();
        HAL_Delay(100);
        
        // 检查WHO_AM_I（MPU6500返回0x70）
        uint8_t who_am_i = readRegister(MPU6500_WHO_AM_I);
        if (who_am_i != 0x70) {
            return false;
        }
        
        // 复位设备
        writeRegister(MPU6500_PWR_MGMT_1, 0x80);
        HAL_Delay(100);
        
        // 唤醒设备，使用陀螺仪时钟
        writeRegister(MPU6500_PWR_MGMT_1, 0x01);
        HAL_Delay(10);
        
        // 使能加速度计和陀螺仪
        writeRegister(MPU6500_PWR_MGMT_2, 0x00);
        HAL_Delay(10);
        
        // 低通滤波器98Hz（平衡车推荐）
        writeRegister(MPU6500_CONFIG, 0x02);
        
        // 陀螺仪量程 ±2000度/秒
        writeRegister(MPU6500_GYRO_CONFIG, 0x18);  // 0x18 = 3<<3
        
        // 加速度计量程 ±8g
        writeRegister(MPU6500_ACCEL_CONFIG, 0x10);  // 0x10 = 2<<3
        
        HAL_Delay(10);
        return true;
    }
    
    bool MPU6500::update(SensorData& data, float dt)
    {
        uint8_t buffer[14];
        
        // 从加速度起始地址连续读14字节（加速度6 + 温度2 + 陀螺仪6）
        readRegisters(MPU6500_ACCEL_XOUT_H, buffer, 14);
        
        // 解析原始数据（大端序）
        int16_t ax_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
        int16_t ay_raw = (int16_t)((buffer[2] << 8) | buffer[3]);
        int16_t az_raw = (int16_t)((buffer[4] << 8) | buffer[5]);
        int16_t temp_raw = (int16_t)((buffer[6] << 8) | buffer[7]);
        int16_t gx_raw = (int16_t)((buffer[8] << 8) | buffer[9]);
        int16_t gy_raw = (int16_t)((buffer[10] << 8) | buffer[11]);
        int16_t gz_raw = (int16_t)((buffer[12] << 8) | buffer[13]);
        
        // 转换为物理值
        data.accel_x = ax_raw * accel_scale_;
        data.accel_y = ay_raw * accel_scale_;
        data.accel_z = az_raw * accel_scale_;
        
        data.gyro_x = gx_raw * gyro_scale_ - gyro_offset_x_;
        data.gyro_y = gy_raw * gyro_scale_ - gyro_offset_y_;
        data.gyro_z = gz_raw * gyro_scale_ - gyro_offset_z_;
        
        data.temp = temp_raw / 326.8f + 25.0f;
        
        // 用加速度计计算倾角（瞬时值，有噪声）
        float accel_pitch = atan2(data.accel_y, sqrt(data.accel_x * data.accel_x + data.accel_z * data.accel_z)) * 57.3f;
        float accel_roll = atan2(-data.accel_x, sqrt(data.accel_y * data.accel_y + data.accel_z * data.accel_z)) * 57.3f;
        
        // 互补滤波融合陀螺仪和加速度计
        // 陀螺仪积分（高频准确，低频漂移）
        pitch_ += data.gyro_y * dt;
        roll_ += data.gyro_x * dt;
        yaw_ += data.gyro_z * dt;
        
        // 与加速度计融合（低频准确，高频噪声）
        float alpha = 0.98f;  // 互补滤波系数（陀螺仪权重）
        pitch_ = alpha * pitch_ + (1.0f - alpha) * accel_pitch;
        roll_ = alpha * roll_ + (1.0f - alpha) * accel_roll;
        
        data.pitch = pitch_;
        data.roll = roll_;
        data.yaw = yaw_;
        
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
            
            HAL_Delay(1);
        }
        
        gyro_offset_x_ = sum_x / samples;
        gyro_offset_y_ = sum_y / samples;
        gyro_offset_z_ = sum_z / samples;
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
}

