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
        , q0_(1.0f)
        , q1_(0.0f)
        , q2_(0.0f)
        , q3_(0.0f)
        , ex_int_(0.0f)
        , ey_int_(0.0f)
        , ez_int_(0.0f)
        , kp_(0.5f)
        , ki_(0.0f)
        , last_dt_(0.001f)
        , integral_limit_(0.5f)

    {
        // 2000度/秒量程
        gyro_scale_ = 2000.0f / 32768.0f;
        
        // 8g量程
        accel_scale_ = (16.0f * 9.81f) / 32768.0f;
        
        updateRotationMatrix();
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

    float ax = _ay;
    float ay = -_az;
    float az = -_ax;

    float gx = _gy;
    float gy = -_gz;
    float gz = -_gx;


    const float deg2rad = 0.017453292519943295f;
    float gx_rad = gx * deg2rad;
    float gy_rad = gy * deg2rad;
    float gz_rad = gz * deg2rad;
    
    data.accel_x = ax;
    data.accel_y = ay;
    data.accel_z = az;
    data.gyro_x = gx;
    data.gyro_y = gy;
    data.gyro_z = gz;
    data.temp = temp_raw / 326.8f + 25.0f;



    // ==================== Mahony 融合 ====================
    float acc_mag = sqrtf(ax*ax + ay*ay + az*az);
    const float ACC_MIN = 4.0f;   // ≈0.4g
    const float ACC_MAX = 15.0f;  // ≈1.5g
    if (acc_mag > ACC_MIN && acc_mag < ACC_MAX) {
        float inv_norm = 1.0f / acc_mag;
        ax *= inv_norm;
        ay *= inv_norm;
        az *= inv_norm;

        // 由四元数估计重力方向
        float vx = 2.0f * (q1_ * q3_ - q0_ * q2_);
        float vy = 2.0f * (q0_ * q1_ + q2_ * q3_);
        float vz = 1.0f - 2.0f * (q1_*q1_ + q2_*q2_);

        // 叉积误差
        float ex = (ay * vz - az * vy);
        float ey = (az * vx - ax * vz);
        float ez = (ax * vy - ay * vx);

        // 积分项（Ki）
        if (ki_ > 0.0f) {
            ex_int_ += ex * dt;
            ey_int_ += ey * dt;
            ez_int_ += ez * dt;
            if (ex_int_ > integral_limit_) ex_int_ = integral_limit_;
            if (ex_int_ < -integral_limit_) ex_int_ = -integral_limit_;
            if (ey_int_ > integral_limit_) ey_int_ = integral_limit_;
            if (ey_int_ < -integral_limit_) ey_int_ = -integral_limit_;
            if (ez_int_ > integral_limit_) ez_int_ = integral_limit_;
            if (ez_int_ < -integral_limit_) ez_int_ = -integral_limit_;

            gx_rad += 2.0f * ki_ * ex_int_;
            gy_rad += 2.0f * ki_ * ey_int_;
            gz_rad += 2.0f * ki_ * ez_int_;
        } else {
            ex_int_ = ey_int_ = ez_int_ = 0.0f;
        }

        // 比例项（Kp）
        gx_rad += 2.0f * kp_ * ex;
        gy_rad += 2.0f * kp_ * ey;
        gz_rad += 2.0f * kp_ * ez;
    }

    // 四元数微分方程（角速度已补偿）
        float qDot1 = 0.5f * (-q1_ * gx_rad - q2_ * gy_rad - q3_ * gz_rad);
        float qDot2 = 0.5f * ( q0_ * gx_rad - q3_ * gy_rad + q2_ * gz_rad);
        float qDot3 = 0.5f * ( q0_ * gy_rad + q3_ * gx_rad - q1_ * gz_rad);
        float qDot4 = 0.5f * ( q0_ * gz_rad - q2_ * gx_rad + q1_ * gy_rad);
        
        q0_ += qDot1 * dt;
        q1_ += qDot2 * dt;
        q2_ += qDot3 * dt;
        q3_ += qDot4 * dt;
        
        normalizeQuaternion();

    // 更新旋转矩阵
    updateRotationMatrix();

    // 提取欧拉角（度）
    const float rad2deg = 57.29577951308232f;
    pitch_ = -asinf(rotation_matrix_[2][0]) * rad2deg;
    roll_  = atan2f(rotation_matrix_[2][1], rotation_matrix_[2][2]) * rad2deg;
    yaw_   = atan2f(rotation_matrix_[1][0], rotation_matrix_[0][0]) * rad2deg;

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
            
            HAL_Delay(1);
        }
        
        gyro_offset_x_ = sum_x / samples;
        gyro_offset_y_ = sum_y / samples;
        gyro_offset_z_ = sum_z / samples;

        resetAttitude();
    }
    
    void MPU6500::setMahonyGains(float kp, float ki, float integral_limit)
    {
        this->kp_ = (kp >= 0.0f) ? kp : 0.0f;
        this->ki_ = (ki >= 0.0f) ? ki : 0.0f;
        this->integral_limit_ = (integral_limit > 0.0f) ? integral_limit : this->integral_limit_;
    }
    
    void MPU6500::resetAttitude(float pitch_deg, float roll_deg, float yaw_deg)
    {
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
        
        this->ex_int_ = this-> ey_int_ = this->ez_int_ = 0.0f;
        normalizeQuaternion();
        updateRotationMatrix();
        
        const float rad2deg = 57.29577951308232f;
        this->pitch_ = -asinf(rotation_matrix_[2][0]) * rad2deg;
        this->roll_  = atan2f(rotation_matrix_[2][1], rotation_matrix_[2][2]) * rad2deg;
        this->yaw_   = atan2f(rotation_matrix_[1][0], rotation_matrix_[0][0]) * rad2deg;
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
    
    void MPU6500::updateRotationMatrix()
    {
        float q1q1 = q1_ * q1_;
        float q2q2 = q2_ * q2_;
        float q3q3 = q3_ * q3_;
        
        float q0q1 = q0_ * q1_;
        float q0q2 = q0_ * q2_;
        float q0q3 = q0_ * q3_;
        float q1q2 = q1_ * q2_;
        float q1q3 = q1_ * q3_;
        float q2q3 = q2_ * q3_;
        
        this->rotation_matrix_[0][0] = 1.0f - 2.0f * (q2q2 + q3q3);
        this->rotation_matrix_[0][1] = 2.0f * (q1q2 - q0q3);
        this->rotation_matrix_[0][2] = 2.0f * (q1q3 + q0q2);
        this->rotation_matrix_[1][0] = 2.0f * (q1q2 + q0q3);
        this->rotation_matrix_[1][1] = 1.0f - 2.0f * (q1q1 + q3q3);
        this->rotation_matrix_[1][2] = 2.0f * (q2q3 - q0q1);
        
        this->rotation_matrix_[2][0] = 2.0f * (q1q3 - q0q2);
        this->rotation_matrix_[2][1] = 2.0f * (q2q3 + q0q1);
        this->rotation_matrix_[2][2] = 1.0f - 2.0f * (q1q1 + q2q2);
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
        float norm = this->q0_ * this->q0_ + this->q1_ * this->q1_ + this->q2_ * this->q2_ + this->q3_ * this->q3_;
        if (norm > 0.0f) {
            norm = 1.0f / sqrtf(norm);
            this->q0_ *= norm;
            this->q1_ *= norm;
            this->q2_ *= norm;
            this->q3_ *= norm;
        } else {
            this->q0_ = 1.0f;
            this->q1_ = this->q2_ = this->q3_ = 0.0f;
        }
    }
}

