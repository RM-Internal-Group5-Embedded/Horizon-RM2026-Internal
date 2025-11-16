
#pragma once
#ifndef MOTORFUNCTIONS_HPP
#define MOTORFUNCTIONS_HPP
#include "DJIMotor.hpp"
#include "uart.hpp"
#include "PID.hpp"
#include "gpio.h"
#include "main.h"
#include <cstdio>
#include <cstring>
#include <cmath>


class MotorFunctions {
protected:
    FDCAN_FilterTypeDef filter;
    FDCAN_TxHeaderTypeDef txHeader;
    uint8_t data[8];
    int8_t motor_id = 0;
    
    // Internal state tracking
    int16_t prev_angle = 0;
    int32_t total_motor_counts = 0;
    
    // PID control state
    float pid_integral = 0;
    float pid_prev_error = 0;
    uint32_t last_time = 0;

public:
    // Essential feedback data
    Modules::DJIMotors::MotorFeedback motor_feedback;
    
    // PID gains (public for tuning)
    float RPM_KP = 150.0f;
    float RPM_KI = 0.1f;  
    float RPM_KD = 10.0f;
    int16_t MAX_CURRENT = 16000;

    MotorFunctions(int id,  pFDCAN_RxFifo0CallbackTypeDef       callback, 
                            pFDCAN_ErrorStatusCallbackTypeDef   errorCallback);

    virtual void resetData();
    
    // Reset only PID state (integral and previous error) without resetting angle tracking
    void resetPIDState() {
        pid_integral = 0;
        pid_prev_error = 0;
    }
    
    virtual void readMotorFeedback(uint8_t rxData[8]);

    virtual void sendCurrent(int16_t current);

    void controlLoop(int choice, uint16_t magnitude);

    void stopLoop();

    // Calculate PID output and optionally send
    virtual int16_t setRpmPID(int16_t target_rpm, float dt, bool send = false);
    virtual int16_t setAnglePID(float target_angle, float dt, bool send = false);

    void setRpm(int16_t target_rpm);
    void setAngle(float target_angle);
    
    // Pack this motor's current into the data array at the correct position
    void packCurrentIntoData(uint8_t* data, int16_t current);
    
    // Setter for TX header (needed for ERClawControl to override TX ID)
    void setTxHeader(const FDCAN_TxHeaderTypeDef& header) { txHeader = header; }
};

class M3508Functions : public MotorFunctions {
    public:
    bool initialized = false;
    M3508Functions(int id,  pFDCAN_RxFifo0CallbackTypeDef callback, 
                            pFDCAN_ErrorStatusCallbackTypeDef errorCallback,
                            uint16_t filterID2, uint16_t filterID1);

    void readMotorFeedback(uint8_t rxData[8]) override;

    void rotationControlLoop(int choice, uint16_t magnitude);

    void directionControlLoop(int choice, uint16_t magnitude);
};

class MG90SFunctions : public MotorFunctions {
    public:
    MG90SFunctions(int id, pFDCAN_RxFifo0CallbackTypeDef callback, 
                            pFDCAN_ErrorStatusCallbackTypeDef errorCallback,
                            uint16_t filterID2, uint16_t filterID1); 

    void readMotorFeedback(uint8_t rxData[8]) override;
};

class JGA25370Functions : public MotorFunctions {
    public:
    JGA25370Functions(int id, pFDCAN_RxFifo0CallbackTypeDef callback, 
                            pFDCAN_ErrorStatusCallbackTypeDef errorCallback,
                            uint16_t filterID2, uint16_t filterID1); 

    void readMotorFeedback(uint8_t rxData[8]) override;
};

class DMJ4310Functions : public MotorFunctions {
private:
    // MIT protocol parameter ranges (can be configured via debug assistant)
    // 1:1 ratio - motor position = output position
    float p_min = -12.5f;   // Position min (radians)
    float p_max = 12.5f;    // Position max (radians)
    float v_min = -45.0f;   // Velocity min (rad/s)
    float v_max = 45.0f;    // Velocity max (rad/s)
    float t_min = -18.0f;   // Torque min (Nm)
    float t_max = 18.0f;    // Torque max (Nm)
    
public:
    bool initialized = false;
    
    // DMJ4310 has 1:1 gear reduction (direct drive)
    static constexpr float REDUCTION_RATIO = 1.0f;
    // Error code from feedback (0=no error, 8-E=various errors)
    uint8_t error_code = 0;
    uint8_t rx_motor_id = 0;
    
    DMJ4310Functions(int id, pFDCAN_RxFifo0CallbackTypeDef callback, 
                            pFDCAN_ErrorStatusCallbackTypeDef errorCallback,
                            uint16_t filterID2, uint16_t filterID1); 

    void readMotorFeedback(uint8_t rxData[8]) override;
    
    // Override resetData to also clear DMJ4310-specific error state
    void resetData();
    
    // MIT protocol: send position, velocity, Kp, Kd, and torque feedforward
    void sendMITCommand(float p_des, float v_des, float kp, float kd, float t_ff);
    
    // Enable/disable motor (must enable before use)
    void enableMotor();
    void disableMotor();
    
    // Set current position as zero reference (MIT protocol: 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFE)
    void setZeroPosition();
    
    // Try enabling with different motor IDs (broadcast-like behavior)
    void enableMotorBroadcast();
    
    // Set parameter ranges (must match motor configuration)
    void setRanges(float p_min, float p_max, float v_min, float v_max, float t_min, float t_max);
    
    // Get output shaft position (1:1 direct drive)
    float getOutputPosition() const { return motor_feedback.top_shaft_angle; }
    
    // Get output shaft velocity (1:1 direct drive)
    float getOutputVelocity() const { return motor_feedback.rpm / 9.5493f; }  // Convert RPM to rad/s
    
    // Get output shaft torque (1:1 direct drive)
    float getOutputTorque() const { return motor_feedback.current / 1000.0f; }  // Convert mNm to Nm
    
    // Check for errors (returns true if error exists)
    bool hasError() const { return error_code != 0; }
    
    // Get error description
    const char* getErrorString() const;
    
private:
    // Helper functions to encode/decode MIT protocol
    uint16_t floatToUint(float x, float x_min, float x_max, int bits);
    float uintToFloat(uint16_t x_int, float x_min, float x_max, int bits);
};

class GM6020Functions : public MotorFunctions {
private:
    public:
    bool initialized = false;
    float maxRpm = 60;
    GM6020Functions(int id, pFDCAN_RxFifo0CallbackTypeDef callback, 
                            pFDCAN_ErrorStatusCallbackTypeDef errorCallback,
                            uint16_t filterID2, uint16_t filterID1);

    void readMotorFeedback(uint8_t rxData[8]) override;

    
};

#endif // MOTORFUNCTIONS_HPP