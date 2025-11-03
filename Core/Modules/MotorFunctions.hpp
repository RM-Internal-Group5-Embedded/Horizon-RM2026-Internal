
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
    float pid_integral = 0;
    float pid_prev_error = 0;
    float pid_error = 0;
    
    int8_t motor_id = 0;
    float pid_output = 0;
    int16_t prev_angle = 0;
    int32_t total_motor_counts = 0;

    uint32_t current_time = 0;
    uint32_t last_time = 0;

public:
    int8_t choice = 1;
    float RPM_KP = 150.0f;
    float RPM_KI = 0.1f;  
    float RPM_KD = 10.0f;
    uint16_t MAX_CURRENT = 16000;

    Modules::DJIMotors::MotorFeedback motor_feedback;

    MotorFunctions(int id,  pFDCAN_RxFifo0CallbackTypeDef       callback, 
                            pFDCAN_ErrorStatusCallbackTypeDef   errorCallback);

    void resetData();
    
    virtual void readMotorFeedback(uint8_t rxData[8]);

    void sendCurrent(int16_t current);

    void controlLoop(int choice, uint16_t magnitude);

    void stopLoop();

    // Calculate PID output and optionally send
    int16_t setRpmPID(int16_t target_rpm, float dt, bool send = false);
    int16_t setAnglePID(float target_angle, float dt, bool send = false);

    void setRpm(int16_t target_rpm);
    void setAngle(float target_angle);
    
    // Pack this motor's current into the data array at the correct position
    void packCurrentIntoData(uint8_t* data, int16_t current);
};

class M3508Functions : public MotorFunctions {
    public:
    M3508Functions(int id,  pFDCAN_RxFifo0CallbackTypeDef callback, 
                            pFDCAN_ErrorStatusCallbackTypeDef errorCallback,
                            uint16_t filterID2, uint16_t filterID1);

    void readMotorFeedback(uint8_t rxData[8]);

    void rotationControlLoop(int choice, uint16_t magnitude);

    void directionControlLoop(int choice, uint16_t magnitude);
};

class MG90SFunctions : public MotorFunctions {
    public:
    MG90SFunctions(int id, pFDCAN_RxFifo0CallbackTypeDef callback, 
                            pFDCAN_ErrorStatusCallbackTypeDef errorCallback,
                            uint16_t filterID2, uint16_t filterID1); 

    void readMotorFeedback(uint8_t rxData[8]);
};

class JGA25370Functions : public MotorFunctions {
    public:
    JGA25370Functions(int id, pFDCAN_RxFifo0CallbackTypeDef callback, 
                            pFDCAN_ErrorStatusCallbackTypeDef errorCallback,
                            uint16_t filterID2, uint16_t filterID1); 

    void readMotorFeedback(uint8_t rxData[8]);
};

class DMJ4310Functions : public MotorFunctions {
    public:
    DMJ4310Functions(int id, pFDCAN_RxFifo0CallbackTypeDef callback, 
                            pFDCAN_ErrorStatusCallbackTypeDef errorCallback,
                            uint16_t filterID2, uint16_t filterID1); 

    void ReadMotorFeedback(uint8_t rxData[8]);
};

class GM6020Functions : public MotorFunctions {
private:
    bool reference_set = false;
    int32_t reference_counts = 0;  // Position when motor was first powered on
    int32_t absolute_counts = 0;   // Total counts from reference
    
public:
    GM6020Functions(int id, pFDCAN_RxFifo0CallbackTypeDef callback, 
                            pFDCAN_ErrorStatusCallbackTypeDef errorCallback,
                            uint16_t filterID2, uint16_t filterID1);

    void readMotorFeedback(uint8_t rxData[8]) override;
    
    // Get total rotation in degrees from reference position (rest position)
    float getAbsoluteDegrees() const {
        return (float)absolute_counts * 360.0f / 8191.0f;
    }
    
    // Reset current position as the new reference (rest position)
    void setReferencePosition() {
        reference_set = true;
        reference_counts = total_motor_counts;
        absolute_counts = 0;
    }
};

#endif // MOTORFUNCTIONS_HPP