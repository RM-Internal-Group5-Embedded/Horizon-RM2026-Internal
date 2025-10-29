
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
    float RPM_KP = 100.0f;
    float RPM_KI = 0.1f;  
    float RPM_KD = 2.0f;
    uint16_t MAX_CURRENT = 16000;

    Modules::DJIMotors::MotorFeedback motor_feedback;

    MotorFunctions(int id,  pFDCAN_RxFifo0CallbackTypeDef       callback, 
                            pFDCAN_ErrorStatusCallbackTypeDef   errorCallback);

    void resetData();
    
    virtual void readMotorFeedback(uint8_t rxData[8]);

    void sendCurrent(int16_t current);

    void controlLoop(int choice, uint16_t magnitude);

    void stopLoop();

    float setRpmPID(int16_t target_rpm, float dt);
    float setAnglePID(float target_angle, float dt);

    void setRpm(int16_t target_rpm);
    void setAngle(float target_angle);
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

    void ReadMotorFeedback(uint8_t rxData[8]);
};

class JGA25370Functions : public MotorFunctions {
    public:
    JGA25370Functions(int id, pFDCAN_RxFifo0CallbackTypeDef callback, 
                            pFDCAN_ErrorStatusCallbackTypeDef errorCallback,
                            uint16_t filterID2, uint16_t filterID1); 

    void ReadMotorFeedback(uint8_t rxData[8]);
};

#endif // MOTORFUNCTIONS_HPP