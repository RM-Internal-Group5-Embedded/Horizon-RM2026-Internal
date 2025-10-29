#include "DJIMotor.hpp"
#include "MotorFunctions.hpp"
#include "PID.hpp"
#include "uart.hpp"

#include <cstdio>
#include <cstring>
#include <cmath>


struct ManualControl {
    //Four Mecanum Wheels
    M3508Functions* motor_front_left    = nullptr;
    M3508Functions* motor_front_right   = nullptr;
    M3508Functions* motor_back_left     = nullptr;
    M3508Functions* motor_back_right    = nullptr;
    
    MG90SFunctions* claw_servo          = nullptr;
};

// SBUS_t is deprecated; use uartdriver::ReceivedValue instead


class ERStatusControl {
private:
    int16_t front_left_rpm, front_right_rpm,
            back_left_rpm,  back_right_rpm;
public:
    enum ERState {
        IDLE, MANUAL, GOLD, MINING, DEPOSIT, ERROR
    };

    ManualControl manual_control;

    ERState current_state;

    ERStatusControl(M3508Functions& motor_front_left, M3508Functions& motor_front_right,
                    M3508Functions& motor_back_left, M3508Functions& motor_back_right) {
        this->manual_control.motor_front_left = &motor_front_left;
        this->manual_control.motor_front_right = &motor_front_right;    
        this->manual_control.motor_back_left = &motor_back_left;
        this->manual_control.motor_back_right = &motor_back_right;
    }

    ERStatusControl(M3508Functions& motor_front_left, M3508Functions& motor_front_right,
                    M3508Functions& motor_back_left, M3508Functions& motor_back_right,
                    MG90SFunctions& claw_servo) {
        this->manual_control.motor_front_left = &motor_front_left;
        this->manual_control.motor_front_right = &motor_front_right;    
        this->manual_control.motor_back_left = &motor_back_left;
        this->manual_control.motor_back_right = &motor_back_right;
        this->manual_control.claw_servo = &claw_servo;
    }   

    void switchState(uartdriver::ReceivedValue received_data);

    void goldMode();

    void idleMode();

    void manualMode(const uartdriver::ReceivedValue& received_data);

    void miningMode(const uartdriver::ReceivedValue& received_data);

    void traversal( uint16_t joystick_r_x, uint16_t joystick_r_y,
                    uint16_t joystick_l_x, uint16_t rpm_magnitude, 
                    uint16_t angle_magnitude);

    void setState(ERState state) {
        current_state = state;
    }

    ERState getState() const {
        return current_state;
    }
};