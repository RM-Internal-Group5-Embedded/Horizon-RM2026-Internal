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
    GM6020Functions* gm6020_claw        = nullptr;
    DMJ4310Functions* base_claw_servo   = nullptr;
    GM6020Functions* small_claw_servo   = nullptr;
    
    uint16_t manual_rpm = 1000;
    uint16_t manual_rotate_rpm = 300;
    uint16_t mining_rpm = 300;
    uint16_t mining_rotate_rpm = 100;
};

// SBUS_t is deprecated; use uartdriver::ReceivedValue instead


class ERStatusControl {
private:
    int16_t front_left_rpm, front_right_rpm,
            back_left_rpm,  back_right_rpm;
    float lengthX, lengthY;
    
    // Shared TX header for motor commands (initialized once)
    FDCAN_TxHeaderTypeDef motor_tx_header;
public:
    enum ERState {
        IDLE, MANUAL, GOLD, MINING, DEPOSIT, ER_ERROR
    };
    enum class ERState_Claw {
        IDLE, CLASP, RELEASE
    };

    ManualControl manual_control;

    ERState current_state;
    ERState_Claw current_state_claw;

    ERStatusControl(M3508Functions& motor_front_left, M3508Functions& motor_front_right,
                    M3508Functions& motor_back_left, M3508Functions& motor_back_right, 
                    float lengthx, float lengthy, GM6020Functions* gm6020_claw = nullptr);


    ERStatusControl(M3508Functions& motor_front_left, M3508Functions& motor_front_right,
                    M3508Functions& motor_back_left, M3508Functions& motor_back_right,
                    DMJ4310Functions& base_claw_servo, GM6020Functions& small_claw_servo) {
        this->manual_control.motor_front_left = &motor_front_left;
        this->manual_control.motor_front_right = &motor_front_right;    
        this->manual_control.motor_back_left = &motor_back_left;
        this->manual_control.motor_back_right = &motor_back_right;
        this->manual_control.base_claw_servo = &base_claw_servo;
        this->manual_control.small_claw_servo = &small_claw_servo;
        current_state_claw = ERState_Claw::IDLE;
    }   

    void switchState(const uartdriver::ReceivedValue& received_data);

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
    
    void setClawState(ERState_Claw state) {
        current_state_claw = state;
    }

    ERState_Claw getClawState() const {
        return current_state_claw;
    }
    
    // Update motors based on current state
    void update(const uartdriver::ReceivedValue& received_data);

    // Send combined motor currents to all 4 motors
    void sendMotorCurrents(int16_t curr_lf, int16_t curr_rf, int16_t curr_lb, int16_t curr_rb);
    
    // Helper functions for input processing (modify values by reference)
    static void applyDeadband(float& value, float deadband);
    static void applyCurveHalfQuad(float& value);
};

// Separate control system for claw motors
class ERClawControl {
private:
    // TX headers for different motor groups
    FDCAN_TxHeaderTypeDef gm6020_tx_header;  // 0x2FF for GM6020 (motors 5-8)
    FDCAN_TxHeaderTypeDef m3508_tx_header;   // 0x1FF for M3508 (motors 5-8)
    
public:
    enum class ClawState {
        IDLE, CLASP, RELEASE
    };
    
    struct ClawMotors {
        DMJ4310Functions* base_claw = nullptr;
        GM6020Functions* small_claw = nullptr;
        M3508Functions* m3508_claw = nullptr;  // 5th M3508 motor for claw
    };
    
    ClawMotors claw_motors;
    ClawState current_state;
    
    ERClawControl(DMJ4310Functions& base_motor, GM6020Functions& small_motor);
    
    ERClawControl(DMJ4310Functions& base_motor, GM6020Functions& small_motor, M3508Functions& m3508_motor);
    
    void switchState(const uartdriver::ReceivedValue& received_data);
    
    void idleMode();
    
    void claspMode();
    
    void releaseMode();
    
    void setState(ClawState state) {
        current_state = state;
    }
    
    ClawState getState() const {
        return current_state;
    }
    
    // Update claw motors based on state
    void update();
    
    // Send claw motor currents
    void sendClawCurrents(int16_t gm6020_current, int16_t m3508_current);
};
