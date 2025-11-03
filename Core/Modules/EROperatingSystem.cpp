#include "EROperatingSystem.hpp"

// Constructor implementation
ERStatusControl::ERStatusControl(M3508Functions& motor_front_left, M3508Functions& motor_front_right,
                                 M3508Functions& motor_back_left, M3508Functions& motor_back_right, 
                                 float lengthx, float lengthy) {
    this->manual_control.motor_front_left = &motor_front_left;
    this->manual_control.motor_front_right = &motor_front_right;    
    this->manual_control.motor_back_left = &motor_back_left;
    this->manual_control.motor_back_right = &motor_back_right;
    lengthX = lengthx;
    lengthY = lengthy;
    
    // Initialize shared TX header once (for 0x200)
    motor_tx_header.Identifier = 0x200;
    motor_tx_header.IdType = FDCAN_STANDARD_ID;
    motor_tx_header.TxFrameType = FDCAN_DATA_FRAME;
    motor_tx_header.DataLength = FDCAN_DLC_BYTES_8;
    motor_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    motor_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    motor_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    motor_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    motor_tx_header.MessageMarker = 0;
}

const float CENTER = 1024.0f;
const float SBUS_SPAN = 783.0f;

//Plan: use FSM to enable multiple states: |, & 0x0008 

void ERStatusControl::switchState(const uartdriver::ReceivedValue& received_data) {
    if(received_data.channel_10 <=1024) {
        current_state = IDLE;
    } else {
        current_state = GOLD;
        if(received_data.channel_7 > 1500) {
        current_state = MANUAL;
        }
        if(received_data.channel_8 > 1500) {
        current_state = MINING;
        }
    }
    
}

void ERStatusControl::goldMode(){
    traversal(1024, 1807, 0, 100, 30);
    
}

void ERStatusControl::idleMode() {
    // Send zero current to all 4 motors
    sendMotorCurrents(0, 0, 0, 0);
    
    // Reset PID states in each motor
    manual_control.motor_front_left->resetData();
    manual_control.motor_front_right->resetData();
    manual_control.motor_back_left->resetData();
    manual_control.motor_back_right->resetData();
}

void ERStatusControl::manualMode(const uartdriver::ReceivedValue& received_data) {
    traversal(received_data.channel_1, received_data.channel_2, received_data.channel_4, 1000, 500);
    
}

void ERStatusControl::miningMode(const uartdriver::ReceivedValue& received_data) {
    traversal(received_data.channel_1, received_data.channel_2, received_data.channel_4, 300, 100);
    
}

//Relative directional control
void ERStatusControl::traversal( uint16_t joystick_r_x, uint16_t joystick_r_y,
                uint16_t joystick_l_x, uint16_t rpm_magnitude, 
                uint16_t angle_magnitude) {
    
    auto apply_deadband = [](float v, float db) -> float {
        return (fabsf(v) < db) ? 0.0f : v;
    };
    auto curve_half_quad = [](float v) -> float {
        float sign = v >= 0.0f ? 1.0f : -1.0f;
        float a = fabsf(v);
        // curvature: 0.5 * x^2 in [0, 0.5], preserve sign
        return sign * (0.5f * a * a);
    };

    float rx = (static_cast<float>(joystick_r_x) - CENTER) / SBUS_SPAN;
    float ry = (static_cast<float>(joystick_r_y) - CENTER) / SBUS_SPAN;
    float lx = (static_cast<float>(joystick_l_x) - CENTER) / SBUS_SPAN;

    // Non-linear mapping for RPM axes (rx, ry): 0.5*x^2 curvature
    rx = curve_half_quad(apply_deadband(rx, 0.03f)) * static_cast<float>(rpm_magnitude);
    ry = curve_half_quad(apply_deadband(ry, 0.03f)) * static_cast<float>(rpm_magnitude);
    // Keep rotation linear
    lx = apply_deadband(lx, 0.03f) * static_cast<float>(angle_magnitude) ;
    //* (lengthX + lengthY);

    // Mecanum mix
    front_left_rpm  = static_cast<int16_t>(ry + rx + lx);
    front_right_rpm = static_cast<int16_t>(ry - rx - lx);
    back_left_rpm   = static_cast<int16_t>(ry - rx + lx);
    back_right_rpm  = static_cast<int16_t>(ry + rx - lx);

    // Calculate timing
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // Reset timing on first call or after long gap
    if (last_time == 0 || (current_time - last_time) > 1000) {
        last_time = current_time;
    }
    
    float dt = (current_time - last_time) / 1000.0f;
    if (dt <= 0 || dt > 0.1f) dt = 0.01f;
    last_time = current_time;
    
    // Use motor class PID methods (send = false, just calculate)
    int16_t curr_lf = manual_control.motor_front_left->setRpmPID(front_left_rpm, dt, false);
    int16_t curr_rf = manual_control.motor_front_right->setRpmPID(front_right_rpm, dt, false);
    int16_t curr_lb = manual_control.motor_back_left->setRpmPID(back_left_rpm, dt, false);
    int16_t curr_rb = manual_control.motor_back_right->setRpmPID(back_right_rpm, dt, false);
    
    // Send combined message
    sendMotorCurrents(curr_lf, curr_rf, curr_lb, curr_rb);
}

void ERStatusControl::sendMotorCurrents(int16_t curr_lf, int16_t curr_rf, int16_t curr_lb, int16_t curr_rb) {
    extern FDCAN_HandleTypeDef hfdcan1;
    uint8_t data[8] = {0}; // Zero initialize
    
    // Use motor class methods to pack each current into data array
    if (manual_control.motor_front_left)
        manual_control.motor_front_left->packCurrentIntoData(data, curr_lf);
    if (manual_control.motor_front_right)
        manual_control.motor_front_right->packCurrentIntoData(data, curr_rf);
    if (manual_control.motor_back_left)
        manual_control.motor_back_left->packCurrentIntoData(data, curr_lb);
    if (manual_control.motor_back_right)
        manual_control.motor_back_right->packCurrentIntoData(data, curr_rb);
    
    // Send using pre-initialized TX header
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &motor_tx_header, data);
}