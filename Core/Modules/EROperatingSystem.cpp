#include "EROperatingSystem.hpp"

const float CENTER = 1024.0f;
const float SBUS_SPAN = 783.0f;


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
    manual_control.motor_front_left ->stopLoop();
    manual_control.motor_front_right->stopLoop();
    manual_control.motor_back_left  ->stopLoop();
    manual_control.motor_back_right ->stopLoop();
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
    lx = apply_deadband(lx, 0.03f) * static_cast<float>(angle_magnitude);

    // Mecanum mix
    front_left_rpm  = static_cast<int16_t>(ry + rx + lx);
    front_right_rpm = static_cast<int16_t>(ry - rx - lx);
    back_left_rpm   = static_cast<int16_t>(ry - rx + lx);
    back_right_rpm  = static_cast<int16_t>(ry + rx - lx);

    // Calculate PID outputs for all motors (but don't send yet)
    manual_control.motor_front_left->motor_feedback.target_rpm = front_left_rpm;
    manual_control.motor_front_right->motor_feedback.target_rpm = front_right_rpm;
    manual_control.motor_back_left->motor_feedback.target_rpm = back_left_rpm;
    manual_control.motor_back_right->motor_feedback.target_rpm = back_right_rpm;
    
    // Send all four motor currents in ONE CAN message to 0x200
    extern FDCAN_HandleTypeDef hfdcan1;
    uint8_t data[8];
    
    // Get PID outputs for each motor
    uint32_t current_time = HAL_GetTick();
    float dt = 0.01f; // ~10ms assumed
    
    int16_t curr_lf = (int16_t)Modules::PID::calculate(
        (float)front_left_rpm, (float)manual_control.motor_front_left->motor_feedback.rpm,
        manual_control.motor_front_left->RPM_KP, manual_control.motor_front_left->RPM_KI, 
        manual_control.motor_front_left->RPM_KD, pid_integral_lf, pid_prev_error_lf, dt);
    int16_t curr_rf = (int16_t)Modules::PID::calculate(
        (float)front_right_rpm, (float)manual_control.motor_front_right->motor_feedback.rpm,
        manual_control.motor_front_right->RPM_KP, manual_control.motor_front_right->RPM_KI, 
        manual_control.motor_front_right->RPM_KD, pid_integral_rf, pid_prev_error_rf, dt);
    int16_t curr_lb = (int16_t)Modules::PID::calculate(
        (float)back_left_rpm, (float)manual_control.motor_back_left->motor_feedback.rpm,
        manual_control.motor_back_left->RPM_KP, manual_control.motor_back_left->RPM_KI, 
        manual_control.motor_back_left->RPM_KD, pid_integral_lb, pid_prev_error_lb, dt);
    int16_t curr_rb = (int16_t)Modules::PID::calculate(
        (float)back_right_rpm, (float)manual_control.motor_back_right->motor_feedback.rpm,
        manual_control.motor_back_right->RPM_KP, manual_control.motor_back_right->RPM_KI, 
        manual_control.motor_back_right->RPM_KD, pid_integral_rb, pid_prev_error_rb, dt);
    
    // Pack all currents into one message (motor IDs 1-4 → bytes 0-7)
    data[0] = (curr_lf >> 8) & 0xFF; data[1] = curr_lf & 0xFF;
    data[2] = (curr_rf >> 8) & 0xFF; data[3] = curr_rf & 0xFF;
    data[4] = (curr_lb >> 8) & 0xFF; data[5] = curr_lb & 0xFF;
    data[6] = (curr_rb >> 8) & 0xFF; data[7] = curr_rb & 0xFF;
    
    FDCAN_TxHeaderTypeDef txHeader;
    txHeader.Identifier = 0x200;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;
    
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, data);
}