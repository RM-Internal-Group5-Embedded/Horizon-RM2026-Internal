#include "EROperatingSystem.hpp"
const float CENTER = 1024.0f;
const float SBUS_SPAN = 660.0f;


void ERStatusControl::switchState(uartdriver::ReceivedValue received_data) {
    if(received_data.channel_10 > 1500) {
        current_state = GOLD;
    } else if(received_data.channel_10 < 500) {
        current_state = IDLE;
    }
    if(received_data.channel_7 > 1500) {
        current_state = MANUAL;
    }
    if(received_data.channel_8 > 1500) {
        current_state = MINING;
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
    traversal(received_data.channel_2, received_data.channel_3, received_data.channel_5, 1000, 500);
    
}

void ERStatusControl::miningMode(const uartdriver::ReceivedValue& received_data) {
    traversal(received_data.channel_2, received_data.channel_3, received_data.channel_5, 500, 200);
    
}

//Relative directional control
void ERStatusControl::traversal( uint16_t joystick_r_x, uint16_t joystick_r_y,
                uint16_t joystick_l_x, uint16_t rpm_magnitude, 
                uint16_t angle_magnitude) {
    
    auto apply_deadband = [](float v, float db) -> float {
        return (fabsf(v) < db) ? 0.0f : v;
    };

    float rx = (static_cast<float>(joystick_r_x) - CENTER) / SBUS_SPAN;
    float ry = (static_cast<float>(joystick_r_y) - CENTER) / SBUS_SPAN;
    float lx = (static_cast<float>(joystick_l_x) - CENTER) / SBUS_SPAN;

    rx = apply_deadband(rx, 0.03f) * static_cast<float>(rpm_magnitude);
    ry = apply_deadband(ry, 0.03f) * static_cast<float>(rpm_magnitude);
    lx = apply_deadband(lx, 0.03f) * static_cast<float>(angle_magnitude);

    // Mecanum mix
    front_left_rpm  = static_cast<int16_t>(ry - rx + lx);
    front_right_rpm = static_cast<int16_t>(ry + rx - lx);
    back_left_rpm   = static_cast<int16_t>(ry + rx + lx);
    back_right_rpm  = static_cast<int16_t>(ry - rx - lx);

    // Set motor RPMs
    manual_control.motor_front_left ->setRpm(front_left_rpm);
    manual_control.motor_front_right->setRpm(front_right_rpm);
    manual_control.motor_back_left  ->setRpm(back_left_rpm);
    manual_control.motor_back_right ->setRpm(back_right_rpm);
}