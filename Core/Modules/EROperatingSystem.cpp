#include "EROperatingSystem.hpp"

// Constructor implementation
ERStatusControl::ERStatusControl(M3508Functions& motor_front_left, M3508Functions& motor_front_right,
                                 M3508Functions& motor_back_left, M3508Functions& motor_back_right, 
                                 float lengthx, float lengthy, GM6020Functions* gm6020_claw) {
    this->manual_control.motor_front_left = &motor_front_left;
    this->manual_control.motor_front_right = &motor_front_right;    
    this->manual_control.motor_back_left = &motor_back_left;
    this->manual_control.motor_back_right = &motor_back_right;
    this->manual_control.gm6020_claw = gm6020_claw;
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
    //traversal(1024, 1807, 1024, 100, 30);
    traversal(1024, 1024, 1024, 100, 30);
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

void ERStatusControl::update(const uartdriver::ReceivedValue& received_data) {
    // Call appropriate mode method based on current state
    switch (current_state) {
        case IDLE:
            idleMode();
            break;
        case MANUAL:
            manualMode(received_data);
            break;
        case GOLD:
            goldMode();
            break;
        case MINING:
            miningMode(received_data);
            break;
        case DEPOSIT:
            // depositMode();
            break;
        case ER_ERROR:
            idleMode();
            break;
    }
}

void ERStatusControl::sendMotorCurrents(int16_t curr_lf, int16_t curr_rf, int16_t curr_lb, int16_t curr_rb) {
    extern FDCAN_HandleTypeDef hfdcan1;
    uint8_t data[8];
    
    // Debug: Store last sent currents AND raw data for inspection
    static volatile int16_t s_last_curr_lf = 0;
    static volatile int16_t s_last_curr_rf = 0;
    static volatile int16_t s_last_curr_lb = 0;
    static volatile int16_t s_last_curr_rb = 0;
    static volatile uint8_t s_last_data[8] = {0};
    s_last_curr_lf = curr_lf;
    s_last_curr_rf = curr_rf;
    s_last_curr_lb = curr_lb;
    s_last_curr_rb = curr_rb;
    
    // Manually pack all 4 motors into the data array
    // Motor 1 (front-left) → bytes 0-1
    data[0] = (curr_lf >> 8) & 0xFF;
    data[1] = curr_lf & 0xFF;
    
    // Motor 2 (front-right) → bytes 2-3
    data[2] = (curr_rf >> 8) & 0xFF;
    data[3] = curr_rf & 0xFF;
    
    // Motor 3 (back-left) → bytes 4-5
    data[4] = (curr_lb >> 8) & 0xFF;
    data[5] = curr_lb & 0xFF;
    
    // Motor 4 (back-right) → bytes 6-7
    data[6] = (curr_rb >> 8) & 0xFF;
    data[7] = curr_rb & 0xFF;
    
    // Store data for debugging
    for (int i = 0; i < 8; i++) {
        s_last_data[i] = data[i];
    }
    
    // Send using pre-initialized TX header (0x200)
    // Check if TX FIFO has space before sending
    uint32_t freeFifoLevel = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
    if (freeFifoLevel > 0) {
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &motor_tx_header, data);
    }
}

// ========== ERClawControl Implementation ==========

ERClawControl::ERClawControl(DMJ4310Functions& base_motor, GM6020Functions& small_motor) {
    claw_motors.base_claw = &base_motor;
    claw_motors.small_claw = &small_motor;
    claw_motors.m3508_claw = nullptr;
    current_state = ClawState::IDLE;
    
    // Initialize TX header for GM6020 claw motors in CURRENT mode (0x2FE for motors 5-7)
    gm6020_tx_header.Identifier = 0x2FE;  // Current mode uses 0x2FE (not 0x2FF)
    gm6020_tx_header.IdType = FDCAN_STANDARD_ID;
    gm6020_tx_header.TxFrameType = FDCAN_DATA_FRAME;
    gm6020_tx_header.DataLength = FDCAN_DLC_BYTES_8;
    gm6020_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    gm6020_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    gm6020_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    gm6020_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    gm6020_tx_header.MessageMarker = 0;
    
    // IMPORTANT: Update the actual GM6020 motor's txHeader to match
    small_motor.setTxHeader(gm6020_tx_header);
    
    // Initialize TX header for M3508 claw motors (0x1FF for M3508 motors 5-8)
    m3508_tx_header.Identifier = 0x1FF;
    m3508_tx_header.IdType = FDCAN_STANDARD_ID;
    m3508_tx_header.TxFrameType = FDCAN_DATA_FRAME;
    m3508_tx_header.DataLength = FDCAN_DLC_BYTES_8;
    m3508_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    m3508_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    m3508_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    m3508_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    m3508_tx_header.MessageMarker = 0;
}

ERClawControl::ERClawControl(DMJ4310Functions& base_motor, GM6020Functions& small_motor, M3508Functions& m3508_motor) {
    claw_motors.base_claw = &base_motor;
    claw_motors.small_claw = &small_motor;
    claw_motors.m3508_claw = &m3508_motor;
    current_state = ClawState::IDLE;
    
    // Initialize TX header for GM6020 claw motors in CURRENT mode (0x2FE for motors 5-7)
    gm6020_tx_header.Identifier = 0x2FE;  // Current mode uses 0x2FE (not 0x2FF)
    gm6020_tx_header.IdType = FDCAN_STANDARD_ID;
    gm6020_tx_header.TxFrameType = FDCAN_DATA_FRAME;
    gm6020_tx_header.DataLength = FDCAN_DLC_BYTES_8;
    gm6020_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    gm6020_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    gm6020_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    gm6020_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    gm6020_tx_header.MessageMarker = 0;
    
    // IMPORTANT: Update the actual GM6020 motor's txHeader to match
    small_motor.setTxHeader(gm6020_tx_header);
    
    // Initialize TX header for M3508 claw motors (0x1FF for M3508 motors 5-8)
    m3508_tx_header.Identifier = 0x1FF;
    m3508_tx_header.IdType = FDCAN_STANDARD_ID;
    m3508_tx_header.TxFrameType = FDCAN_DATA_FRAME;
    m3508_tx_header.DataLength = FDCAN_DLC_BYTES_8;
    m3508_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    m3508_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    m3508_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    m3508_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    m3508_tx_header.MessageMarker = 0;
    
    // IMPORTANT: Update the actual M3508 motor's txHeader to match
    m3508_motor.setTxHeader(m3508_tx_header);
}

void ERClawControl::idleMode() {
    // Stop all claw motors
    if (claw_motors.base_claw) {
        claw_motors.base_claw->sendCurrent(0);
    }
    // Send zero to both GM6020 and M3508 claw motors
    sendClawCurrents(0, 0);
}

void ERClawControl::claspMode() {
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // Reset timing on first call or after long gap
    if (last_time == 0 || (current_time - last_time) > 1000) {
        last_time = current_time;
    }
    
    float dt = (current_time - last_time) / 1000.0f;
    if (dt <= 0 || dt > 0.1f) dt = 0.01f;
    last_time = current_time;
    
    int16_t gm6020_current = 0;
    int16_t m3508_current = 0;
    
    // Debug: store angle info
    static volatile float s_gm6020_current_angle = 0;
    static volatile float s_gm6020_target_angle = 0;
    static volatile int16_t s_gm6020_rpm = 0;
    static volatile int16_t s_gm6020_output = 0;
    
    // DMJ4310 base claw control
    if (claw_motors.base_claw) {
        //claw_motors.base_claw->sendMITCommand(1.0f, 0.0f, 50.0f, 2.0f, 0.0f);
    }
    
    if (claw_motors.small_claw) {
        s_gm6020_current_angle = claw_motors.small_claw->motor_feedback.top_shaft_angle;
        s_gm6020_target_angle = 720.0f;
        s_gm6020_rpm = claw_motors.small_claw->motor_feedback.rpm;
        
        // Use PID to reach 720 degrees (2 full rotations)
        gm6020_current = claw_motors.small_claw->setAnglePID(720.0f, dt, false);
        s_gm6020_output = gm6020_current;
    }
    
    if (claw_motors.m3508_claw) {
        m3508_current = claw_motors.m3508_claw->setAnglePID(720.0f, dt, false);
    }
    
    // Send both motors in one call
    sendClawCurrents(gm6020_current, m3508_current);
}

void ERClawControl::releaseMode() {
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // Reset timing on first call or after long gap
    if (last_time == 0 || (current_time - last_time) > 1000) {
        last_time = current_time;
    }
    
    float dt = (current_time - last_time) / 1000.0f;
    if (dt <= 0 || dt > 0.1f) dt = 0.01f;
    last_time = current_time;
    
    // DMJ4310 base claw control
    if (claw_motors.base_claw) {
        //claw_motors.base_claw->sendMITCommand(-1.0f, 0.0f, 50.0f, 2.0f, 0.0f);
    }
    
    // Calculate PID for both claw motors
    int16_t gm6020_current = 0;
    int16_t m3508_current = 0;
    
    if (claw_motors.small_claw) {
        gm6020_current = claw_motors.small_claw->setAnglePID(0.0f, dt, false);
    }
    
    if (claw_motors.m3508_claw) {
        m3508_current = claw_motors.m3508_claw->setAnglePID(0.0f, dt, false);
    }
    
    // Send both motors in one call
    sendClawCurrents(gm6020_current, m3508_current);
}

void ERClawControl::switchState(const uartdriver::ReceivedValue& received_data) {
    if (received_data.channel_9 < 500) {
        current_state = ClawState::IDLE;
    } else if (received_data.channel_9 == 1024) {
        current_state = ClawState::RELEASE;
    } else {
        current_state = ClawState::CLASP;
    }
}

void ERClawControl::update() {
    // Call appropriate mode method based on current state
    switch (current_state) {
        case ClawState::IDLE:
            idleMode();
            break;
        case ClawState::CLASP:
            claspMode();
            break;
        case ClawState::RELEASE:
            releaseMode();
            break;
        default:
            idleMode();
            break;
    }
}

void ERClawControl::sendClawCurrents(int16_t gm6020_current, int16_t m3508_current) {
    // Debug: Store last sent claw currents
    static volatile int16_t s_last_gm6020_curr = 0;
    static volatile int16_t s_last_m3508_curr = 0;
    s_last_gm6020_curr = gm6020_current;
    s_last_m3508_curr = m3508_current;
    
    // Use motor's own sendCurrent method (handles all the packing internally)
    if (claw_motors.small_claw) {
        claw_motors.small_claw->sendCurrent(gm6020_current);
    }
    
    if (claw_motors.m3508_claw) {
        claw_motors.m3508_claw->sendCurrent(m3508_current);
    }
}