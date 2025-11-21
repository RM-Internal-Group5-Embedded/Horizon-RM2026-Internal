#include "EROperatingSystem.hpp"
#include "DJIMotor.hpp"
#include <cmath>

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

static float computeDeltaTime(uint32_t& last_time_ms) {
    uint32_t current_time = xTaskGetTickCount();

    if (last_time_ms == 0 || (current_time - last_time_ms) > 1000) {
        last_time_ms = current_time;
    }

    float dt = (current_time - last_time_ms) / 1000.0f;
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.01f;
    }
    last_time_ms = current_time;
    return dt;
}

//Plan: use FSM to enable multiple states: |, & 0x0008 

void ERStatusControl::switchState(const uartdriver::ReceivedValue& received_data) {
    // Safety: If channel_10 is low, always go to IDLE (disconnected/safe mode)
    if(received_data.channel_10 <= 1024) {
        // Reset yaw when entering idle mode
        if (current_state != IDLE) {
            resetMPUYaw();
        }
        current_state = IDLE;
        return;
    }
    
    // Active mode: Use channel_9 to select operation mode
    // Use range checks instead of exact equality for robustness
    // Channel 9 ranges:
    //   < 500:    FAST mode
    //   500-900:  GOLD mode (default/center position)
    //   900-1150: MANUAL mode (around 1024 center, with tolerance)
    //   > 1500:   MINING mode
    ERState new_state;
    if(received_data.channel_9 < 500) {
        new_state = FAST;
    } else if(received_data.channel_9 >= 900 && received_data.channel_9 <= 1150) {
        // MANUAL mode: around center (1024) with ±150 tolerance
        new_state = MANUAL;
    } else if(received_data.channel_9 > 1500) {
        new_state = MINING;
    } else {
        // Default to GOLD mode for middle ranges (500-900 or 1150-1500)
        new_state = GOLD;
    }
    
    // Reset yaw when transitioning to IDLE (though this shouldn't happen here)
    // This is handled above for channel_10 safety case
    if (new_state == IDLE && current_state != IDLE) {
        resetMPUYaw();
    }
    
    current_state = new_state;
    gold_Over = true;  // Set flag for gold mode operations
}

void ERStatusControl::goldMode(){
    //traversal(1024, 1807, 1024, 100, 30);
    traversal(1024, 1024, 1024, 100, 30);
}

void ERStatusControl::idleMode() {
    // Send zero current to all 4 motors
    sendMotorCurrents(0, 0, 0, 0);
    
    // Reset PID states in each motor
    //manual_control.motor_front_left->resetData();
    //manual_control.motor_front_right->resetData();
    //manual_control.motor_back_left->resetData();
    //manual_control.motor_back_right->resetData();
    
    // Note: Yaw reset is handled in switchState() when transitioning to IDLE
    // This ensures yaw is reset once when entering idle mode, not every cycle
}

void ERStatusControl::fastMode(const uartdriver::ReceivedValue& received_data) {
    traversal(received_data.channel_1, received_data.channel_2, received_data.channel_4, 1000, 500);
}

void ERStatusControl::manualMode(const uartdriver::ReceivedValue& received_data) {
    traversal(received_data.channel_1, received_data.channel_2, received_data.channel_4, 600, 500);
}

void ERStatusControl::miningMode(const uartdriver::ReceivedValue& received_data) {
    traversal(received_data.channel_1, received_data.channel_2, received_data.channel_4, 300, 100);
    
}

//Relative directional control
void ERStatusControl::traversal( uint16_t joystick_r_x, uint16_t joystick_r_y,
                uint16_t joystick_l_x, uint16_t rpm_magnitude, 
                uint16_t angle_magnitude) {
    
    //float yawDeg = getMPUYaw();
    float yawDeg = 0;
    float yawRad = yawDeg * M_PI / 180;
    //global direction
    //orthogonal basis
    float gx = sin(yawRad);
    float gy = cos(yawRad);

    // Convert joystick values to normalized [-1, 1] range
    float rx = (static_cast<float>(joystick_r_x) - CENTER) / SBUS_SPAN;
    float ry = (static_cast<float>(joystick_r_y) - CENTER) / SBUS_SPAN;
    float lx = (static_cast<float>(joystick_l_x) - CENTER) / SBUS_SPAN;

    // Apply deadband to filter out small inputs
    applyDeadband(rx, 0.03f);
    applyDeadband(ry, 0.03f);
    applyDeadband(lx, 0.03f);
    
    // Apply non-linear curve to RPM axes for smoother control at low speeds
    applyCurveHalfQuad(rx);
    applyCurveHalfQuad(ry);
    
    // Calculate joystick magnitude BEFORE rotation (preserve original input magnitude)
    float controller_magnitude = sqrtf(rx*rx + ry*ry);
    
    // Rotate joystick direction from robot frame to global frame
    // Rotation matrix: [cos(θ) -sin(θ)] [rx]
    //                  [sin(θ)  cos(θ)] [ry]
    // where θ = yaw angle, gx = sin(θ), gy = cos(θ)
    if (controller_magnitude > 0.001f) {
        // Normalize direction vector, rotate, then restore magnitude
        float dir_x = rx / controller_magnitude;
        float dir_y = ry / controller_magnitude;
        
        // Rotate direction vector from robot frame to global frame
        float rotated_x = dir_x*gy - dir_y*gx;  // cos(yaw)*dir_x - sin(yaw)*dir_y
        float rotated_y = dir_x*gx + dir_y*gy;  // sin(yaw)*dir_x + cos(yaw)*dir_y
        
        // Restore original magnitude and scale by rpm_magnitude
        rx = rotated_x * controller_magnitude * static_cast<float>(rpm_magnitude);
        ry = rotated_y * controller_magnitude * static_cast<float>(rpm_magnitude);
    } else {
        // Joystick centered - no movement (avoid division by zero)
        rx = 0.0f;
        ry = 0.0f;
    }
    lx = lx * static_cast<float>(angle_magnitude);

    // Mecanum mix
    front_left_rpm  = static_cast<int16_t>(ry + rx + lx);
    front_right_rpm = -static_cast<int16_t>(ry - rx - lx);
    back_left_rpm   = static_cast<int16_t>(ry - rx + lx);
    back_right_rpm  = -static_cast<int16_t>(ry + rx - lx);

    uint32_t current_time = HAL_GetTick();
    static uint32_t last_time = 0;
    float dt;
    if (last_time == 0 || (current_time - last_time) > 100) {
        // First call or large gap - use default dt
        dt = 0.01f;  // 10ms default (matches task rate)
        last_time = current_time;
    } else {
        dt = (current_time - last_time) / 1000.0f;
        if (dt <= 0.0f || dt > 0.1f) {
            dt = 0.01f;  // Clamp to reasonable range
        }
        last_time = current_time;
    }
    
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
        case GOLD:
            goldMode();
            break;
        case FAST:
            fastMode(received_data);
            break;
        case MANUAL:
            manualMode(received_data);
            break;
        case MINING:
            miningMode(received_data);
            break;
        case DEPOSIT:
            // depositMode();
            break;
    }
}

// Helper function: Apply deadband to a value (modifies by reference)
void ERStatusControl::applyDeadband(float& value, float deadband) {
    if (fabsf(value) < deadband) {
        value = 0.0f;
    }
}

// Helper function: Apply half-quadratic curve to a value (modifies by reference)
void ERStatusControl::applyCurveHalfQuad(float& value) {
    float sign = value >= 0.0f ? 1.0f : -1.0f;
    float a = fabsf(value);
    // curvature: 0.5 * x^2, preserve sign
    value = sign * (0.5f * a * a);
}

void ERStatusControl::sendMotorCurrents(int16_t curr_lf, int16_t curr_rf, int16_t curr_lb, int16_t curr_rb) {
    extern FDCAN_HandleTypeDef hfdcan1;
    uint8_t data[8];
    
    
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
    
    // Send using pre-initialized TX header (0x200)
    // FIX: Don't check FIFO level - hardware handles queuing
    // Silently dropping commands causes delayed response
    // Only check CAN state to avoid errors
    if (hfdcan1.State != HAL_FDCAN_STATE_RESET && hfdcan1.State != HAL_FDCAN_STATE_ERROR) {
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
    // Always keep DMJ4310 alive by issuing a zero command and periodic enable
    if (claw_motors.base_claw) {
        // Send periodic enable command to keep motor enabled (required for feedback)
        static uint32_t last_enable_time = 0;
        uint32_t current_time = HAL_GetTick();
        // Send enable command every 100ms to keep motor enabled
        if (current_time - last_enable_time >= 100) {
            claw_motors.base_claw->enableMotor();
            last_enable_time = current_time;
        }
        // OPTIMIZATION: Non-blocking error recovery using state machine
        // Only attempt recovery once per error, don't block task with HAL_Delay
        static uint32_t last_error_recovery_time = 0;
        static bool recovery_in_progress = false;
        static bool motor_disabled = false;
        
        if (claw_motors.base_claw->hasError()) {
            if (!recovery_in_progress) {
                // Start recovery: disable motor
                claw_motors.base_claw->disableMotor();
                motor_disabled = true;
                recovery_in_progress = true;
                last_error_recovery_time = current_time;
            } else if (motor_disabled && (current_time - last_error_recovery_time >= 5)) {
                // After 5ms delay (non-blocking), re-enable motor
                claw_motors.base_claw->enableMotor();
                motor_disabled = false;
                recovery_in_progress = false;
            }
        } else {
            // No error - reset recovery state
            recovery_in_progress = false;
            motor_disabled = false;
        }
    }
    // Send zero to both GM6020 and M3508 claw motors
    if (claw_motors.small_claw) {
        claw_motors.small_claw->initialized = false;
        claw_motors.small_claw->sendCurrent(0);
    }
    
    if (claw_motors.m3508_claw) {
        claw_motors.m3508_claw->initialized = false;
        claw_motors.m3508_claw->sendCurrent(0);
    }
}

void ERClawControl::fullManualClaw(const uartdriver::ReceivedValue& received_data, bool down, bool clasp){
    resetMoveSequence(release_up);
    resetMoveSequence(release_down);
    resetMoveSequence(get_down);
    static uint32_t last_time = 0;
    float dt = computeDeltaTime(last_time);
    int16_t gm6020_current = 0;
    float gm6020degree = 90.0f;
    if(!clasp) gm6020degree = 180*(static_cast<float>(received_data.channel_6) - CENTER) / SBUS_SPAN; 
    
    float m3508_angle = 360*(static_cast<float>(received_data.channel_3) - CENTER) / SBUS_SPAN;
    
    float motor_position = 4;
    if(!down) motor_position = 8*(static_cast<float>(received_data.channel_5) - CENTER) / SBUS_SPAN;
    
    sendClawCurrent(gm6020degree, motor_position, m3508_angle, dt);
}




void ERClawControl::sendClawCurrent(float gm6020degree, float motor_position, float m3508_angle, float dt){
    // Ensure all three motors send commands at least once each cycle
    if(turn){
    // Motor 1: GM6020 small claw
        if (claw_motors.small_claw) {
            // Use PID to reach target angle - always sends when called with send=true
            claw_motors.small_claw->setAnglePID(gm6020degree, dt, true);
        }
    // Motor 3: M3508 claw
        if (claw_motors.m3508_claw) {
            // Use PID to reach target angle - always sends when called with send=true
            claw_motors.m3508_claw->setAnglePID(m3508_angle, dt, true);
        }
        turn = false;
        return;
    }
    turn = true;;
    // Motor 2: DMJ4310 base claw
    if (claw_motors.base_claw) {
        // Ensure motor is enabled before sending commands
        static uint32_t last_enable_time = 0;
        static uint32_t last_call_time = 0;
        uint32_t current_time = HAL_GetTick();
       
        // Enable immediately on first call after idle (gap > 200ms indicates transition from idle)
        // Or enable periodically (every 100ms) to keep motor enabled
        if (last_call_time == 0 || (current_time - last_call_time > 200) || (current_time - last_enable_time >= 100)) {
            // Clear error codes before enabling to ensure motor can enable
            if (claw_motors.base_claw->hasError()) {
                claw_motors.base_claw->error_code = 0;
                claw_motors.base_claw->rx_motor_id = 0;
            }
            claw_motors.base_claw->enableMotor();
            last_enable_time = current_time;
        }
        last_call_time = current_time;
       
        // MIT command: always sends command each cycle
        claw_motors.base_claw->sendMITCommand(motor_position, 0.01f,
                                claw_motors.base_claw->RPM_KP,
                                claw_motors.base_claw->RPM_KD,
                                claw_motors.base_claw->RPM_KI);
    }
}


void ERClawControl::resetMoveSequence(MoveSequence &target){
    target.active = false;
    target.starttime = 0;
    target.currenttime = 0;
    target.m3508_input = 0;
    target.dmj_input = 0;
    target.gm6020_input = 0;
    for(int i = 0; i < 10; i++){
        target.progress[i] = false;
    }
}


void ERClawControl::captureStartDegrees(MoveSequence &sequence, uint8_t phase_index){
    if(sequence.progress[phase_index] == false){
        sequence.progress[phase_index] = true;
        // Capture actual current position to ensure smooth phase transitions
        // This prevents jumps when transitioning between phases
        if(claw_motors.m3508_claw){
            sequence.m3508_startdeg = claw_motors.m3508_claw->motor_feedback.top_shaft_angle;
        }
        if(claw_motors.small_claw){
            sequence.gm6020_startdeg = claw_motors.small_claw->motor_feedback.top_shaft_angle;
        }
        if(claw_motors.base_claw){
            sequence.dmj_startdeg = claw_motors.base_claw->motor_feedback.top_shaft_angle;
        }
    }
}


float ERClawControl::calculateProgress(uint32_t duration, const MoveSequence& sequence, uint8_t phase_index){
    // Calculate offset by summing all previous phase durations (0 through phase_index-1)
    uint32_t offset = 0;
    for(uint8_t i = 0; i < phase_index; i++){
        offset += sequence.duration[i];
    }
   
    // Get the current phase duration
    uint16_t phase_duration = sequence.duration[phase_index];
    if(phase_duration == 0) return 1.0f; // Avoid division by zero
   
    float progress = static_cast<float>(duration - offset) / static_cast<float>(phase_duration);
    if(progress > 1.0f) progress = 1.0f;
    if(progress < 0.0f) progress = 0.0f;
    return progress;
}


uint32_t ERClawControl::calculateTotalDuration(const MoveSequence& sequence, uint8_t phase_index){
    // Sum durations from [0] to [phase_index] (inclusive)
    uint32_t total = 0;
    for(uint8_t i = 0; i <= phase_index; i++){
        total += sequence.duration[i];
    }
    return total;
}


void ERClawControl::moveMotor_gm6020(   float starting_degree,
                                        float target, float progress, float dt){
    // Use captured start position for consistent interpolation
    // The start position is captured once at the beginning of each phase
    if(turn) return;
    turn = false;
    float starting = (static_cast<float> (starting_degree) / 180.0f) * SBUS_SPAN + CENTER;
    float gm6020degree =
    180*(static_cast<float>(starting + (target - starting)*progress) - CENTER) / SBUS_SPAN;
    //+ 180*(static_cast<float>(received_data_.channel_6) - CENTER) / SBUS_SPAN;
   
    if (claw_motors.small_claw) {
        claw_motors.small_claw->setAnglePID(gm6020degree, dt, true);
    }
}


void ERClawControl::moveMotor_m3508(float starting_degree,
                                    float target, float progress, float dt){
    if(turn) return;
    turn = false;
    float starting = (static_cast<float> (starting_degree) / 360.0f) * SBUS_SPAN + CENTER;
    float m3508_angle =
    360*(static_cast<float>(starting + (target - starting)*progress) - CENTER) / SBUS_SPAN;
    //+ 360*(static_cast<float>(received_data_.channel_3) - CENTER) / SBUS_SPAN;
   


    if (claw_motors.m3508_claw) {
        claw_motors.m3508_claw->setAnglePID(m3508_angle, dt, true);
    }
}


void ERClawControl::moveMotor_dmj(  float starting_degree,
                                    float target, float progress, float dt){
    if(!turn) return;
    turn = true;
    float starting = (static_cast<float> (starting_degree) / 80.0f) * SBUS_SPAN + CENTER;
    float motor_position =
    8*(static_cast<float>(starting + (target - starting)*progress) - CENTER) / SBUS_SPAN;
    //+ 8*(static_cast<float>(received_data_.channel_5) - CENTER) / SBUS_SPAN;
     
    if (claw_motors.base_claw) {
        // Ensure motor is enabled before sending commands
        static uint32_t last_enable_time = 0;
        static uint32_t last_call_time = 0;
        uint32_t current_time = HAL_GetTick();
       
        // Enable immediately on first call after idle (gap > 200ms indicates transition from idle)
        // Or enable periodically (every 100ms) to keep motor enabled
        if (last_call_time == 0 || (current_time - last_call_time > 200) || (current_time - last_enable_time >= 100)) {
            // Clear error codes before enabling to ensure motor can enable
            if (claw_motors.base_claw->hasError()) {
                claw_motors.base_claw->error_code = 0;
                claw_motors.base_claw->rx_motor_id = 0;
            }
            claw_motors.base_claw->enableMotor();
            last_enable_time = current_time;
        }
        last_call_time = current_time;
       
        // MIT command: position=12.566 rad, velocity=0, kp=50, kd=2, torque=0
        claw_motors.base_claw->sendMITCommand(motor_position, 0.01f,
                                claw_motors.base_claw->RPM_KP,
                                claw_motors.base_claw->RPM_KD,
                                claw_motors.base_claw->RPM_KI);
    }
}


void ERClawControl::holdMotor_gm6020(float target, float dt){
    if(!turn) return;
    turn = false;
    float gm6020degree =
    180*(static_cast<float>(target - CENTER)) / SBUS_SPAN;
    //+ 180*(static_cast<float>(received_data_.channel_6) - CENTER) / SBUS_SPAN;
   
    if (claw_motors.small_claw) {
        claw_motors.small_claw->setAnglePID(gm6020degree, dt, true);
    }
}
void ERClawControl::holdMotor_m3508(float target, float dt){
    if(!turn) return;
    turn = false;
    float m3508_angle =
    360*(static_cast<float>(target - CENTER)) / SBUS_SPAN;
    //+ 360*(static_cast<float>(received_data_.channel_3) - CENTER) / SBUS_SPAN;
   
    if (claw_motors.m3508_claw) {
        claw_motors.m3508_claw->setAnglePID(m3508_angle, dt, true);
    }
}
void ERClawControl::holdMotor_dmj(float target, float dt){
    if(turn) return;
    turn = true;
    float motor_position =
    8*(static_cast<float>(target) - CENTER) / SBUS_SPAN;
    //+ 8*(static_cast<float>(received_data_.channel_5) - CENTER) / SBUS_SPAN;
   
    if (claw_motors.base_claw) {
        // Ensure motor is enabled before sending commands
        static uint32_t last_enable_time = 0;
        static uint32_t last_call_time = 0;
        uint32_t current_time = HAL_GetTick();
       
        // Enable immediately on first call after idle (gap > 200ms indicates transition from idle)
        // Or enable periodically (every 100ms) to keep motor enabled
        if (last_call_time == 0 || (current_time - last_call_time > 200) || (current_time - last_enable_time >= 100)) {
            // Clear error codes before enabling to ensure motor can enable
            if (claw_motors.base_claw->hasError()) {
                claw_motors.base_claw->error_code = 0;
                claw_motors.base_claw->rx_motor_id = 0;
            }
            claw_motors.base_claw->enableMotor();
            last_enable_time = current_time;
        }
        last_call_time = current_time;
       
        // MIT command: position=12.566 rad, velocity=0, kp=50, kd=2, torque=0
        claw_motors.base_claw->sendMITCommand(motor_position, 0.01f,
                                claw_motors.base_claw->RPM_KP,
                                claw_motors.base_claw->RPM_KD,
                                claw_motors.base_claw->RPM_KI);
    }
}



void ERClawControl::getDownSequence(){
    static uint32_t last_time = 0;
    float dt = computeDeltaTime(last_time);
    float progress = 0;


    if(!get_down.active){
        // Reset other sequences when starting this one
        resetMoveSequence(release_up);
        resetMoveSequence(release_down);
        resetMoveSequence(get_down);
        // Initialize this sequence
        get_down.active = true;
        get_down.starttime = xTaskGetTickCount();
    }
    // Update current time every cycle (always update, regardless of active state)
    get_down.currenttime = xTaskGetTickCount();
    // FIX: Use uint32_t to prevent overflow (uint16_t overflows after ~65 seconds)
    uint32_t duration = get_down.currenttime - get_down.starttime;
   
    // Sequence:
    // Phase 0 extend all
    // Phase 1 lower down
    // Phase 2 clasp
   
    if(duration < calculateTotalDuration(get_down, 0)){ // arm all goes up, claw open
        progress = calculateProgress(duration, get_down, 0);
       
        captureStartDegrees(get_down, 0);


        moveMotor_m3508(get_down.m3508_startdeg, 1500, progress, dt);
        moveMotor_gm6020(get_down.gm6020_startdeg, 800, progress, dt);
        moveMotor_dmj(get_down.dmj_startdeg, 1000, progress, dt);
    } else if(duration < calculateTotalDuration(get_down, 1)){ //arm goes down
        progress = calculateProgress(duration, get_down, 1);


        captureStartDegrees(get_down, 1);


        holdMotor_m3508(1500, dt);
        holdMotor_gm6020(800, dt);
        moveMotor_dmj(get_down.dmj_startdeg, 1500, progress, dt);
    } else if(duration < calculateTotalDuration(get_down, 2)){ //arm down sweep back
        progress = calculateProgress(duration, get_down, 2);


        captureStartDegrees(get_down, 2);


        moveMotor_m3508(get_down.m3508_startdeg, 850, progress, dt);
        holdMotor_gm6020(450, dt);
        holdMotor_dmj(1500, dt);
    } else if(duration < calculateTotalDuration(get_down, 3)){ //claw clench
        progress = calculateProgress(duration, get_down, 3);


        captureStartDegrees(get_down, 3);


        moveMotor_m3508(get_down.m3508_startdeg, 1024, progress, dt);
        moveMotor_gm6020(get_down.gm6020_startdeg, 1500, progress, dt);
        holdMotor_dmj(1500, dt);
    } else if(duration < calculateTotalDuration(get_down, 4)){ //clasp safety
        holdMotor_m3508(1024, dt);
        holdMotor_gm6020(1500, dt);
        holdMotor_dmj(1500, dt);
    } else if(duration < calculateTotalDuration(get_down, 5)){ //arm move up
        progress = calculateProgress(duration, get_down, 5);


        captureStartDegrees(get_down, 5);


        holdMotor_m3508(1024, dt);
        holdMotor_gm6020(1500, dt);
        moveMotor_dmj(get_down.dmj_startdeg, 1024, progress, dt);
    } else { //final state: holding object up
        progress = 1.0f;
        // Final phase - hold position
        holdMotor_m3508(1024, dt);
        holdMotor_gm6020(1500, dt);
        holdMotor_dmj(1024, dt);
    }
}  


void ERClawControl::releaseDownSequence(){
    static uint32_t last_time = 0;
    float dt = computeDeltaTime(last_time);
    float progress = 0;


    if(!release_down.active){
        // Reset other sequences when starting this one
        resetMoveSequence(release_up);
        resetMoveSequence(release_down);
        resetMoveSequence(get_down);
        // Initialize this sequence
        release_down.active = true;
        release_down.starttime = xTaskGetTickCount();
    }
    // Update current time every cycle (always update, regardless of active state)
    release_down.currenttime = xTaskGetTickCount();
    // FIX: Use uint32_t to prevent overflow (uint16_t overflows after ~65 seconds)
    uint32_t duration = release_down.currenttime - release_down.starttime;
   
    if(duration < calculateTotalDuration(release_down, 0)){ //clasped arm goes up
        progress = calculateProgress(duration, release_down, 0);
       
        captureStartDegrees(release_down, 0);


        moveMotor_m3508(release_down.m3508_startdeg, 1800, progress, dt);
        holdMotor_gm6020(1500, dt);
        moveMotor_dmj(release_down.dmj_startdeg, 770, progress, dt);
    } else if(duration < calculateTotalDuration(release_down, 1)){ //clasped arm goes down
        progress = calculateProgress(duration, release_down, 1);


        captureStartDegrees(release_down, 1);


        moveMotor_m3508(release_down.m3508_startdeg, 1800, progress, dt);
        holdMotor_gm6020(1500, dt);
        moveMotor_dmj(release_down.dmj_startdeg, 1800, progress, dt);
    } else if(duration < calculateTotalDuration(release_down, 2)){ //clasped arm release
        progress = calculateProgress(duration, release_down, 2);


        captureStartDegrees(release_down, 2);


        holdMotor_m3508(1800, dt);
        moveMotor_gm6020(release_down.gm6020_startdeg, 450, progress, dt);
        holdMotor_dmj(1400, dt);
    } else if(duration < calculateTotalDuration(release_down, 3)){ //clasped arm moves back up
        progress = calculateProgress(duration, release_down, 3);


        captureStartDegrees(release_down, 3);


        holdMotor_m3508(1800, dt);
        holdMotor_gm6020(450, dt);
        moveMotor_dmj(release_down.dmj_startdeg, 770, progress, dt);
    } else if(duration < calculateTotalDuration(release_down, 4)){ //move back to relax
        progress = calculateProgress(duration, release_down, 4);


        captureStartDegrees(release_down, 4);


        moveMotor_m3508(release_down.m3508_startdeg, 1024, progress, dt);
        holdMotor_gm6020(450, dt);
        holdMotor_dmj(770, dt);
    } else { //arm is idle
        holdMotor_m3508(1024, dt);
        holdMotor_gm6020(450, dt);
        holdMotor_dmj(770, dt);
    }
}


void ERClawControl::releaseUpSequence(){
    static uint32_t last_time = 0;
    float dt = computeDeltaTime(last_time);
    float progress = 0;


    if(!release_up.active){
        // Reset other sequences when starting this one
        resetMoveSequence(release_up);
        resetMoveSequence(release_down);
        resetMoveSequence(get_down);
        // Initialize this sequence
        release_up.active = true;
        release_up.starttime = xTaskGetTickCount();
    }
    // Update current time every cycle (always update, regardless of active state)
    release_up.currenttime = xTaskGetTickCount();
    // FIX: Use uint32_t to prevent overflow (uint16_t overflows after ~65 seconds)
    uint32_t duration = release_up.currenttime - release_up.starttime;
   
    if(duration < calculateTotalDuration(release_up, 0)){ //clasped arm goes up
        progress = calculateProgress(duration, release_up, 0);
       
        captureStartDegrees(release_up, 0);


        moveMotor_m3508(release_up.m3508_startdeg, 1800, progress, dt);
        holdMotor_gm6020(1500, dt);
        moveMotor_dmj(release_up.dmj_startdeg, 700, progress, dt);
    } else if(duration < calculateTotalDuration(release_up, 1)){ //clasped arm waits for car to push
        progress = calculateProgress(duration, release_up, 1);


        captureStartDegrees(release_up, 1);


        holdMotor_m3508(1800, dt);
        holdMotor_gm6020(1500, dt);
        holdMotor_dmj(750, dt);
    } else if(duration < calculateTotalDuration(release_up, 2)){ //clasped arm release
        progress = calculateProgress(duration, release_up, 2);


        captureStartDegrees(release_up, 2);


        holdMotor_m3508(1800, dt);
        moveMotor_gm6020(release_up.gm6020_startdeg, 450, progress, dt);
        holdMotor_dmj(750, dt);
    } else if(duration < calculateTotalDuration(release_up, 3)){ //clasped arm stalls at up
        progress = 1;


        captureStartDegrees(release_up, 3);


        holdMotor_m3508(1800, dt);
        holdMotor_gm6020(450, dt);
        holdMotor_dmj(770, dt);
    } else if(duration < calculateTotalDuration(release_up, 4)){ //relax
        progress = calculateProgress(duration, release_up, 4);


        captureStartDegrees(release_up, 4);


        moveMotor_m3508(release_up.m3508_startdeg, 1024, progress, dt);
        holdMotor_gm6020(450, dt);
        holdMotor_dmj(770, dt);
    } else { //arm is idle
        holdMotor_m3508(1024, dt);
        holdMotor_gm6020(450, dt);
        holdMotor_dmj(770, dt);
    }
}





void ERClawControl::claspMode(const uartdriver::ReceivedValue& received_data) {
    static uint32_t last_time = 0;
    float dt = computeDeltaTime(last_time);
    int16_t gm6020_current = 0;
    float gm6020degree = 180*(static_cast<float>(received_data.channel_6) - CENTER) / SBUS_SPAN;
    if (claw_motors.small_claw) {
        // Use PID to reach 720 degrees (2 full rotations)
        gm6020_current = claw_motors.small_claw->setAnglePID(gm6020degree, dt, true);
    }
}

void ERClawControl::releaseMode(const uartdriver::ReceivedValue& received_data) {
    static uint32_t last_time = 0;
    float dt = computeDeltaTime(last_time);
    int16_t gm6020_current = 0;
    float gm6020degree = -70.0f;
    if (claw_motors.small_claw) {
        // Use PID to reach 720 degrees (2 full rotations)
        gm6020_current = claw_motors.small_claw->setAnglePID(gm6020degree, dt, true);
    }
}

void ERClawControl::upMode(const uartdriver::ReceivedValue& received_data) {

    static uint32_t last_time = 0;
    float dt = computeDeltaTime(last_time);
    
    float m3508_angle = 0;
    m3508_angle = 360*(static_cast<float>(received_data.channel_3) - CENTER) / SBUS_SPAN;
    // float motor_position = 0.0f;  // Return to zero
    // motor_position = 8*(static_cast<float>(received_data.channel_5) - CENTER) / SBUS_SPAN;

    // DMJ4310 base claw control - RELEASE MODE: go to 0° (top shaft angle)
    if (claw_motors.base_claw) {
        // Ensure motor is enabled before sending commands
        static uint32_t last_enable_time = 0;
        uint32_t current_time = HAL_GetTick();
        // Send enable command periodically (every 100ms) to keep motor enabled
        if (current_time - last_enable_time >= 100) {
            claw_motors.base_claw->enableMotor();
            last_enable_time = current_time;
        }
        // claw_motors.base_claw->sendMITCommand(motor_position, 0.01f, 
        //                         claw_motors.base_claw->RPM_KP, 
        //                         claw_motors.base_claw->RPM_KD, 
        //                         claw_motors.base_claw->RPM_KI);
        claw_motors.base_claw->sendMITCommand(0, 0.01f, 
                                0, 
                                0, 
                                0);
    }

    if (claw_motors.m3508_claw) {
        m3508_angle = claw_motors.m3508_claw->setAnglePID(m3508_angle, dt, true);
    }
}

void ERClawControl::downMode(const uartdriver::ReceivedValue& received_data) {
    static uint32_t last_time = 0;
    float dt = computeDeltaTime(last_time);
    float m3508_angle = 0;
    m3508_angle = 360*(static_cast<float>(received_data.channel_3) - CENTER) / SBUS_SPAN;
    float motor_position = 0.0f; 
    motor_position = 8*(static_cast<float>(received_data.channel_5) - CENTER) / SBUS_SPAN;

    // DMJ4310 base claw control - RELEASE MODE: go to 0° (top shaft angle)
    if (claw_motors.base_claw) {
        // Ensure motor is enabled before sending commands
        static uint32_t last_enable_time = 0;
        uint32_t current_time = HAL_GetTick();
        // Send enable command periodically (every 100ms) to keep motor enabled
        if (current_time - last_enable_time >= 100) {
            claw_motors.base_claw->enableMotor();
            last_enable_time = current_time;
        }
        // MIT command: position=12.566 rad, velocity=0, kp=50, kd=2, torque=0
        claw_motors.base_claw->sendMITCommand(motor_position, 0.01f, 
                                claw_motors.base_claw->RPM_KP, 
                                claw_motors.base_claw->RPM_KD, 
                                claw_motors.base_claw->RPM_KI);
    }

    if (claw_motors.m3508_claw) {
        m3508_angle = claw_motors.m3508_claw->setAnglePID(m3508_angle, dt, true);
    }
}

void ERClawControl::switchState(const uartdriver::ReceivedValue& received_data) {
    // Safety: channel_10 is used as enable/disable switch
    // channel_10 = 240: UART disconnected (heartbeat mode) -> IDLE
    // channel_10 <= 1024: Safety switch OFF (center or below) -> IDLE  
    // channel_10 > 1024: Safety switch ON (upper half) -> Active mode
    // Note: SBUS range is 240-1807, center is 1024
    
    // Store previous state to detect transition from/to IDLE
    ClawState previous_state = current_state;
    
    if(received_data.channel_10 <= 1024) {
        current_state = ClawState::IDLE;
    } else {
        if((received_data.channel_7 < 500) && (received_data.channel_8 < 500)){
            current_state = ClawState::MANUAL;
        } 
        if((received_data.channel_7 > 1500) && (received_data.channel_8 > 1500)){
            current_state = ClawState::CLASPDOWN;
        } 
        if((received_data.channel_7 < 500) && (received_data.channel_8 > 1500)){
            current_state = ClawState::DOWN;
        } 
        if((received_data.channel_7 > 1500) && (received_data.channel_8 < 500)){
            current_state = ClawState::CLASP;
        } 
        
        // channel_10 > 1024: Active mode - process commands
        // if (received_data.channel_7 < 500) {
        //     current_state = ClawState::CLASP;
        // } else {
        //     current_state = ClawState::RELEASE;
        // }

        // if (received_data.channel_8 < 500) {
        //     arm_state = ArmState::DOWN;
        // } else {
        //     arm_state = ArmState::UP;
        // }
    }
    
    // Enable DMJ4310 motor when transitioning from IDLE to any active state
    if (previous_state == ClawState::IDLE && current_state != ClawState::IDLE) {
        if (claw_motors.base_claw) {
            claw_motors.base_claw->enableMotor();
        }
    }
}

//extern uint16_t Modules::DJIMotors::counter;
void ERClawControl::update(const uartdriver::ReceivedValue& received_data) {
    // Call appropriate mode method based on current state
    switch (current_state) {
        case ClawState::IDLE:
            idleMode();
            break;
        case ClawState::CLASP:
            fullManualClaw(received_data, false, true);
            break;
        case ClawState::DOWN:
            fullManualClaw(received_data, true, false);
            break;
        case ClawState::CLASPDOWN:
            fullManualClaw(received_data, true, true);
            break;
        case ClawState::RELEASE:
            releaseMode(received_data);
            break;
        case ClawState::GETDOWN:
            // claw_motors.base_claw->RPM_KP = 5.0f;
            // claw_motors.base_claw->RPM_KI = 0.0f;
            // claw_motors.base_claw->RPM_KP = 2.0f;
            getDownSequence();
            break;
        case ClawState::MANUAL:
            // claw_motors.base_claw->RPM_KP = 5.0f;
            // claw_motors.base_claw->RPM_KI = 0.0f;
            // claw_motors.base_claw->RPM_KP = 0.0f;
            fullManualClaw(received_data, false, false);
            break;
        case ClawState::RELEASEDOWN:
            // claw_motors.base_claw->RPM_KP = 5.0f;
            // claw_motors.base_claw->RPM_KI = 0.0f;
            // claw_motors.base_claw->RPM_KP = 0.0f;
            releaseDownSequence();
            break;
        case ClawState::RELEASEUP:
            // claw_motors.base_claw->RPM_KP = 5.0f;
            // claw_motors.base_claw->RPM_KI = 0.0f;
            // claw_motors.base_claw->RPM_KP = 2.0f;
            releaseUpSequence();
            break;
        default:
            idleMode();
            break;

    }
    // if(current_state!= ClawState::IDLE)
    // switch (arm_state) {
    //     case ArmState::UP:
    //         //claw_motors.base_claw->enableMotor();
    //         upMode(received_data);
    //         break;
    //     case ArmState::DOWN:
    //         //claw_motors.base_claw->enableMotor();
    //         downMode(received_data);
    //         break;
    // }
    // else {
        
    // }
    //counter = 0;
}

// void ERClawControl::sendClawCurrents(int16_t gm6020_current, int16_t m3508_current) {
    
//     // Use motor's own sendCurrent method (handles all the packing internally)
//     if (claw_motors.small_claw) {
//         claw_motors.small_claw->sendCurrent(gm6020_current);
//     }
    
//     if (claw_motors.m3508_claw) {
//         claw_motors.m3508_claw->sendCurrent(m3508_current);
//     }
// }