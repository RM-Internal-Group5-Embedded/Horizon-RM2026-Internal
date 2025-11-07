#include "MotorFunctions.hpp"


// Base MotorFunctions constructor implementation
MotorFunctions::MotorFunctions(int id, pFDCAN_RxFifo0CallbackTypeDef callback,
                               pFDCAN_ErrorStatusCallbackTypeDef errorCallback) {
    motor_id = id;
    RPM_KP = 100.0f;
    RPM_KI = 0.1f;
    RPM_KD = 10.0f;
    MAX_CURRENT = 5000; 

    // choose filter/tx header based on id range
    filter = Modules::DJIMotors::getFilter(0x201, 0x204);
    txHeader = Modules::DJIMotors::getTxHeader(id, Modules::DJIMotors::MotorType::M3508);
    

    // Initialize timing/data
    last_time = current_time = HAL_GetTick();
    resetData();

    // Note: Filter/callbacks now registered once in canbridge::init
}

MG90SFunctions::MG90SFunctions(int id, pFDCAN_RxFifo0CallbackTypeDef callback, 
                            pFDCAN_ErrorStatusCallbackTypeDef errorCallback,
                            uint16_t filterID2, uint16_t filterID1)
                            : MotorFunctions(id, callback, errorCallback) {
    // Small hobby servo; keep defaults but allow CAN filter if used
    filter = Modules::DJIMotors::getFilter(filterID2, filterID1);
    txHeader = Modules::DJIMotors::getTxHeader(id, Modules::DJIMotors::MotorType::M3508);
    RPM_KP = 20.0f; 
    RPM_KI = 0.05f; 
    RPM_KD = 0.5f; 
    MAX_CURRENT = 5000;
}

JGA25370Functions::JGA25370Functions(int id, pFDCAN_RxFifo0CallbackTypeDef callback, 
                            pFDCAN_ErrorStatusCallbackTypeDef errorCallback,
                            uint16_t filterID2, uint16_t filterID1)
                            : MotorFunctions(id, callback, errorCallback) {
    // Larger hobby motor; tune defaults accordingly
    filter = Modules::DJIMotors::getFilter(filterID2, filterID1);
    txHeader = Modules::DJIMotors::getTxHeader(id, Modules::DJIMotors::MotorType::M3508);
    RPM_KP = 50.0f; RPM_KI = 0.1f; RPM_KD = 1.0f; MAX_CURRENT = 12000;
}



M3508Functions::M3508Functions(int id,  pFDCAN_RxFifo0CallbackTypeDef callback, 
                                        pFDCAN_ErrorStatusCallbackTypeDef errorCallback,
                                        uint16_t filterID2, uint16_t filterID1)
                                        : MotorFunctions(id, callback, errorCallback) {
    filter = Modules::DJIMotors::getFilter(filterID2, filterID1); 
    txHeader = Modules::DJIMotors::getTxHeader(id, Modules::DJIMotors::MotorType::M3508);
            // Default PID values for M3508
    RPM_KP = 80.0f;
    RPM_KI = 0.1f;
    RPM_KD = 2.0f;
    MAX_CURRENT = 20000;

    if(id>4){
        RPM_KP = 20.0f;
        RPM_KI = 0.1f;
        RPM_KD = 2.0f;
        MAX_CURRENT = 4000;
    }
}

// Provide a base virtual method implementation so the vtable is emitted
void MotorFunctions::readMotorFeedback(uint8_t rxData[8]) {
    // Default generic parsing: angle, rpm, current, temperature
    int16_t current_angle = (rxData[0] << 8) | rxData[1];
    motor_feedback.angle = current_angle;
    motor_feedback.rpm = (rxData[2] << 8) | rxData[3];
    motor_feedback.current = (rxData[4] << 8) | rxData[5];
    motor_feedback.temperature = rxData[6];
    motor_feedback.last_update = HAL_GetTick();
}

void MotorFunctions::resetData() {
    pid_integral = 0;
    pid_prev_error = 0;
    pid_error = 0;
    last_time = current_time = HAL_GetTick();
    prev_angle = 0;
    total_motor_counts = 0;
    motor_feedback.target_angle = 0;
    motor_feedback.target_rpm = 0;
}

void M3508Functions::readMotorFeedback(uint8_t rxData[8]) {
    int16_t current_angle = (rxData[0] << 8) | rxData[1];
    int32_t current = HAL_GetTick();
    motor_feedback.frequency = 
        1000.0f / (current - motor_feedback.last_update);
    // Handle wrap-around to track total motor rotation
    if(!initialized){
        initialized = true;
        prev_angle = current_angle;
    }
    int16_t angle_diff = current_angle - prev_angle;
    if (angle_diff < -4000) {
        // Positive wrap: 8191 → 0
        total_motor_counts += 8191 + angle_diff;
    } else if (angle_diff > 4000) {
        // Negative wrap: 0 → 8191  
        total_motor_counts += angle_diff - 8191;
    } else {
        total_motor_counts += angle_diff;
    }
    prev_angle = current_angle;
    // Store data
    motor_feedback.angle = current_angle;
    
    motor_feedback.bottom_rpm = ((rxData[2] << 8) | rxData[3]);
    motor_feedback.rpm = (187.0f / 3591.0f) * motor_feedback.bottom_rpm;
    motor_feedback.current = (rxData[4] << 8) | rxData[5];
    motor_feedback.temperature = rxData[6];
    motor_feedback.last_update = current;

    // Calculate top shaft angle (0-360°)
    // Total motor revolutions = total_motor_counts / 8191.0f
    // Top shaft revolutions = motor_revs × (187/3591)
    float total_motor_revs = (float)total_motor_counts / 8191.0f;
    float top_shaft_revs = total_motor_revs * (187.0f / 3591.0f);
    
    motor_feedback.top_shaft_angle = top_shaft_revs*360;
}

void MotorFunctions::sendCurrent(int16_t current) {
    // Debug: Store last sent data for inspection
    static volatile int16_t s_last_motor_id = 0;
    static volatile int16_t s_last_current = 0;
    static volatile uint8_t s_last_tx_data[8] = {0};
    
    s_last_motor_id = motor_id;
    s_last_current = current;
    
    // Construct TX data and send directly
    Modules::DJIMotors::constructTxData(data, motor_id, current);
    
    // Store data for debugging
    for (int i = 0; i < 8; i++) {
        s_last_tx_data[i] = data[i];
    }
    
    Modules::DJIMotors::send(&txHeader, data);
}

void MotorFunctions::controlLoop(int choice, uint16_t magnitude) {
    current_time = HAL_GetTick();
    float dt = (current_time - last_time) / 1000.0f; // Update dt
    if (dt <= 0 || dt > 0.1f) dt = 0.01f; // Clamp
    
    setRpmPID(1000, dt, true);  // Calculate and send
    last_time = current_time;
}

void MotorFunctions::stopLoop(){
    Modules::DJIMotors::constructTxData(data, motor_id, 0);
    Modules::DJIMotors::send(&txHeader, data);
    Modules::PID::resetPIDState(pid_integral, pid_prev_error);
}


void MotorFunctions::setRpm(int16_t target_rpm){
    current_time = HAL_GetTick();
    float dt = (current_time - last_time) / 1000.0f; // Update dt
    if (dt <= 0 || dt > 0.1f) dt = 0.01f; // Clamp

    setRpmPID(target_rpm, dt, true);  // Calculate and send
    last_time = current_time;
}

int16_t MotorFunctions::setRpmPID(int16_t target_rpm, float dt, bool send){
    motor_feedback.target_rpm = target_rpm;
    
    // Read motor RPM atomically (motor_feedback.rpm is written by ISR)
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    int16_t current_rpm = motor_feedback.rpm;
    __set_PRIMASK(primask);
    
    // Calculate PID
    pid_output = Modules::PID::calculate(
        (float)motor_feedback.target_rpm,
        (float)current_rpm,
        RPM_KP, RPM_KI, RPM_KD,
        pid_integral,
        pid_prev_error,
        dt
    );
    pid_output = Modules::PID::pidClampMinMax(pid_output, MAX_CURRENT);
    motor_feedback.output_current = (int16_t)pid_output;
    
    if (send) {
        sendCurrent(motor_feedback.output_current);
    }
    
    return motor_feedback.output_current;
}

void MotorFunctions::setAngle(float target_angle){
    current_time = HAL_GetTick();
    float dt = (current_time - last_time) / 1000.0f; // Update dt
    if (dt <= 0 || dt > 0.1f) dt = 0.01f; // Clamp

    setAnglePID(target_angle, dt, true);  // Calculate and send
    last_time = current_time;
}

int16_t MotorFunctions::setAnglePID(float target_angle, float dt, bool send){
    motor_feedback.target_angle = target_angle;
    
    // Read motor angle atomically (motor_feedback.top_shaft_angle is written by ISR)
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    float current_angle = motor_feedback.top_shaft_angle;
    __set_PRIMASK(primask);
    
    // Calculate PID
    pid_output = Modules::PID::calculate(
        (float)motor_feedback.target_angle,
        current_angle,
        RPM_KP, RPM_KI, RPM_KD,
        pid_integral,
        pid_prev_error,
        dt
    );
    pid_output = Modules::PID::pidClampMinMax(pid_output, MAX_CURRENT);
    motor_feedback.output_current = (int16_t)pid_output;
    
    if (send) {
        sendCurrent(motor_feedback.output_current);
    }
    
    return motor_feedback.output_current;
}

// Pack this motor's current into the correct position in the 8-byte array
void MotorFunctions::packCurrentIntoData(uint8_t* data, int16_t current) {
    // Motor IDs 1-4 map to byte offsets 0,2,4,6 (for 0x200 or 0x1FF)
    // Motor IDs 5-8 also map to byte offsets 0,2,4,6 (for 0x1FF or 0x2FF)
    int offset;
    if (motor_id >= 1 && motor_id <= 4) {
        offset = (motor_id - 1) * 2;  // IDs 1-4 → offsets 0,2,4,6
    } else if (motor_id >= 5 && motor_id <= 8) {
        offset = (motor_id - 5) * 2;  // IDs 5-8 → offsets 0,2,4,6
    } else {
        return;  // Invalid motor ID
    }
    
    data[offset] = (current >> 8) & 0xFF;
    data[offset + 1] = current & 0xFF;
}

// ========== DMJ4310 Implementation ==========
DMJ4310Functions::DMJ4310Functions(int id, pFDCAN_RxFifo0CallbackTypeDef callback,
                                   pFDCAN_ErrorStatusCallbackTypeDef errorCallback,
                                   uint16_t filterID2, uint16_t filterID1)
                                   : MotorFunctions(id, callback, errorCallback) {
    // DMJ4310 MIT Mode: TX = motor's CAN_ID, RX = Master ID (default 0)
    // The motor's CAN_ID must be set via debug assistant
    filter = Modules::DJIMotors::getFilter(filterID2, filterID1);
    txHeader = Modules::DJIMotors::getTxHeader(id, Modules::DJIMotors::MotorType::DMJ4310);
    
    // DMJ4310-specific PID tuning
    RPM_KP = 25.0f;    // Moderate P gain
    RPM_KI = 0.05f;    // Low integral to prevent windup
    RPM_KD = 1.0f;     // Moderate derivative
    MAX_CURRENT = 10000;  // Conservative current limit
}

void DMJ4310Functions::readMotorFeedback(uint8_t rxData[8]) {
    // DMJ4310 MIT protocol feedback format (from manual):
    // D[0]: MST_ID (Master ID)
    // D[1]: ID | ERR << 4 (Motor ID[3:0] | Error[7:4])
    // D[2]: POS[15:8] (Position high byte)
    // D[3]: POS[7:0] (Position low byte)
    // D[4]: VEL[11:4] (Velocity high 8 bits)
    // D[5]: VEL[3:0] | T[11:8] (Velocity low 4 bits + Torque high 4 bits)
    // D[6]: T[7:0] (Torque low byte)
    // D[7]: T_MOS or T_Rotor (Temperature in °C)
    
    uint32_t current = HAL_GetTick();
    motor_feedback.frequency = 
        1000.0f / (current - motor_feedback.last_update);
    
    // Extract error code from D[1]
    uint8_t rx_motor_id = rxData[1] & 0x0F;
    uint8_t rx_error_code = (rxData[1] >> 4) & 0x0F;
    
    // Decode position (16 bits) from D[2:3]
    uint16_t p_int = (rxData[2] << 8) | rxData[3];
    float position = uintToFloat(p_int, p_min, p_max, 16);
    
    // Decode velocity (12 bits) from D[4:5]
    uint16_t v_int = (rxData[4] << 4) | ((rxData[5] >> 4) & 0x0F);
    float velocity = uintToFloat(v_int, v_min, v_max, 12);
    
    // Decode torque (12 bits) from D[5:6]
    uint16_t t_int = ((rxData[5] & 0x0F) << 8) | rxData[6];
    float torque = uintToFloat(t_int, t_min, t_max, 12);
    
    // Temperature from D[7]
    uint8_t temperature = rxData[7];
    
    // Convert to motor_feedback structure
    // Position in radians -> convert to encoder counts (0-8191 range)
    motor_feedback.angle = (int16_t)((position / (2.0f * 3.14159f)) * 8191.0f);
    
    // Velocity in rad/s -> RPM
    motor_feedback.rpm = (int16_t)(velocity * 9.5493f); // rad/s to RPM
    motor_feedback.bottom_rpm = motor_feedback.rpm;
    
    // Torque in Nm -> store as mNm (millinewton-meters)
    motor_feedback.current = (int16_t)(torque * 1000.0f);
    
    // Temperature in °C
    motor_feedback.temperature = temperature;
    motor_feedback.last_update = current;
    
    // Track position changes for absolute position tracking
    int16_t angle_diff = motor_feedback.angle - prev_angle;
    if (angle_diff < -4000) {
        // Wrapped forward
        total_motor_counts += 8191 + angle_diff;
    } else if (angle_diff > 4000) {
        // Wrapped backward
        total_motor_counts += angle_diff - 8191;
    } else {
        // Normal increment
        total_motor_counts += angle_diff;
    }
    prev_angle = motor_feedback.angle;
    
    // Calculate top shaft angle (0-360° or -180 to 180°)
    float total_revs = (float)total_motor_counts / 8191.0f;
    motor_feedback.top_shaft_angle = fmodf(total_revs * 360.0f, 360.0f);
    if (motor_feedback.top_shaft_angle > 180.0f) {
        motor_feedback.top_shaft_angle -= 360.0f;
    }
    
    // Store error code for debugging
    // Error codes: 8=Overvoltage, 9=Undervoltage, A=Overcurrent, 
    //              B=MOS overtemp, C=Coil overtemp, D=Comm loss, E=Overload
    this->error_code = rx_error_code;
    (void)rx_motor_id;   // Should match our motor_id
}

const char* DMJ4310Functions::getErrorString() const {
    switch (error_code) {
        case 0x0: return "No Error";
        case 0x8: return "Overvoltage";
        case 0x9: return "Undervoltage";
        case 0xA: return "Overcurrent";
        case 0xB: return "MOS Overtemperature";
        case 0xC: return "Motor Coil Overtemperature";
        case 0xD: return "Communication Loss";
        case 0xE: return "Overload";
        default: return "Unknown Error";
    }
}

void DMJ4310Functions::sendCurrent(int16_t current) {
    // Simple current control using MIT protocol
    // Convert current to torque (assuming simple linear relationship)
    // For now, use torque feedforward with position/velocity = 0
    float torque = (float)current / 1000.0f; // Convert mA to estimated Nm
    sendMITCommand(0.0f, 0.0f, 0.0f, 0.0f, torque);
}

void DMJ4310Functions::sendMITCommand(float p_des, float v_des, float kp, float kd, float t_ff) {
    // MIT protocol encoding
    // Convert floats to scaled integers
    uint16_t p_int = floatToUint(p_des, p_min, p_max, 16);
    uint16_t v_int = floatToUint(v_des, v_min, v_max, 12);
    uint16_t kp_int = floatToUint(kp, 0.0f, 500.0f, 12);
    uint16_t kd_int = floatToUint(kd, 0.0f, 5.0f, 12);
    uint16_t t_int = floatToUint(t_ff, t_min, t_max, 12);
    
    // Pack into 8 bytes
    memset(data, 0, 8);
    data[0] = (p_int >> 8) & 0xFF;                    // p_des[15:8]
    data[1] = p_int & 0xFF;                           // p_des[7:0]
    data[2] = (v_int >> 4) & 0xFF;                    // v_des[11:4]
    data[3] = ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F);  // v_des[3:0] | Kp[11:8]
    data[4] = kp_int & 0xFF;                          // Kp[7:0]
    data[5] = (kd_int >> 4) & 0xFF;                   // Kd[11:4]
    data[6] = ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F);  // Kd[3:0] | t_ff[11:8]
    data[7] = t_int & 0xFF;                           // t_ff[7:0]
    
    Modules::DJIMotors::send(&txHeader, data);
}

void DMJ4310Functions::enableMotor() {
    // MIT Cheetah protocol: Enable motor command
    // Send: 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFC
    memset(data, 0xFF, 8);
    data[7] = 0xFC;
    Modules::DJIMotors::send(&txHeader, data);
    // Note: No delay - motor will respond when ready
}

void DMJ4310Functions::disableMotor() {
    // MIT Cheetah protocol: Disable motor command
    // Send: 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFD
    memset(data, 0xFF, 8);
    data[7] = 0xFD;
    Modules::DJIMotors::send(&txHeader, data);
}

void DMJ4310Functions::zeroPosition() {
    // MIT Cheetah protocol: Zero position command
    // Send: 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFE
    memset(data, 0xFF, 8);
    data[7] = 0xFE;
    Modules::DJIMotors::send(&txHeader, data);
    
    // Reset internal position tracking
    prev_angle = 0;
    total_motor_counts = 0;
}

void DMJ4310Functions::enableMotorBroadcast() {
    // Try enabling on multiple possible CAN IDs
    // MIT Mode: Send directly to motor's CAN_ID (not 0x100 + ID!)
    FDCAN_TxHeaderTypeDef tempHeader = txHeader;
    
    memset(data, 0xFF, 8);
    data[7] = 0xFC;  // Enable command
    
    // Try common CAN IDs: 1-8 (direct motor IDs in MIT mode)
    for (uint8_t id = 1; id <= 8; id++) {
        tempHeader.Identifier = id;  // MIT mode uses CAN_ID directly
        Modules::DJIMotors::send(&tempHeader, data);
    }
}

void DMJ4310Functions::setRanges(float p_min_val, float p_max_val, float v_min_val, float v_max_val, float t_min_val, float t_max_val) {
    p_min = p_min_val;
    p_max = p_max_val;
    v_min = v_min_val;
    v_max = v_max_val;
    t_min = t_min_val;
    t_max = t_max_val;
}

// Helper: Convert float to scaled unsigned int
uint16_t DMJ4310Functions::floatToUint(float x, float x_min, float x_max, int bits) {
    // Clamp value
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;
    
    // Scale to [0, 2^bits - 1]
    float span = x_max - x_min;
    float offset = x - x_min;
    uint32_t max_val = (1 << bits) - 1;
    return (uint16_t)((offset / span) * max_val);
}

// Helper: Convert scaled unsigned int to float
float DMJ4310Functions::uintToFloat(uint16_t x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    uint32_t max_val = (1 << bits) - 1;
    return ((float)x_int / max_val) * span + x_min;
}

// ========== GM6020 Implementation ==========
GM6020Functions::GM6020Functions(int id, pFDCAN_RxFifo0CallbackTypeDef callback,
                                 pFDCAN_ErrorStatusCallbackTypeDef errorCallback,
                                 uint16_t filterID2, uint16_t filterID1)
                                 : MotorFunctions(id, callback, errorCallback) {
    // GM6020 uses CAN IDs 0x205-0x20B (for motors 1-7)
    // GM6020 motors (ID 5-8)
    motor_id = 7;
    filter = Modules::DJIMotors::getFilter(0x205, 0x208); // Receive 0x205-0x208
    txHeader = Modules::DJIMotors::getTxHeader(id, Modules::DJIMotors::MotorType::GM6020);
    // GM6020 in current mode - uses voltage command format but interprets as current
    // Command range is still ±25000 (voltage range) but represents current
    // PID values for angle control (tuned to prevent overshoot/oscillation)
    RPM_KP = 30.0f;      // Reduced proportional gain to prevent overshoot
    RPM_KI = 0.01f;      // Very small integral to prevent windup
    RPM_KD = 0.5f;       // Reduced derivative for smooth damping
    MAX_CURRENT = 8000;  // Reduced current limit for smoother control
}

void GM6020Functions::readMotorFeedback(uint8_t rxData[8]) {
    int16_t current_angle = (rxData[0] << 8) | rxData[1];
    int32_t current = HAL_GetTick();
    motor_feedback.frequency = 1000.0f / (current - motor_feedback.last_update);
    
    // On first feedback, initialize prev_angle to avoid offset
    if(!initialized){
        initialized = true;
        prev_angle = current_angle;
    }
    int16_t angle_diff = current_angle - prev_angle;
    if (angle_diff < -4000) {
        // Positive wrap: 8191 → 0
        total_motor_counts += 8191 + angle_diff;
    } else if (angle_diff > 4000) {
        // Negative wrap: 0 → 8191  
        total_motor_counts += angle_diff - 8191;
    } else {
        total_motor_counts += angle_diff;
    }
    prev_angle = current_angle;
    // Store data
    motor_feedback.top_shaft_angle = total_motor_counts / 8191.0f * 360.0f;
    motor_feedback.angle = current_angle;
    motor_feedback.rpm = ((rxData[2] << 8) | rxData[3]);
    motor_feedback.current = (rxData[4] << 8) | rxData[5];
    motor_feedback.temperature = rxData[6];
    motor_feedback.last_update = current;
}



