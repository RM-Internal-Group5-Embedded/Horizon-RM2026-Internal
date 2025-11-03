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
    
    motor_feedback.top_shaft_angle = (fmodf(top_shaft_revs * 360.0f, 360.0f)>180) ? 
        (fmodf(top_shaft_revs * 360.0f, 360.0f)-360.0f) : 
        fmodf(top_shaft_revs * 360.0f, 360.0f);
}

void MotorFunctions::sendCurrent(int16_t current) {
    Modules::DJIMotors::constructTxData(data, motor_id, current);
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
    // Motor IDs 1-4 map to byte offsets 0,2,4,6
    // Motor IDs 5-8 would use a different CAN ID (0x1FF) in real system
    int offset = (motor_id - 1) * 2;
    if (offset >= 0 && offset < 8) {
        data[offset] = (current >> 8) & 0xFF;
        data[offset + 1] = current & 0xFF;
    }
}

// ========== DMJ4310 Implementation ==========
DMJ4310Functions::DMJ4310Functions(int id, pFDCAN_RxFifo0CallbackTypeDef callback,
                                   pFDCAN_ErrorStatusCallbackTypeDef errorCallback,
                                   uint16_t filterID2, uint16_t filterID1)
                                   : MotorFunctions(id, callback, errorCallback) {
    // DMJ4310 is a planetary gearbox servo motor (similar characteristics to M3508)
    filter = Modules::DJIMotors::getFilter(filterID2, filterID1);
    txHeader = Modules::DJIMotors::getTxHeader(id, Modules::DJIMotors::MotorType::M3508);
    
    // DMJ4310-specific PID tuning
    RPM_KP = 25.0f;    // Moderate P gain
    RPM_KI = 0.05f;    // Low integral to prevent windup
    RPM_KD = 1.0f;     // Moderate derivative
    MAX_CURRENT = 10000;  // Conservative current limit
}

void DMJ4310Functions::ReadMotorFeedback(uint8_t rxData[8]) {
    // DMJ4310 uses standard M3508-like feedback format
    int16_t current_angle = (rxData[0] << 8) | rxData[1];
    int32_t current = HAL_GetTick();
    motor_feedback.frequency = 
        1000.0f / (current - motor_feedback.last_update);
    
    // Handle wrap-around to track total motor rotation
    int16_t angle_diff = current_angle - prev_angle;
    if (angle_diff < -4000) {
        total_motor_counts += 8191 + angle_diff;
    } else if (angle_diff > 4000) {
        total_motor_counts += angle_diff - 8191;
    } else {
        total_motor_counts += angle_diff;
    }
    prev_angle = current_angle;
    
    // Store data
    motor_feedback.angle = current_angle;
    motor_feedback.bottom_rpm = ((rxData[2] << 8) | rxData[3]);
    motor_feedback.rpm = motor_feedback.bottom_rpm;  // DMJ4310: 1:1 ratio assumed
    motor_feedback.current = (rxData[4] << 8) | rxData[5];
    motor_feedback.temperature = rxData[6];
    motor_feedback.last_update = current;
    
    // Calculate top shaft angle (assuming 1:1 for now, adjust if geared)
    float total_revs = (float)total_motor_counts / 8191.0f;
    motor_feedback.top_shaft_angle = fmodf(total_revs * 360.0f, 360.0f);
    if (motor_feedback.top_shaft_angle > 180.0f) {
        motor_feedback.top_shaft_angle -= 360.0f;
    }
}

// ========== GM6020 Implementation ==========
GM6020Functions::GM6020Functions(int id, pFDCAN_RxFifo0CallbackTypeDef callback,
                                 pFDCAN_ErrorStatusCallbackTypeDef errorCallback,
                                 uint16_t filterID2, uint16_t filterID1)
                                 : MotorFunctions(id, callback, errorCallback) {
    // GM6020 uses CAN IDs 0x205-0x20B (for motors 1-7)
    // GM6020 motors (ID 5-8)
    filter = Modules::DJIMotors::getFilter(0x205, 0x208); // Receive 0x205-0x208
    txHeader = Modules::DJIMotors::getTxHeader(id, Modules::DJIMotors::MotorType::GM6020);
    // Adjusted PID values for GM6020
    RPM_KP = 30.0f;      // Lower for position control
    RPM_KI = 3.0f;       // Small integral
    RPM_KD = 0.0f;       // No derivative for now
    MAX_CURRENT = 3000;   // GM6020 has lower current limit
}

void GM6020Functions::readMotorFeedback(uint8_t rxData[8]) {
    // GM6020 feedback format
    int16_t current_angle = (rxData[0] << 8) | rxData[1];
    int32_t current = HAL_GetTick();
    motor_feedback.frequency = 
        1000.0f / (current - motor_feedback.last_update);
    
    // Track total rotation with wrap-around handling
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
    
    // Set reference position on first feedback
    if (!reference_set) {
        reference_counts = total_motor_counts;
        absolute_counts = 0;
        reference_set = true;
    } else {
        // Calculate position relative to reference (rest position)
        absolute_counts = total_motor_counts - reference_counts;
    }
    
    // GM6020: Direct drive (1:1 ratio) - NO GEAR REDUCTION!
    // Current position in -180 to 180 range
    float raw_degrees = (current_angle / 8191.0f) * 360.0f;
    motor_feedback.top_shaft_angle = (raw_degrees > 180.0f) ? (raw_degrees - 360.0f) : raw_degrees;
    
    // Store data
    motor_feedback.angle = current_angle;
    motor_feedback.rpm = ((rxData[2] << 8) | rxData[3]);
    motor_feedback.current = (rxData[4] << 8) | rxData[5];
    motor_feedback.temperature = rxData[6];
    motor_feedback.last_update = current;
}



