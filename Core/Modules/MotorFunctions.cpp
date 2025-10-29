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
    RPM_KP = 100.0f;
    RPM_KI = 0.1f;
    RPM_KD = 20.0f;
    MAX_CURRENT = 10000;
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
    
    pid_output = setRpmPID(1000, dt);   
    pid_error = motor_feedback.target_angle - motor_feedback.top_shaft_angle;
    motor_feedback.output_current = (int16_t)pid_output;
    sendCurrent(motor_feedback.output_current);

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

    pid_output = setRpmPID(target_rpm, dt);   
    pid_error = motor_feedback.target_angle - motor_feedback.top_shaft_angle;
    motor_feedback.output_current = (int16_t)pid_output;

    sendCurrent(motor_feedback.output_current);
    last_time = current_time;
}

float MotorFunctions::setRpmPID(int16_t target_rpm, float dt){
    motor_feedback.target_rpm =  target_rpm;
    pid_output = Modules::PID::calculate(
        (float)motor_feedback.target_rpm,
        (float)motor_feedback.rpm,
        RPM_KP, RPM_KI, RPM_KD,
        pid_integral,
        pid_prev_error,
        dt
    );
    pid_output = Modules::PID::pidClampMinMax(pid_output, MAX_CURRENT);
    return pid_output;
}

void MotorFunctions::setAngle(float target_angle){
    current_time = HAL_GetTick();
    float dt = (current_time - last_time) / 1000.0f; // Update dt
    if (dt <= 0 || dt > 0.1f) dt = 0.01f; // Clamp

    pid_output = setAnglePID(target_angle, dt);   
    pid_error = motor_feedback.target_angle - motor_feedback.top_shaft_angle;
    motor_feedback.output_current = (int16_t)pid_output;

    sendCurrent(motor_feedback.output_current);
    last_time = current_time;
}

float MotorFunctions::setAnglePID(float target_angle, float dt){
    motor_feedback.target_angle =  target_angle;
    pid_output = Modules::PID::calculate(
        (float)motor_feedback.target_angle,
        (float)motor_feedback.top_shaft_angle,
        RPM_KP, RPM_KI, RPM_KD,
        pid_integral,
        pid_prev_error,
        dt
    );
    pid_output = Modules::PID::pidClampMinMax(pid_output, MAX_CURRENT);
    return pid_output;
}



