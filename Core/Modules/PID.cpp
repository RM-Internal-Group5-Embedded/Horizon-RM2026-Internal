#include "PID.hpp"

/*
Todo: Implement your functions or classes here
*/

namespace Modules {
namespace PID {

/**
 * @brief Calculate the PID output.
 * @param target The target value.
 * @param current The current value.
 * @param kp The proportional gain.
 * @param ki The integral gain.
 * @param kd The derivative gain.
 * @param integral The integral term (will be updated through PBR).
 * @param previousError The previous error (will be updated through PBR).
 * @param dt The time difference.
 * @return The PID output.
 */


float calculate(const float target, const float current, const float kp,
                const float ki, const float kd, float &integral,
                float &previousError, const float dt){

    float error = target - current;
    
    // Use the provided dt parameter instead of hardcoded 0.001f
    float valid_dt = dt;
    if (dt <= 0.0f || dt > 1.0f) {
        valid_dt = 0.001f; 
    }
    
    // Proportional term
    float p_output_ = kp * error;
    
    // Integral term with anti-windup clamping
    integral += error * valid_dt;
    // Clamp integral to prevent windup (typical range: ±10000)
    const float MAX_INTEGRAL = 10000.0f;
    if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
    else if (integral < -MAX_INTEGRAL) integral = -MAX_INTEGRAL;
    float i_output_ = ki * integral;
    
    // Derivative term - use valid_dt instead of hardcoded 0.001f
    float d_output_ = kd * (error - previousError) / valid_dt;

    float output = p_output_ + i_output_ + d_output_;
    
    // Update previous error for next iteration
    previousError = error;
    
    return output;
}

/**
 * @brief Reset PID state.
 * @param integral The integral term to be reset.
 * @param previousError The previous error term to be reset.
 */
void resetPIDState(float &integral, float &previousError){
    integral = 0.0f;
    previousError = 0.0f;
}

/**
 * @brief Clamp the input value to be within the range [-value, value].
 * @param input The input value.
 * @param value The absolute value limit.
 * @return The clamped value.
 */
float pidClampMinMax(const float input, const float value) {
    if (input > value) {
        return value;
    } else if (input < -value) {
        return -value;
    } else {
        return input;
    }
}

} // namespace PID

} // namespace Modules