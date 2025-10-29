// Todo: Declare your functions or classes(Bonus) here

// You may modify this file to a more OOP method if you prefer

// This is just a starter code to make your life easier. You may and should
// modify it to add functionalities.

#pragma once

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
                float &previousError, const float dt);

/**
 * @brief Reset PID state.
 * @param integral The integral term to be reset.
 * @param previousError The previous error term to be reset.
 */
void resetPIDState(float &integral, float &previousError);

/**
 * @brief Clamp the input value to be within the range [-value, value].
 * @param input The input value.
 * @param value The absolute value limit.
 * @return The clamped value.
 */
float pidClampMinMax(const float input, const float value);

} // namespace PID

} // namespace Modules