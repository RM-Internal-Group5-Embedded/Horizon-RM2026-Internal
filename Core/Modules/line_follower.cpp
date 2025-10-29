/**
 * @file line_follower.cpp
 * @brief Implementation of line following algorithm
 * @version 0.1
 * @date 2025-10-29
 */

#include "line_follower.hpp"

namespace LineFollower
{

/**
 * @brief Constructor - initialize with pin configuration
 */
LineFollowerController::LineFollowerController(const SensorPins& _pins)
    : pins_(_pins),
      sensor_readings_{0, 0, 0, 0},
      current_state_(LineState::LINE_LOST),
      motor_command_{0, 0},
      previous_state_(LineState::LINE_LOST)
{
    // Initialize debug info
    debug_info_.current_state_ = LineState::LINE_LOST;
    debug_info_.sensor_readings_ = {0, 0, 0, 0};
    debug_info_.motor_command_ = {0, 0};
    debug_info_.state_change_count_ = 0;
    debug_info_.line_lost_count_ = 0;
}

/**
 * @brief Initialize line follower (called once at startup)
 */
void LineFollowerController::init()
{
    // GPIO pins should already be configured by STM32CubeMX/HAL
    // This function is placeholder for future initialization needs
    
    // Reset state
    current_state_ = LineState::LINE_LOST;
    motor_command_ = {0, 0};
    
    // Read initial sensor values
    sensor_readings_ = readSensors();
}

/**
 * @brief Read current sensor values from GPIO pins
 * @return SensorReadings struct with current digital values
 */
SensorReadings LineFollowerController::readSensors()
{
    SensorReadings readings;
    
    // Read digital sensor values (GPIO_PIN_SET = 1, GPIO_PIN_RESET = 0)
    // Assuming sensors output HIGH when line is detected (dark line on light surface)
    readings.slot = (HAL_GPIO_ReadPin(pins_.slot_port_, pins_.slot_pin_) == GPIO_PIN_SET) ? 1 : 0;
    readings.left = (HAL_GPIO_ReadPin(pins_.left_port_, pins_.left_pin_) == GPIO_PIN_SET) ? 1 : 0;
    readings.middle = (HAL_GPIO_ReadPin(pins_.middle_port_, pins_.middle_pin_) == GPIO_PIN_SET) ? 1 : 0;
    readings.right = (HAL_GPIO_ReadPin(pins_.right_port_, pins_.right_pin_) == GPIO_PIN_SET) ? 1 : 0;
    
    return readings;
}

/**
 * @brief Process line following algorithm
 * @details Main control loop function - reads sensors, determines state, calculates motor command
 */
void LineFollowerController::processLineFollowing()
{
    // Read current sensor values
    sensor_readings_ = readSensors();
    
    // Determine current state from sensor readings
    LineState new_state = determineState(sensor_readings_);
    
    // Track state changes for debugging
    if (new_state != current_state_)
    {
        previous_state_ = current_state_;
        current_state_ = new_state;
        debug_info_.state_change_count_++;
        
        // Track line lost events
        if (new_state == LineState::LINE_LOST)
        {
            debug_info_.line_lost_count_++;
        }
    }
    
    // Calculate motor command based on current state
    motor_command_ = calculateMotorCommand(current_state_);
    
    // Update debug info for Ozone live watch
    debug_info_.current_state_ = current_state_;
    debug_info_.sensor_readings_ = sensor_readings_;
    debug_info_.motor_command_ = motor_command_;
}

/**
 * @brief Get current motor command
 * @return MotorCommand struct with motor speeds
 */
MotorCommand LineFollowerController::getMotorCommand() const
{
    return motor_command_;
}

/**
 * @brief Determine line following state from sensor readings
 * @param _readings Current sensor readings (L, M, R)
 * @return LineState enum value
 */
LineState LineFollowerController::determineState(const SensorReadings& _readings)
{
    // State determination based on 3-sensor configuration
    // Pattern: [LEFT, MIDDLE, RIGHT]
    
    if (_readings.left == 0 && _readings.middle == 1 && _readings.right == 0)
    {
        // [0, 1, 0] - Centered on line
        return LineState::CENTERED;
    }
    else if (_readings.left == 1 && _readings.middle == 1 && _readings.right == 0)
    {
        // [1, 1, 0] - Drifted left (line moved to the left)
        return LineState::DRIFT_LEFT;
    }
    else if (_readings.left == 0 && _readings.middle == 1 && _readings.right == 1)
    {
        // [0, 1, 1] - Drifted right (line moved to the right)
        return LineState::DRIFT_RIGHT;
    }
    else if (_readings.left == 1 && _readings.middle == 0 && _readings.right == 0)
    {
        // [1, 0, 0] - Far left of line
        return LineState::FAR_LEFT;
    }
    else if (_readings.left == 0 && _readings.middle == 0 && _readings.right == 1)
    {
        // [0, 0, 1] - Far right of line
        return LineState::FAR_RIGHT;
    }
    else if (_readings.left == 0 && _readings.middle == 0 && _readings.right == 0)
    {
        // [0, 0, 0] - Line lost
        return LineState::LINE_LOST;
    }
    else
    {
        // Unexpected pattern (e.g., [1, 1, 1] or [1, 0, 1])
        // Treat as line lost for safety
        return LineState::LINE_LOST;
    }
}

/**
 * @brief Calculate motor speeds based on current line following state
 * @param _state Current LineState
 * @return MotorCommand with left/right motor speeds
 */
MotorCommand LineFollowerController::calculateMotorCommand(LineState _state)
{
    MotorCommand cmd;
    
    switch (_state)
    {
        case LineState::CENTERED:
            // Go straight - both motors at nominal speed
            cmd.left_speed = NOMINAL_SPEED;
            cmd.right_speed = NOMINAL_SPEED;
            break;
            
        case LineState::DRIFT_LEFT:
            // Gentle turn right - reduce left motor speed
            cmd.left_speed = static_cast<int16_t>(NOMINAL_SPEED * GENTLE_TURN_RATIO);
            cmd.right_speed = NOMINAL_SPEED;
            break;
            
        case LineState::DRIFT_RIGHT:
            // Gentle turn left - reduce right motor speed
            cmd.left_speed = NOMINAL_SPEED;
            cmd.right_speed = static_cast<int16_t>(NOMINAL_SPEED * GENTLE_TURN_RATIO);
            break;
            
        case LineState::FAR_LEFT:
            // Sharp turn right - significantly reduce left motor
            cmd.left_speed = static_cast<int16_t>(NOMINAL_SPEED * SHARP_TURN_RATIO);
            cmd.right_speed = NOMINAL_SPEED;
            break;
            
        case LineState::FAR_RIGHT:
            // Sharp turn left - significantly reduce right motor
            cmd.left_speed = NOMINAL_SPEED;
            cmd.right_speed = static_cast<int16_t>(NOMINAL_SPEED * SHARP_TURN_RATIO);
            break;
            
        case LineState::LINE_LOST:
            // Stop both motors for safety
            cmd.left_speed = 0;
            cmd.right_speed = 0;
            break;
            
        default:
            // Safety fallback - stop
            cmd.left_speed = 0;
            cmd.right_speed = 0;
            break;
    }
    
    return cmd;
}

}  // namespace LineFollower

