/**
 * @file line_follower.hpp
 * @brief Line following algorithm for 3-sensor configuration
 * @details Implements basic line following using LEFT, MIDDLE, RIGHT IR sensors
 *          Line: 40mm black line on grey surface
 *          Sensor spacing: 10-15mm between L-M-R sensors
 * @version 0.1
 * @date 2025-10-29
 */

#pragma once

#include "main.h"
#include "gpio.h"
#include <stdint.h>

namespace LineFollower
{

/**
 * @brief Line following states based on 3-sensor configuration
 */
enum class LineState : uint8_t
{
    CENTERED = 0,    // L=0, M=1, R=0: Robot centered on line
    DRIFT_LEFT,      // L=1, M=1, R=0: Robot drifted left
    DRIFT_RIGHT,     // L=0, M=1, R=1: Robot drifted right
    FAR_LEFT,        // L=1, M=0, R=0: Robot far left of line
    FAR_RIGHT,       // L=0, M=0, R=1: Robot far right of line
    LINE_LOST        // L=0, M=0, R=0: Line lost
};

/**
 * @brief GPIO pin configuration for IR sensors
 * @note Pins configured via constructor, not hardcoded
 */
struct SensorPins
{
    GPIO_TypeDef* slot_port_;      // Slot detection sensor (not used in basic following)
    uint16_t slot_pin_;
    GPIO_TypeDef* left_port_;      // Left line sensor
    uint16_t left_pin_;
    GPIO_TypeDef* middle_port_;    // Middle line sensor
    uint16_t middle_pin_;
    GPIO_TypeDef* right_port_;     // Right line sensor
    uint16_t right_pin_;
};

/**
 * @brief Current sensor readings (digital: 0=no line, 1=line detected)
 */
struct SensorReadings
{
    uint8_t slot;     // Slot sensor (0 or 1)
    uint8_t left;     // Left sensor (0 or 1)
    uint8_t middle;   // Middle sensor (0 or 1)
    uint8_t right;    // Right sensor (0 or 1)
};

/**
 * @brief Motor control command output
 * @note Placeholder interface - actual motor control implemented elsewhere
 */
struct MotorCommand
{
    int16_t left_speed;    // Left motor speed (-1024 to 1024)
    int16_t right_speed;   // Right motor speed (-1024 to 1024)
};

/**
 * @brief Configuration constants
 */
constexpr int16_t NOMINAL_SPEED = 1024;           // Base speed for forward motion
constexpr float GENTLE_TURN_RATIO = 0.7f;         // Speed reduction for gentle turns
constexpr float SHARP_TURN_RATIO = 0.3f;          // Speed reduction for sharp turns
constexpr uint32_t LINE_FOLLOWER_UPDATE_RATE_MS = 20;  // 50Hz update rate

/**
 * @brief Line follower controller class
 */
class LineFollowerController
{
public:
    /**
     * @brief Debug information structure (public for Ozone live watch)
     */
    struct DebugInfo
    {
        LineState current_state_;
        SensorReadings sensor_readings_;
        MotorCommand motor_command_;
        uint32_t state_change_count_;   // Number of state transitions
        uint32_t line_lost_count_;      // Number of times line was lost
    };

    DebugInfo debug_info_;  // Public for debugger access

    /**
     * @brief Constructor
     * @param _pins GPIO pin configuration for sensors
     */
    explicit LineFollowerController(const SensorPins& _pins);

    /**
     * @brief Initialize the line follower
     * @note Call once during startup
     */
    void init();

    /**
     * @brief Read current sensor values
     * @return SensorReadings struct with current values
     */
    SensorReadings readSensors();

    /**
     * @brief Process line following algorithm
     * @details Updates internal state and calculates motor commands
     */
    void processLineFollowing();

    /**
     * @brief Get current motor command
     * @return MotorCommand struct with left/right motor speeds
     */
    MotorCommand getMotorCommand() const;

private:
    SensorPins pins_;                  // GPIO pin configuration
    SensorReadings sensor_readings_;   // Current sensor values
    LineState current_state_;          // Current line following state
    MotorCommand motor_command_;       // Current motor command
    LineState previous_state_;         // Previous state for change detection

    /**
     * @brief Determine line state from sensor readings
     * @param _readings Current sensor readings
     * @return Calculated LineState
     */
    LineState determineState(const SensorReadings& _readings);

    /**
     * @brief Calculate motor speeds based on current state
     * @param _state Current line following state
     * @return MotorCommand with calculated speeds
     */
    MotorCommand calculateMotorCommand(LineState _state);
};

}  // namespace LineFollower

