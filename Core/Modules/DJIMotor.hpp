// Todo: Declare your functions or classes here

// You need to enable FDCAN in CubeMX and generate the code first

// You may modify this file to a more OOP method if you prefer

// This is just a starter code to make your life easier. You may and should
// modify it to add functionalities.

#pragma once
#include "fdcan.h"

namespace Modules {
namespace DJIMotors {

// Clean motor feedback structure - essential feedback data only
struct MotorFeedback {
    // Position
    int16_t angle;             // Current encoder angle (raw counts or scaled)
    float top_shaft_angle;     // Accumulated angle (degrees for GM6020/M3508, radians for DMJ4310)
    
    // Velocity
    int16_t rpm;               // Current speed (RPM)
    int16_t bottomrpm;
    
    // Torque/Current
    int16_t current;           // Current torque or current
    
    // Status
    uint8_t temperature;       // Motor temperature (°C)
    uint32_t last_update;      // Last feedback timestamp (ms)
};

enum class MotorType { GM6020 = 0, M3508 = 1, DMJ4310 = 2 };

//void parseMotorFeedback(uint16_t can_id, uint8_t data[8]);

/**
 * @brief Returns the appropriate filter configuration based on the provided
 * parameters.
 * @param filterID2 The second filter ID.
 * @param filterID1 The first filter ID.
 * @param type The type of filter (standard or extended).
 * @param config The filter configuration (list or range).
 * @return The corresponding FDCAN filter configuration.
 */
FDCAN_FilterTypeDef getFilter(uint16_t filterID2, uint16_t filterID1);

/**
 * @brief Returns the appropriate Tx header configuration based on the provided
 * parameters.
 * @param id The ID of the motor.
 * @param type The type of motor (GM6020 or M3508). You should use this to
 * determine tx header id. (0x1FF/0x2FF/0x200)
 * @return The corresponding FDCAN Tx header configuration.
 */
FDCAN_TxHeaderTypeDef getTxHeader(uint8_t id, MotorType type);

/**
 * @brief Initializes the FDCAN module with the specified callback function.
 * @param callback The callback function to be called on receiving data.
 * @param filter The filter configuration to be applied.
 * @note Please note that both types of motors are using the same callback.
 * @note You can modify this function to use different callbacks, but you have
 * to do research on your own.
 */
void init(pFDCAN_RxFifo0CallbackTypeDef callback, pFDCAN_ErrorStatusCallbackTypeDef errorCallback, FDCAN_FilterTypeDef* filter);

/**
 * @brief Constructs the transmission data for the motor.
 * @param data The array to hold the constructed data (must be of size 8).
 * @param id The ID of the motor.
 * @param current The current value to be sent to the motor.
 * @param type The type of motor (GM6020 or M3508). You should use this to
 * determine tx header id. (0x1FF/0x2FF/0x200)
 */
void constructTxData(uint8_t data[8], const uint8_t id, const int16_t current);

/**
 * @brief Sends the constructed data using the specified Tx header.
 * @param header The Tx header to be used for sending data.
 * @param data The array containing the data to be sent (must be of size 8).
 */
void send(const FDCAN_TxHeaderTypeDef *header, const uint8_t data[8]);
} // namespace DJIMotors
} // namespace Modules