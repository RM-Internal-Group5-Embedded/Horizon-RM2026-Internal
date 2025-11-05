#include "DJIMotor.hpp"
#include <cstring>
#include <cstdio>

// Make sure these are declared as extern
extern FDCAN_HandleTypeDef hfdcan1;
extern UART_HandleTypeDef huart1;

namespace Modules {
namespace DJIMotors {


//MotorFeedback motor_feedback[8] = {0};


// void parseMotorFeedback(uint16_t can_id, uint8_t data[8]) {
//         if (can_id < 0x201 || can_id > 0x208) return;
        
//         uint8_t motor_id = can_id - 0x201;
        
//         motor_feedback[motor_id].angle = (data[0] << 8) | data[1];
//         motor_feedback[motor_id].rpm = (data[2] << 8) | data[3];
//         motor_feedback[motor_id].current = (data[4] << 8) | data[5];
//         motor_feedback[motor_id].temperature = data[6];
//         motor_feedback[motor_id].last_update = HAL_GetTick();
        
//         // Debug output
//         char msg[100];
//         sprintf(msg, "M%d: Angle=%d, RPM=%d, Current=%d, Temp=%d°C\r\n",
//                 motor_id + 1, 
//                 motor_feedback[motor_id].angle,
//                 motor_feedback[motor_id].rpm,
//                 motor_feedback[motor_id].current,
//                 motor_feedback[motor_id].temperature);
//         HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 100);
//     }


    /**
 * @brief Returns the appropriate filter configuration based on the provided
 * parameters.
 * @param filterID2 The second filter ID.
 * @param filterID1 The first filter ID.
 * @param type The type of filter (standard or extended).
 * @param config The filter configuration (list or range).
 * @return The corresponding FDCAN filter configuration.
 */
FDCAN_FilterTypeDef getFilter(uint16_t filterID2, uint16_t filterID1) {
    FDCAN_FilterTypeDef filterConfig;
    
    filterConfig.IdType = FDCAN_STANDARD_ID;
    filterConfig.FilterIndex = 0;
    filterConfig.FilterType = FDCAN_FILTER_RANGE;  // Changed to range mode
    filterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    // For GM6020: We want to receive 0x205-0x208
    // For M3508: We want to receive 0x201-0x204
    filterConfig.FilterID1 = filterID1;  // Start ID (no shift needed)
    filterConfig.FilterID2 = filterID2;  // End ID (no shift needed)

    
    
    return filterConfig;
}


FDCAN_TxHeaderTypeDef getTxHeader(uint8_t id, MotorType type) {
    FDCAN_TxHeaderTypeDef txHeader;
    
    txHeader.Identifier = 0;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    
    
    switch(type) {
        case MotorType::GM6020:
            // GM6020 motors: IDs 1-4 use 0x1FF, IDs 5-8 use 0x2FF
            if (id >= 1 && id <= 4) {
                txHeader.Identifier = 0x1FF;  // Motors 1-4
            } else if (id >= 5 && id <= 8) {
                txHeader.Identifier = 0x2FF;  // Motors 5-8
            } else {
                txHeader.Identifier = 0x1FF;  // Default fallback
            }
            break;
        case MotorType::M3508:
            // M3508 motors: IDs 1-4 use 0x200, IDs 5-8 use 0x1FF
            if (id >= 1 && id <= 4) {
                txHeader.Identifier = 0x200;  // Motors 1-4 (feedback 0x201-0x204)
            } else if (id >= 5 && id <= 8) {
                txHeader.Identifier = 0x1FF;  // Motors 5-8 (feedback 0x205-0x208)
            } else {
                txHeader.Identifier = 0x200;  // Default fallback
            }
            break;
        case MotorType::DMJ4310:
            // DMJ4310 MIT Mode: TX = motor's CAN_ID directly
            // The CAN_ID must be configured via debug assistant
            // Default is usually the motor ID (1-8)
            txHeader.Identifier = id;  // Send to motor's CAN_ID directly
            break;
        default:
            txHeader.Identifier = 0x200;
            break;
    }
    
    return txHeader;
}

void init(pFDCAN_RxFifo0CallbackTypeDef callback, 
    pFDCAN_ErrorStatusCallbackTypeDef errorCallback, 
    FDCAN_FilterTypeDef* filter) {
    // Configure filter
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, filter) != HAL_OK) {
        Error_Handler();
    }
    // Configure global filter
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, 
                                FDCAN_ACCEPT_IN_RX_FIFO0, 
                                FDCAN_REJECT, 
                                FDCAN_FILTER_REMOTE, 
                                FDCAN_FILTER_REMOTE);
    // Activate notification
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    
    // Register callbacks
    HAL_FDCAN_RegisterRxFifo0Callback(&hfdcan1, callback);
    HAL_FDCAN_RegisterErrorStatusCallback(&hfdcan1, errorCallback);
}

void constructTxData(uint8_t data[8], const uint8_t id, const int16_t current) {
    memset(data, 0, 8);
    if (id >= 1 && id <= 4) {
        // Motors 1-4 (offset 0-3 in data array)
        uint8_t byte_offset = (id - 1) * 2;
        data[byte_offset] = (current >> 8) & 0xFF;
        data[byte_offset + 1] = current & 0xFF;
    } else if (id >= 5 && id <= 8) {
        // Motors 5-8 (offset 0-3 in data array, subtract 5)
        uint8_t byte_offset = (id - 5) * 2;
        data[byte_offset] = (current >> 8) & 0xFF;
        data[byte_offset + 1] = current & 0xFF;
    }  
}


void send(const FDCAN_TxHeaderTypeDef *header, const uint8_t data[8]) {
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, header, data);
}

} // namespace DJIMotors
} // namespace Modules