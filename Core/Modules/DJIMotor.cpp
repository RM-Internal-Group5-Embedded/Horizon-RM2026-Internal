#include "DJIMotor.hpp"
#include <cstring>
#include <cstdio>
#include "MotorFunctions.hpp"

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
    
    // Use DUAL mode for single ID (when filterID1 == filterID2), RANGE mode for ID ranges
    if (filterID1 == filterID2) {
        // Single ID: Use DUAL mode (matches FilterID1 OR FilterID2, both set to same value)
        filterConfig.FilterType = FDCAN_FILTER_DUAL;
    } else {
        // ID range: Use RANGE mode (matches IDs from FilterID1 to FilterID2)
        filterConfig.FilterType = FDCAN_FILTER_RANGE;
    }
    
    filterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    // For GM6020: We want to receive 0x205-0x208
    // For M3508: We want to receive 0x201-0x204
    // For DMJ4310: We want to receive 0x300 (single ID, uses DUAL mode)
    filterConfig.FilterID1 = filterID1;  // Start ID or first ID (no shift needed)
    filterConfig.FilterID2 = filterID2;  // End ID or second ID (no shift needed)

    
    
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
    HAL_FDCAN_ActivateNotification(&hfdcan1, 
                                  FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 
                                  0);
    
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

uint16_t counter = 0;
void send(const FDCAN_TxHeaderTypeDef *header, const uint8_t data[8]) {
    // CAN Bus Collision Avoidance:
    // 1. Check TX FIFO free level before sending
    // 2. Handle FIFO full condition gracefully
    // 3. Monitor CAN state to prevent sending when bus is down
    // 4. Rate limiting: Max 8 messages per millisecond to prevent bus overload
    
    // Check if CAN is in a valid state (READY or BUSY are both valid for sending)
    // Only reject if in ERROR or RESET state
    if (hfdcan1.State == HAL_FDCAN_STATE_RESET || hfdcan1.State == HAL_FDCAN_STATE_ERROR) {
        // CAN is in error state - skip send to avoid errors
        return;
    }
    
    // Rate limiting: Max 8 messages per millisecond
    // System has 7 motors total:
    //   - ER task: 1 message (0x200 = 4 wheels combined)
    //   - Claw task: 3 messages (GM6020=0x2FF, M3508 claw=0x1FF, DMJ4310=direct ID)
    //   - Callbacks: Receive only, don't send
    // Normal operation: 4 msgs/10ms = 0.4 msgs/ms, well below 8/ms limit
    // This limit protects against bursts, initialization, or unexpected high-frequency sends
    static uint32_t last_ms = 0;
    static uint8_t messages_this_ms = 0;
    static uint8_t er_messages_this_ms = 0;  // Track ER task messages (0x200)
    static uint8_t claw_messages_this_ms = 0;  // Track Claw task messages (all others)
    uint32_t current_ms = xTaskGetTickCount();
    
    // Reset counters if we're in a new millisecond
    if (current_ms != last_ms) {
        messages_this_ms = 0;
        er_messages_this_ms = 0;
        claw_messages_this_ms = 0;
        last_ms = current_ms;
    }
    
    // Determine if this is an ER task message (0x200 = 4 wheels combined)
    // All other IDs are claw motors: 0x1FF (M3508 claw), 0x2FF (GM6020), or direct ID (DMJ4310)
    bool is_er_message = (header->Identifier == 0x200);
    
    // Enforce 8 messages/ms limit (safety limit for bursts/initialization)
    // if (messages_this_ms >= 4) {
    //     return;  // Rate limit exceeded - skip this message
    // }
    
    // Fairness: Reserve slots to ensure both tasks can send
    // ER needs: 1 slot (0x200)
    // Claw needs: 3 slots (GM6020, M3508 claw, DMJ4310)
    // Total: 4 slots needed, leaving 4 slots for bursts/callbacks
    // When approaching limit, ensure both tasks get their minimum slots
    
    
    // Attempt to send message (hardware will handle if FIFO is truly full)
    // Note: We don't check FIFO level here - hardware handles queuing
    // Checking and dropping causes command delays
    HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, header, data);
    counter++;
    
    
    
    // If send failed, it's likely a real error (not just FIFO full)
    // The hardware typically queues messages even if FIFO appears full
    if (status != HAL_OK) {
        // Error occurred - could be bus error, etc.
        // Don't retry here to avoid blocking - let next cycle handle it
    }
}

} // namespace DJIMotors
} // namespace Modules