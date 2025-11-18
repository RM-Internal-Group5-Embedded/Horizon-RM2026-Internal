
#include <cstring>
#include "jc24btran.hpp"
namespace jc24b
{
    // 静态实例指针，用于回调函数访问
static DataTransceiver* g_transceiver_instance = nullptr;

// 静态回调桥接函数（C风格，用于HAL回调）
extern "C" void TransceiverRxEventCallback(UART_HandleTypeDef* huart, uint16_t size) {
    if(g_transceiver_instance != nullptr) {
        g_transceiver_instance->RxEventCallback(huart, size);
    }
}

// 构造函数
DataTransceiver::DataTransceiver() 
    : huart_(nullptr), 
      connected_(false),
      last_rx_time_(0),
      reconnect_count_(0),
      data_valid_(false),
      last_reconnect_time_(0)
{
    memset(&rx_data_, 0, sizeof(user_data));
    memset(&ack_data_, 0, sizeof(ack_data));
    memset(rx_buffer_, 0, sizeof(rx_buffer_));
    resetDebouncedState();
}

// 析构函数
DataTransceiver::~DataTransceiver() {
    if (g_transceiver_instance == this) {
        g_transceiver_instance = nullptr;
    }
}

// 计算校验和
uint8_t DataTransceiver::calculateChecksum(const uint8_t* data, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

// 验证校验和
bool DataTransceiver::verifyChecksum(const uint8_t* data, uint8_t len, uint8_t checksum) {
    return (calculateChecksum(data, len) == checksum);
}

// 验证位置数据包
bool DataTransceiver::validateData(const user_data& data) {
    // 检查包头
    if (data.header1 != 0xAA || data.header2 != 0xBB) {
        return false;
    }
    
    // 验证校验和
    uint8_t* data_ptr = (uint8_t*)&data;
    if (!verifyChecksum(data_ptr, sizeof(user_data) - 1, data.checksum)) {
        return false;
    }
    
    return true;
}

void DataTransceiver::resetDebouncedState() {
    memset(positions_, 0, sizeof(positions_));
    memset(debounce_counters_, 0, sizeof(debounce_counters_));
}

void DataTransceiver::updateDebouncedPositions(const user_data& data) {
    const uint8_t raw_positions[4] = {
        static_cast<uint8_t>(data.position1 != 0),
        static_cast<uint8_t>(data.position2 != 0),
        static_cast<uint8_t>(data.position3 != 0),
        static_cast<uint8_t>(data.position4 != 0)
    };

    if (DEBOUNCE_FRAMES <= 1) {
        for (uint8_t i = 0; i < 4; ++i) {
            positions_[i] = (raw_positions[i] != 0);
            debounce_counters_[i] = raw_positions[i] != 0 ? 1 : 0;
        }
        return;
    }

    for (uint8_t i = 0; i < 4; ++i) {
        // 需要连续 DEBOUNCE_FRAMES 次同样的非零输入才会置位，
        // 因此 PC 端必须持续周期性发送最新状态，而非仅在事件发生时单发一帧
        if (raw_positions[i] != 0) {
            if (debounce_counters_[i] < DEBOUNCE_FRAMES) {
                debounce_counters_[i]++;
            }
        } else {
            if (debounce_counters_[i] > 0) {
                debounce_counters_[i]--;
            }
        }

        positions_[i] = (debounce_counters_[i] >= DEBOUNCE_FRAMES);
    }
}

// 刷新看门狗
void DataTransceiver::refreshWatchdog() {
    last_rx_time_ = xTaskGetTickCount();
    connected_ = true;
    data_valid_ = true;
}

// 检查看门狗
void DataTransceiver::checkWatchdog() {
    if (!connected_) {
        return;
    }
    
    TickType_t current_time = xTaskGetTickCount();
    TickType_t elapsed = current_time - last_rx_time_;
    
    // 超时检测
    if (elapsed > WATCHDOG_TIMEOUT) {
        disconnect();
    }
}

// 断连处理
void DataTransceiver::disconnect() {
    connected_ = false;
    data_valid_ = false;
    
    // 清空接收数据
    memset(rx_buffer_, 0, sizeof(rx_buffer_));
    memset(&rx_data_, 0, sizeof(user_data));
    resetDebouncedState();
    
    // 尝试重连
    // 立即标记为需要重连
    last_reconnect_time_ = xTaskGetTickCount() - RECONNECT_RETRY_INTERVAL;
    tryReconnect();
}

// 尝试重连
void DataTransceiver::tryReconnect() {
    if (huart_ == nullptr) {
        return;
    }

    reconnect_count_++;
    last_reconnect_time_ = xTaskGetTickCount();

    //终止当前接收并停止DMA
    HAL_UART_AbortReceive_IT(huart_);
    if (huart_->hdmarx != nullptr) {
        HAL_DMA_Abort(huart_->hdmarx);
    }

    //清理UART错误标志，防止溢出或帧错误导致的死锁
    __HAL_UART_CLEAR_FLAG(huart_, UART_CLEAR_OREF);
    __HAL_UART_CLEAR_FLAG(huart_, UART_CLEAR_NEF);
    __HAL_UART_CLEAR_FLAG(huart_, UART_CLEAR_FEF);
    __HAL_UART_CLEAR_FLAG(huart_, UART_CLEAR_PEF);

    //清空缓存，避免旧数据被误用
    memset(rx_buffer_, 0, sizeof(rx_buffer_));
    memset(&rx_data_, 0, sizeof(user_data));

    //重新启动DMA接收
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_buffer_, sizeof(rx_buffer_));
    if (status == HAL_OK && huart_->hdmarx != nullptr) {
        __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
    }
}

// 初始化
void DataTransceiver::init(UART_HandleTypeDef* huart) {
    huart_ = huart;
    g_transceiver_instance = this;
    
    // 注册回调
    HAL_UART_RegisterRxEventCallback(huart_, TransceiverRxEventCallback);
    
    // 初始化时间戳
    last_rx_time_ = xTaskGetTickCount();
    connected_ = false;
    data_valid_ = false;
    reconnect_count_ = 0;
    last_reconnect_time_ = xTaskGetTickCount();
    resetDebouncedState();
    
    // 启动DMA接收
    if (huart_ != nullptr) {
        HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_buffer_, sizeof(rx_buffer_));
        __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
    }

}

// 发送确认包
bool DataTransceiver::send_ack(uint8_t status) {
    if (huart_ == nullptr) {
        return false;
    }
    
    // 构建确认包
    ack_data_.header1 = 0xCC;
    ack_data_.header2 = 0xDD;
    ack_data_.status = status;
    
    // 计算校验和
    uint8_t* data_ptr = (uint8_t*)&ack_data_;
    ack_data_.checksum = calculateChecksum(data_ptr, sizeof(ack_data) - 1);
    
    // 发送
    HAL_StatusTypeDef result = HAL_UART_Transmit(huart_, data_ptr, sizeof(ack_data), 100);
    
    return (result == HAL_OK);
}

// 获取位置数据（安全接口）
bool DataTransceiver::getPositions(bool positions[4]) {
    if (!connected_ || !data_valid_) {
        // 数据无效，清零输出
        memset(positions, 0, 4);
        return false;
    }
    
    // 复制去抖后的稳定位置数据
    positions[0] = positions_[0];
    positions[1] = positions_[1];
    positions[2] = positions_[2];
    positions[3] = positions_[3];
    
    return true;
}

// 查询连接状态
bool DataTransceiver::isConnected() const {
    return connected_;
}

// 获取重连次数
uint16_t DataTransceiver::getReconnectCount() const {
    return reconnect_count_;
}

// DMA接收回调
void DataTransceiver::RxEventCallback(UART_HandleTypeDef* huart, uint16_t size) {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);  // PA1闪烁表示进入回调，仅作指示
    
    if (huart_ != nullptr && huart == huart_) {
        // 检查接收长度
        if (size != sizeof(user_data)) {
            // 重启DMA
            HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_buffer_, sizeof(rx_buffer_));
            __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
            return;
        }
        
        // 复制到临时结构
        user_data temp_data;
        memcpy(&temp_data, rx_buffer_, sizeof(user_data));
        
        // 验证数据
        if (validateData(temp_data)) {
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
            
            // 数据有效，更新接收数据
            memcpy(&rx_data_, &temp_data, sizeof(user_data));
            updateDebouncedPositions(rx_data_);
            
            // 刷新看门狗
            refreshWatchdog();
            
            // 发送确认包
            send_ack(0x00);  // 状态0表示正常
            
            // 重连成功，清零计数
            if (reconnect_count_ > 0) {
                reconnect_count_ = 0;
            }
        }
        
        HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_buffer_, sizeof(rx_buffer_));
        __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
    }
}

// 主循环处理
void DataTransceiver::process() {
    TickType_t now = xTaskGetTickCount();

    if (connected_) {
        // 检查看门狗超时
        checkWatchdog();
    } else {
        // 周期性尝试重连
        if ((now - last_reconnect_time_) >= RECONNECT_RETRY_INTERVAL) {
            tryReconnect();
        }
    }
}

} // namespace jc24b
