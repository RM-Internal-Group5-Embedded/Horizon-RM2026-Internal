#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <cstring>
#include "gpio.h"
#include "main.h"

namespace jc24b
{

// 用户数据结构 - 四个位置的小方块状态
typedef struct _user_data
{
    uint8_t header1;        // 0xAA
    uint8_t header2;        // 0xBB  
    uint8_t position1;      // 位置1状态
    uint8_t position2;      // 位置2状态
    uint8_t position3;      // 位置3状态
    uint8_t position4;      // 位置4状态
    uint8_t checksum;       // 校验和
} user_data;

// 确认包结构 - 回复位置数据包
typedef struct _ack_data
{
    uint8_t header1;        // 0xCC
    uint8_t header2;        // 0xDD
    uint8_t status;         // 状态标志
    uint8_t checksum;       // 校验和
} ack_data;

class DataTransceiver
{
private:
    user_data rx_data_;              // 接收的原始数据
    ack_data ack_data_;              // 确认包数据
    UART_HandleTypeDef* huart_;      // 串口句柄
    uint8_t rx_buffer_[16];          // DMA接收缓冲区

    bool positions_[4];       // 位置状态
    uint8_t debounce_counters_[4];   // 去抖计数器
    
    // 连接状态
    bool connected_;             // 连接状态
    TickType_t last_rx_time_;    // 最后接收时间
    // 重连相关
    uint16_t reconnect_count_;   // 重连计数
    bool data_valid_;            // 数据有效标志
    TickType_t last_reconnect_time_; // 上次重连时间

    // 常量定义
    static constexpr TickType_t WATCHDOG_TIMEOUT = pdMS_TO_TICKS(500);
    static constexpr TickType_t RECONNECT_RETRY_INTERVAL = pdMS_TO_TICKS(200);
    static constexpr uint8_t DEBOUNCE_FRAMES = 3;    // 去抖阈值

    // 私有方法
    uint8_t calculateChecksum(const uint8_t* data, uint8_t len);
    bool verifyChecksum(const uint8_t* data, uint8_t len, uint8_t checksum);
    bool validateData(const user_data& data);
    void refreshWatchdog();
    void checkWatchdog();        // 检查看门狗（在process中调用）
    void disconnect();           // 断连处理
    void tryReconnect();         // 尝试重连
    void resetDebouncedState();  // 重置去抖状态
    void updateDebouncedPositions(const user_data& data); // 更新稳定位置状态

public:
    DataTransceiver();
    ~DataTransceiver();

    // 初始化
    void init(UART_HandleTypeDef* huart);
    
    // 发送确认包
    bool send_ack(uint8_t status = 0);
    
    // 外部接口 - 获取位置数据（安全接口）
    bool getPositions(bool (&positions)[4]);  // 返回false表示数据无效
    
    // 连接状态查询
    bool isConnected() const;
    
    // 获取重连次数
    uint16_t getReconnectCount() const;
    
    // 接收回调（DMA中断中调用）
    void RxEventCallback(UART_HandleTypeDef* huart, uint16_t size);
    
    // 主循环重复调用
    void process();

};

// C风格回调
extern "C" void TransceiverRxEventCallback(UART_HandleTypeDef* huart, uint16_t size);

} // namespace jc24b